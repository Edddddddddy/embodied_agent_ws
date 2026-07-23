#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

#include <embodied_agent_interfaces/msg/lidar_loop_constraint_decision.hpp>
#include <embodied_agent_interfaces/msg/lidar_loop_constraint_result.hpp>

#include "embodied_agent_middleware/qos_profiles.hpp"
#include "embodied_slam/loop_constraint_adapter.hpp"
#include "embodied_slam/loop_frontend_diagnostics.hpp"
#include "rclcpp/rclcpp.hpp"
#include "slam_toolbox/slam_toolbox_async.hpp"

namespace embodied_slam
{
namespace
{

std::string jsonEscape(const std::string & value)
{
  std::ostringstream output;
  for (const char character : value) {
    switch (character) {
      case '\\': output << "\\\\"; break;
      case '"': output << "\\\""; break;
      case '\n': output << "\\n"; break;
      case '\r': output << "\\r"; break;
      case '\t': output << "\\t"; break;
      default: output << character; break;
    }
  }
  return output.str();
}

void writeOptional(std::ostream & output, const std::optional<double> & value)
{
  if (value.has_value()) {
    output << std::setprecision(12) << *value;
  } else {
    output << "null";
  }
}

Pose2d fromKarto(const karto::Pose2 & pose)
{
  return {pose.GetX(), pose.GetY(), pose.GetHeading()};
}

karto::Pose2 toKarto(const Pose2d & pose)
{
  return karto::Pose2(pose.x, pose.y, pose.yaw);
}

bool positiveDefiniteCovariance(const std::array<double, 9U> & covariance)
{
  if (!std::all_of(
      covariance.begin(), covariance.end(),
      [](const double value) {return std::isfinite(value);}))
  {
    return false;
  }
  constexpr double tolerance = 1.0e-9;
  if (std::abs(covariance[1] - covariance[3]) > tolerance ||
    std::abs(covariance[2] - covariance[6]) > tolerance ||
    std::abs(covariance[5] - covariance[7]) > tolerance)
  {
    return false;
  }
  // Sylvester 判据阻止非正定协方差进入优化器，避免错误信息矩阵破坏求解。
  const double leading_1 = covariance[0];
  const double leading_2 = covariance[0] * covariance[4] - covariance[1] * covariance[3];
  const double determinant =
    covariance[0] * (covariance[4] * covariance[8] - covariance[5] * covariance[7]) -
    covariance[1] * (covariance[3] * covariance[8] - covariance[5] * covariance[6]) +
    covariance[2] * (covariance[3] * covariance[7] - covariance[4] * covariance[6]);
  return leading_1 > tolerance && leading_2 > tolerance && determinant > tolerance;
}

}  // namespace

class InstrumentedAsynchronousSlamToolbox :
  public slam_toolbox::AsynchronousSlamToolbox,
  public karto::MapperLoopClosureListener
{
public:
  explicit InstrumentedAsynchronousSlamToolbox(const rclcpp::NodeOptions & options)
  : slam_toolbox::AsynchronousSlamToolbox(options)
  {
    const char * environment_path = std::getenv("EMBODIED_SLAM_FRONTEND_LOG");
    const std::string default_path = environment_path == nullptr ? "" : environment_path;
    declare_parameter<std::string>("loop_frontend_log_path", default_path);
    declare_parameter<bool>("external_loop_constraint_enabled", false);
    declare_parameter<std::string>(
      "external_loop_constraint_topic", "/slam/loop_constraint_decisions");
    declare_parameter<std::string>(
      "external_loop_constraint_result_topic", "/slam/loop_constraint_results");
    declare_parameter<double>("external_loop_constraint_max_stamp_delta_ms", 25.0);
  }

  ~InstrumentedAsynchronousSlamToolbox() override
  {
    detachListener();
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override
  {
    const auto result = slam_toolbox::AsynchronousSlamToolbox::on_configure(state);
    if (result != CallbackReturn::SUCCESS) {
      return result;
    }
    try {
      const std::string path = get_parameter("loop_frontend_log_path").as_string();
      if (path.empty()) {
        RCLCPP_INFO(get_logger(), "Loop frontend trace disabled (empty path)");
      } else {
        trace_.open(path, std::ios::out | std::ios::trunc);
        if (!trace_.is_open()) {
          throw std::runtime_error("cannot open loop frontend trace: " + path);
        }
        attached_mapper_ = smapper_->getMapper();
        attached_mapper_->AddListener(this);
        RCLCPP_INFO(get_logger(), "Loop frontend trace -> %s", path.c_str());
      }

      external_constraint_enabled_ =
        get_parameter("external_loop_constraint_enabled").as_bool();
      const double maximum_delta_ms =
        get_parameter("external_loop_constraint_max_stamp_delta_ms").as_double();
      if (!std::isfinite(maximum_delta_ms) || maximum_delta_ms <= 0.0) {
        throw std::invalid_argument(
                "external_loop_constraint_max_stamp_delta_ms must be positive");
      }
      external_constraint_max_stamp_delta_ns_ =
        static_cast<std::int64_t>(maximum_delta_ms * 1.0e6);
      if (external_constraint_enabled_) {
        external_constraint_result_publisher_ =
          create_publisher<embodied_agent_interfaces::msg::LidarLoopConstraintResult>(
          get_parameter("external_loop_constraint_result_topic").as_string(),
          embodied_agent_middleware::event_qos());
        external_constraint_subscription_ =
          create_subscription<embodied_agent_interfaces::msg::LidarLoopConstraintDecision>(
          get_parameter("external_loop_constraint_topic").as_string(),
          embodied_agent_middleware::event_qos(),
          [this](
            const embodied_agent_interfaces::msg::LidarLoopConstraintDecision::SharedPtr message)
          {onExternalLoopConstraint(*message);});
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "instrumentation configure failed: %s", error.what());
      resetExternalConstraintRuntime();
      detachListener();
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(
      get_logger(), "external loop-constraint commit=%s",
      external_constraint_enabled_ ? "enabled (experimental)" : "disabled");
    return result;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override
  {
    const auto result = slam_toolbox::AsynchronousSlamToolbox::on_activate(state);
    if (result == CallbackReturn::SUCCESS) {
      external_constraint_active_ = true;
      if (external_constraint_result_publisher_) {
        external_constraint_result_publisher_->on_activate();
      }
    }
    return result;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override
  {
    external_constraint_active_ = false;
    if (external_constraint_result_publisher_) {
      external_constraint_result_publisher_->on_deactivate();
    }
    return slam_toolbox::AsynchronousSlamToolbox::on_deactivate(state);
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    external_constraint_active_ = false;
    resetExternalConstraintRuntime();
    detachListener();
    return slam_toolbox::AsynchronousSlamToolbox::on_cleanup(state);
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
  {
    external_constraint_active_ = false;
    resetExternalConstraintRuntime();
    detachListener();
    return slam_toolbox::AsynchronousSlamToolbox::on_shutdown(state);
  }

protected:
  karto::LocalizedRangeScan * addScan(
    karto::LaserRangeFinder * laser,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & scan,
    karto::Pose2 & karto_pose) override
  {
    karto::Mapper * mapper = smapper_ ? smapper_->getMapper() : nullptr;
    const std::size_t before_count = mapper ? mapper->GetAllProcessedScans().size() : 0U;
    current_trace_active_ = trace_.is_open();
    current_stamp_ns_ = rclcpp::Time(scan->header.stamp).nanoseconds();
    current_scan_index_ = before_count;
    coarse_checks_ = 0;
    fine_checks_ = 0;
    matcher_rejections_ = 0;
    closure_begins_ = 0;
    closure_ends_ = 0;

    // addScan 是同步进入 Karto 的窄接口；在这里包裹调用可以覆盖每个真正被接受的扫描，
    // 不会像异步 laserCallback 前后计数那样只偶然观察到后台处理结果。
    auto * localized_scan = slam_toolbox::SlamToolbox::addScan(laser, scan, karto_pose);

    mapper = smapper_ ? smapper_->getMapper() : nullptr;
    const std::size_t after_count = mapper ? mapper->GetAllProcessedScans().size() : before_count;
    if (current_trace_active_ && mapper != nullptr && after_count > before_count) {
      writeCandidateTopology(*mapper, after_count);
    }
    current_trace_active_ = false;
    return localized_scan;
  }

  void LoopClosureCheck(const std::string & message) override
  {
    const auto event = parseLoopMatcherEvent(message);
    switch (event.type) {
      case LoopMatcherEventType::kCoarseCheck: ++coarse_checks_; break;
      case LoopMatcherEventType::kFineCheck: ++fine_checks_; break;
      case LoopMatcherEventType::kRejected: ++matcher_rejections_; break;
      case LoopMatcherEventType::kUnknown: break;
    }
    if (!current_trace_active_ || !trace_.is_open()) {
      return;
    }
    std::ostringstream payload;
    payload << "{\"schema_version\":1,\"event\":\"matcher_"
            << toString(event.type) << "\",\"stamp_ns\":" << current_stamp_ns_
            << ",\"scan_index\":" << current_scan_index_ << ",\"response\":";
    writeOptional(payload, event.response);
    payload << ",\"response_threshold\":";
    writeOptional(payload, event.response_threshold);
    payload << ",\"variance_x\":";
    writeOptional(payload, event.variance_x);
    payload << ",\"variance_y\":";
    writeOptional(payload, event.variance_y);
    payload << ",\"variance_threshold\":";
    writeOptional(payload, event.variance_threshold);
    payload << ",\"raw\":\"" << jsonEscape(event.raw) << "\"}";
    writeLine(payload.str());
  }

  void BeginLoopClosure(const std::string & message) override
  {
    ++closure_begins_;
    writeClosureEvent("begin_closure", message);
  }

  void EndLoopClosure(const std::string & message) override
  {
    ++closure_ends_;
    writeClosureEvent("end_closure", message);
  }

private:
  using ConstraintDecision =
    embodied_agent_interfaces::msg::LidarLoopConstraintDecision;
  using ConstraintResult = embodied_agent_interfaces::msg::LidarLoopConstraintResult;

  void publishConstraintResult(
    const ConstraintDecision & decision, const bool committed,
    const std::string & reason, const std::int64_t query_unique_id = -1,
    const std::int64_t candidate_unique_id = -1)
  {
    if (!external_constraint_result_publisher_ ||
      !external_constraint_result_publisher_->is_activated())
    {
      return;
    }
    ConstraintResult result;
    result.header = decision.header;
    result.decision_sequence = decision.decision_sequence;
    result.query_id = decision.query_id;
    result.candidate_id = decision.candidate_id;
    result.resolved_query_unique_id = query_unique_id;
    result.resolved_candidate_unique_id = candidate_unique_id;
    result.committed = committed;
    result.backend = "slam_toolbox_karto";
    result.reason = reason;
    external_constraint_result_publisher_->publish(std::move(result));
  }

  void onExternalLoopConstraint(const ConstraintDecision & decision)
  {
    if (!external_constraint_active_ || !external_constraint_enabled_) {
      return;
    }
    if (!decision.policy_approved || !decision.commit_requested) {
      publishConstraintResult(decision, false, "commit_not_requested");
      return;
    }
    const auto query_stamp_ns = rclcpp::Time(decision.header.stamp).nanoseconds();
    const auto candidate_stamp_ns = rclcpp::Time(decision.candidate_stamp).nanoseconds();
    if (query_stamp_ns <= 0 || candidate_stamp_ns <= 0 ||
      candidate_stamp_ns >= query_stamp_ns ||
      decision.matching_mode != "scan_to_submap" ||
      !std::isfinite(decision.target_to_source.x) ||
      !std::isfinite(decision.target_to_source.y) ||
      !std::isfinite(decision.target_to_source.theta) ||
      !positiveDefiniteCovariance(decision.covariance))
    {
      publishConstraintResult(decision, false, "invalid_commit_payload");
      return;
    }

    boost::mutex::scoped_lock lock(smapper_mutex_);
    karto::Mapper * mapper = smapper_ ? smapper_->getMapper() : nullptr;
    if (mapper == nullptr || mapper->GetGraph() == nullptr || mapper->getScanSolver() == nullptr) {
      publishConstraintResult(decision, false, "slam_backend_not_ready");
      return;
    }
    const auto processed_scans = mapper->GetAllProcessedScans();
    std::vector<LoopConstraintScanRecord> records;
    records.reserve(processed_scans.size());
    for (const auto * scan : processed_scans) {
      if (scan == nullptr) {
        continue;
      }
      records.push_back({
        scan->GetUniqueId(),
        static_cast<std::int64_t>(std::llround(scan->GetTime() * 1.0e9)),
        fromKarto(scan->GetCorrectedPose()),
        fromKarto(scan->GetSensorPose())});
    }
    const auto resolved = resolveLoopConstraint(
      {
        query_stamp_ns,
        candidate_stamp_ns,
        {
          decision.target_to_source.x,
          decision.target_to_source.y,
          decision.target_to_source.theta}},
      records, external_constraint_max_stamp_delta_ns_);
    if (!resolved.available) {
      publishConstraintResult(decision, false, resolved.reason);
      return;
    }

    karto::LocalizedRangeScan * query_scan = nullptr;
    karto::LocalizedRangeScan * candidate_scan = nullptr;
    for (auto * scan : processed_scans) {
      if (scan == nullptr) {
        continue;
      }
      if (scan->GetUniqueId() == resolved.query.unique_id) {
        query_scan = scan;
      }
      if (scan->GetUniqueId() == resolved.candidate.unique_id) {
        candidate_scan = scan;
      }
    }
    if (query_scan == nullptr || candidate_scan == nullptr ||
      query_scan->GetSensorName() != candidate_scan->GetSensorName())
    {
      publishConstraintResult(
        decision, false, "resolved_scan_unavailable_or_sensor_mismatch",
        resolved.query.unique_id, resolved.candidate.unique_id);
      return;
    }

    // Karto SDK 将 kt_bool 定义在全局命名空间；这里同时利用 is_new_edge
    // 防止重复决策在图中生成第二条相同边。
    kt_bool is_new_edge = true;
    auto * edge = mapper->GetGraph()->AddEdge(candidate_scan, query_scan, is_new_edge);
    if (edge == nullptr) {
      publishConstraintResult(
        decision, false, "graph_edge_creation_failed",
        resolved.query.unique_id, resolved.candidate.unique_id);
      return;
    }
    if (!is_new_edge) {
      publishConstraintResult(
        decision, false, "graph_edge_already_exists",
        resolved.query.unique_id, resolved.candidate.unique_id);
      return;
    }

    karto::Matrix3 covariance;
    for (std::size_t row = 0U; row < 3U; ++row) {
      for (std::size_t column = 0U; column < 3U; ++column) {
        covariance(row, column) = decision.covariance[row * 3U + column];
      }
    }
    // 复用 Karto LinkScans 的边标签语义，但只通过公开 AddEdge/ScanSolver 接口写入。
    // 两阶段门控确保只有 typed commit_requested=true 的决策能走到这里。
    edge->SetLabel(new karto::LinkInfo(
        candidate_scan->GetCorrectedPose(),
        query_scan->GetCorrectedAt(toKarto(resolved.query_sensor_pose)), covariance));
    mapper->getScanSolver()->AddConstraint(edge);
    mapper->GetGraph()->CorrectPoses();
    publishConstraintResult(
      decision, true, "committed",
      resolved.query.unique_id, resolved.candidate.unique_id);
  }

  void resetExternalConstraintRuntime()
  {
    external_constraint_subscription_.reset();
    external_constraint_result_publisher_.reset();
    external_constraint_enabled_ = false;
  }

  void detachListener()
  {
    if (attached_mapper_ != nullptr) {
      attached_mapper_->RemoveListener(this);
      attached_mapper_ = nullptr;
    }
    if (trace_.is_open()) {
      trace_.flush();
      trace_.close();
    }
  }

  void writeLine(const std::string & payload)
  {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    trace_ << payload << '\n';
    trace_.flush();
  }

  void writeClosureEvent(const std::string & event, const std::string & message)
  {
    if (!current_trace_active_ || !trace_.is_open()) {
      return;
    }
    std::ostringstream payload;
    payload << "{\"schema_version\":1,\"event\":\"" << event
            << "\",\"stamp_ns\":" << current_stamp_ns_
            << ",\"scan_index\":" << current_scan_index_
            << ",\"raw\":\"" << jsonEscape(message) << "\"}";
    writeLine(payload.str());
  }

  void writeCandidateTopology(karto::Mapper & mapper, const std::size_t processed_count)
  {
    auto * graph = mapper.GetGraph();
    const auto processed_scans = mapper.GetAllProcessedScans();
    if (graph == nullptr || processed_scans.empty()) {
      return;
    }
    karto::LocalizedRangeScan * current = processed_scans.back();
    if (current == nullptr) {
      return;
    }
    const auto & sensor_name = current->GetSensorName();

    const double search_distance = mapper.getParamLoopSearchMaximumDistance();
    const std::size_t minimum_chain = static_cast<std::size_t>(
      std::max(1, mapper.getParamLoopMatchMinimumChainSize()));
    const bool use_barycenter = mapper.getParamUseScanBarycenter();
    const auto current_pose = current->GetReferencePose(use_barycenter);
    const auto near_linked = graph->FindNearLinkedScans(current, search_distance);
    std::unordered_set<int> near_linked_ids;
    for (const auto * scan : near_linked) {
      if (scan != nullptr) {
        near_linked_ids.insert(scan->GetStateId());
      }
    }

    std::vector<LoopCandidateScan> samples;
    for (const auto * scan : processed_scans) {
      if (scan == nullptr || scan->GetSensorName() != sensor_name) {
        continue;
      }
      const auto pose = scan->GetReferencePose(use_barycenter);
      samples.push_back(
        {scan->GetStateId(), pose.GetX(), pose.GetY(),
          near_linked_ids.count(scan->GetStateId()) > 0,
          scan == current});
    }
    const auto topology = summarizeLoopCandidateTopology(
      samples, current_pose.GetX(), current_pose.GetY(), search_distance, minimum_chain);

    std::ostringstream payload;
    payload << "{\"schema_version\":1,\"event\":\"candidate_topology\""
            << ",\"stamp_ns\":" << current_stamp_ns_
            << ",\"scan_index\":" << current_scan_index_
            << ",\"state_id\":" << current->GetStateId()
            << ",\"unique_id\":" << current->GetUniqueId()
            << ",\"sensor\":\"" << jsonEscape(sensor_name.ToString()) << "\""
            << ",\"processed_scan_count\":" << processed_count
            << ",\"matcher_events\":{\"coarse_checks\":" << coarse_checks_
            << ",\"fine_checks\":" << fine_checks_
            << ",\"rejections\":" << matcher_rejections_
            << ",\"closure_begins\":" << closure_begins_
            << ",\"closure_ends\":" << closure_ends_ << "}"
            << ",\"topology\":{\"historical_scan_count\":"
            << topology.historical_scan_count
            << ",\"geometric_near_count\":" << topology.geometric_near_count
            << ",\"near_linked_count\":" << topology.near_linked_count
            << ",\"eligible_unlinked_count\":" << topology.eligible_unlinked_count
            << ",\"qualifying_chain_count\":" << topology.qualifying_chain_count
            << ",\"maximum_eligible_chain_size\":"
            << topology.maximum_eligible_chain_size
            << ",\"linked_terminated_chain_max\":"
            << topology.linked_terminated_chain_max
            << ",\"trailing_chain_size\":" << topology.trailing_chain_size
            << ",\"minimum_chain_size\":" << topology.minimum_chain_size
            << ",\"search_distance_m\":" << topology.search_distance_m
            << ",\"primary_reason\":\"" << jsonEscape(topology.primary_reason) << "\"}"
            // Karto 的候选函数是 private；这里按同一条链规则复算并显式标注来源，
            // 避免把诊断结果误写成对私有函数的二次调用。
            << ",\"replicated_karto_rule\":{\"candidate_chains\":"
            << topology.qualifying_chain_count << ",\"maximum_chain_size\":"
            << topology.maximum_eligible_chain_size << "}}";
    writeLine(payload.str());
  }

  karto::Mapper * attached_mapper_{nullptr};
  std::ofstream trace_;
  std::mutex trace_mutex_;
  bool current_trace_active_{false};
  std::int64_t current_stamp_ns_{0};
  std::size_t current_scan_index_{0};
  std::size_t coarse_checks_{0};
  std::size_t fine_checks_{0};
  std::size_t matcher_rejections_{0};
  std::size_t closure_begins_{0};
  std::size_t closure_ends_{0};
  bool external_constraint_enabled_{false};
  bool external_constraint_active_{false};
  std::int64_t external_constraint_max_stamp_delta_ns_{25000000LL};
  rclcpp::Subscription<ConstraintDecision>::SharedPtr external_constraint_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<ConstraintResult>::SharedPtr
    external_constraint_result_publisher_;
};

}  // namespace embodied_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<embodied_slam::InstrumentedAsynchronousSlamToolbox>(options);
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
