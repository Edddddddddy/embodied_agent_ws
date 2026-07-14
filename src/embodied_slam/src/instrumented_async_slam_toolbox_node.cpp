#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

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
    const std::string path = get_parameter("loop_frontend_log_path").as_string();
    if (path.empty()) {
      RCLCPP_INFO(get_logger(), "Loop frontend trace disabled (empty path)");
      return result;
    }
    trace_.open(path, std::ios::out | std::ios::trunc);
    if (!trace_.is_open()) {
      RCLCPP_ERROR(get_logger(), "Cannot open loop frontend trace: %s", path.c_str());
      return CallbackReturn::FAILURE;
    }
    attached_mapper_ = smapper_->getMapper();
    attached_mapper_->AddListener(this);
    RCLCPP_INFO(get_logger(), "Loop frontend trace -> %s", path.c_str());
    return result;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override
  {
    detachListener();
    return slam_toolbox::AsynchronousSlamToolbox::on_cleanup(state);
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override
  {
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
