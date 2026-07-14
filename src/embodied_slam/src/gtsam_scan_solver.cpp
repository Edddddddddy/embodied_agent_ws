#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <karto_sdk/Mapper.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "embodied_slam/gtsam_pose_graph.hpp"
#include "embodied_slam/scan_overlap_validator.hpp"

namespace embodied_slam
{

class GtsamScanSolver : public karto::ScanSolver
{
public:
  void Configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node) override
  {
    node_ = node;
    PoseGraphOptimizerConfig config;
    config.max_iterations = static_cast<std::size_t>(
      declare_or_get(*node, "gtsam_max_iterations", 50));
    config.relative_error_tolerance =
      declare_or_get(*node, "gtsam_relative_error_tolerance", 1e-5);
    const double legacy_huber_k = declare_or_get(*node, "gtsam_huber_k", 1.345);
    config.robust_kernel = robust_kernel_from_string(
      declare_or_get(*node, "gtsam_robust_kernel", std::string("huber")));
    config.robust_kernel_k = declare_or_get(*node, "gtsam_robust_kernel_k", legacy_huber_k);
    config.robustify_loop_constraints_only =
      declare_or_get(*node, "gtsam_robustify_loop_constraints_only", false);
    config.minimum_covariance_eigenvalue =
      declare_or_get(*node, "gtsam_minimum_covariance_eigenvalue", 1e-8);
    const int configured_loop_id_separation =
      declare_or_get(*node, "gtsam_loop_constraint_min_id_separation", 20);
    loop_constraint_min_id_separation_ = static_cast<std::size_t>(
      std::max(2, configured_loop_id_separation));
    config.loop_constraint_min_id_separation = loop_constraint_min_id_separation_;
    config.enable_nonlocal_consistency_gate =
      declare_or_get(*node, "gtsam_enable_nonlocal_consistency_gate", false);
    config.max_nonlocal_translation_residual_m =
      declare_or_get(*node, "gtsam_max_nonlocal_translation_residual_m", 2.0);
    config.max_nonlocal_yaw_residual_rad =
      declare_or_get(*node, "gtsam_max_nonlocal_yaw_residual_rad", 0.7853981633974483);
    config.enable_scan_overlap_gate =
      declare_or_get(*node, "gtsam_enable_scan_overlap_gate", false);
    config.minimum_scan_overlap_ratio =
      declare_or_get(*node, "gtsam_minimum_scan_overlap_ratio", 0.65);
    config.scan_overlap_gate_min_translation_residual_m = declare_or_get(
      *node, "gtsam_scan_overlap_gate_min_translation_residual_m", 1.0);
    compute_scan_overlap_ = config.enable_scan_overlap_gate ||
      declare_or_get(*node, "gtsam_compute_scan_overlap_evidence", false);
    scan_overlap_config_.match_distance_m =
      declare_or_get(*node, "gtsam_scan_overlap_match_distance_m", 0.20);
    scan_overlap_config_.point_stride = static_cast<std::size_t>(std::max(
        1, declare_or_get(*node, "gtsam_scan_overlap_point_stride", 2)));
    scan_overlap_config_.minimum_points = static_cast<std::size_t>(std::max(
        1, declare_or_get(*node, "gtsam_scan_overlap_minimum_points", 30)));
    if (compute_scan_overlap_ && (!(scan_overlap_config_.match_distance_m > 0.0) ||
      !std::isfinite(scan_overlap_config_.match_distance_m)))
    {
      throw std::invalid_argument("gtsam_scan_overlap_match_distance_m must be positive");
    }
    if (configured_loop_id_separation < 2) {
      RCLCPP_WARN(
        node->get_logger(), "gtsam_loop_constraint_min_id_separation=%d is unsafe; using 2",
        configured_loop_id_separation);
    }
    const char * log_from_environment = std::getenv("EMBODIED_SLAM_CONSTRAINT_LOG");
    constraint_log_path_ = declare_or_get(*node, "gtsam_constraint_log_path", std::string(""));
    if (constraint_log_path_.empty() && log_from_environment != nullptr) {
      constraint_log_path_ = log_from_environment;
    }
    if (!constraint_log_path_.empty()) {
      if (constraint_log_.is_open()) {
        constraint_log_.close();
      }
      constraint_log_.clear();
      constraint_log_.open(constraint_log_path_, std::ios::out | std::ios::trunc);
      if (!constraint_log_) {
        RCLCPP_WARN(
          node->get_logger(), "Cannot open GTSAM constraint evidence log: %s",
          constraint_log_path_.c_str());
      }
    }
    const char * graph_from_environment = std::getenv("EMBODIED_SLAM_GRAPH_LOG");
    graph_log_path_ = declare_or_get(*node, "gtsam_graph_log_path", std::string(""));
    if (graph_log_path_.empty() && graph_from_environment != nullptr) {
      graph_log_path_ = graph_from_environment;
    }
    best_graph_snapshot_nodes_ = 0U;
    best_graph_snapshot_constraints_ = 0U;
    evidence_poses_.clear();
    evidence_node_stamps_.clear();
    evidence_constraints_.clear();
    optimizer_ = GtsamPoseGraphOptimizer(config);
    RCLCPP_INFO(
      node->get_logger(),
      "Configured GTSAM backend: kernel=%s k=%.3f loop_only=%s "
      "loop_id_separation=%zu gate=%s gate_translation=%.3f gate_yaw=%.3f "
      "scan_overlap_gate=%s compute_overlap=%s min_overlap=%.3f",
      robust_kernel_name(config.robust_kernel), config.robust_kernel_k,
      config.robustify_loop_constraints_only ? "true" : "false",
      config.loop_constraint_min_id_separation,
      config.enable_nonlocal_consistency_gate ? "true" : "false",
      config.max_nonlocal_translation_residual_m, config.max_nonlocal_yaw_residual_rad,
      config.enable_scan_overlap_gate ? "true" : "false",
      compute_scan_overlap_ ? "true" : "false", config.minimum_scan_overlap_ratio);
  }

  void AddNode(karto::Vertex<karto::LocalizedRangeScan> * vertex) override
  {
    if (vertex == nullptr || vertex->GetObject() == nullptr) {
      return;
    }
    std::scoped_lock lock(mutex_);
    const auto * scan = vertex->GetObject();
    initial_poses_[scan->GetUniqueId()] = from_karto(scan->GetCorrectedPose());
    node_stamps_[scan->GetUniqueId()] = scan->GetTime();
    evidence_poses_[scan->GetUniqueId()] = initial_poses_.at(scan->GetUniqueId());
    evidence_node_stamps_[scan->GetUniqueId()] = scan->GetTime();
  }

  void AddConstraint(karto::Edge<karto::LocalizedRangeScan> * edge) override
  {
    if (edge == nullptr || edge->GetSource() == nullptr || edge->GetTarget() == nullptr) {
      return;
    }
    auto * link = dynamic_cast<karto::LinkInfo *>(edge->GetLabel());
    const auto * source = edge->GetSource()->GetObject();
    const auto * target = edge->GetTarget()->GetObject();
    if (link == nullptr || source == nullptr || target == nullptr) {
      return;
    }
    PoseGraphConstraint constraint;
    constraint.source_id = source->GetUniqueId();
    constraint.target_id = target->GetUniqueId();
    constraint.relative_pose = from_karto(link->GetPoseDifference());
    for (std::size_t row = 0; row < 3U; ++row) {
      for (std::size_t column = 0; column < 3U; ++column) {
        constraint.covariance(row, column) = link->GetCovariance()(row, column);
      }
    }
    const auto id_separation = static_cast<std::size_t>(
      std::abs(constraint.source_id - constraint.target_id));
    if (compute_scan_overlap_ && id_separation >= loop_constraint_min_id_separation_) {
      const auto overlap = evaluateScanOverlap(
        // Karto 的缓存点已经随 corrected pose 放到世界系，不能再套相对位姿。
        // 这里从原始量程和激光外参重建 base 局部点，使在线与离线消融坐标语义一致。
        to_local_scan_points(*source),
        to_local_scan_points(*target),
        constraint.relative_pose, scan_overlap_config_);
      if (overlap.available) {
        constraint.scan_overlap_ratio = overlap.overlap_ratio;
      }
    }
    std::scoped_lock lock(mutex_);
    constraints_.push_back(constraint);
    evidence_constraints_[constraint_key(constraint.source_id, constraint.target_id)] = constraint;
    if (constraint_log_) {
      const char * kind =
        id_separation >= loop_constraint_min_id_separation_ ? "loop" : "sequential";
      // ScanSolver 只会收到前端已经接受的边；记录时间戳和相对约束后，离线评估器才能
      // 用独立真值计算 accepted-edge precision / false-loop rate，而不是从最终 ATE 猜测。
      constraint_log_ << std::setprecision(17)
                      << "{\"schema_version\":1,\"backend\":\"gtsam\","
                      << "\"accepted\":true,\"constraint_kind\":\"" << kind << "\","
                      << "\"source_id\":" << constraint.source_id << ","
                      << "\"target_id\":" << constraint.target_id << ","
                      << "\"id_separation\":" << id_separation << ","
                      << "\"source_stamp_s\":" << source->GetTime() << ","
                      << "\"target_stamp_s\":" << target->GetTime() << ","
                      << "\"relative_x_m\":" << constraint.relative_pose.x << ","
                      << "\"relative_y_m\":" << constraint.relative_pose.y << ","
                      << "\"relative_yaw_rad\":" << constraint.relative_pose.yaw << ","
                      << "\"scan_overlap_ratio\":";
      if (constraint.scan_overlap_ratio.has_value()) {
        constraint_log_ << *constraint.scan_overlap_ratio;
      } else {
        constraint_log_ << "null";
      }
      constraint_log_ << "}\n";
      constraint_log_.flush();
    }
  }

  void Compute() override
  {
    std::scoped_lock lock(mutex_);
    try {
      // 每次优化前覆盖写入“本次真正使用”的完整图。消融实验随后复用同一份节点和边，
      // 避免多次 rosbag 回放因异步前端时序不同而把前端差异误判成后端鲁棒核收益。
      write_graph_snapshot();
      const PoseGraphResult result = optimizer_.optimize(initial_poses_, constraints_);
      corrections_.clear();
      corrections_.reserve(result.poses.size());
      graph_.clear();
      for (const auto & [id, pose] : result.poses) {
        corrections_.emplace_back(id, karto::Pose2(pose.x, pose.y, pose.yaw));
        graph_[id] = Eigen::Vector3d(pose.x, pose.y, pose.yaw);
        initial_poses_[id] = pose;
      }
      std::sort(
        corrections_.begin(), corrections_.end(),
        [](const auto & left, const auto & right) {return left.first < right.first;});
      if (auto node = node_.lock()) {
        RCLCPP_DEBUG(
          node->get_logger(),
          "GTSAM optimized %zu nodes/%zu constraints (%zu robust/%zu rejected): "
          "%zu overlap rejected/%zu unavailable, %.6f -> %.6f (%zu iterations)",
          result.poses.size(), constraints_.size(), result.robustified_constraints,
          result.consistency_rejected_constraints,
          result.scan_overlap_rejected_constraints,
          result.scan_overlap_unavailable_constraints,
          result.initial_error, result.final_error, result.iterations);
      }
    } catch (const std::exception & error) {
      if (auto node = node_.lock()) {
        RCLCPP_ERROR(node->get_logger(), "GTSAM optimization failed: %s", error.what());
      }
    }
  }

  const IdPoseVector & GetCorrections() const override {return corrections_;}

  void RemoveNode(kt_int32s id) override
  {
    std::scoped_lock lock(mutex_);
    initial_poses_.erase(id);
    node_stamps_.erase(id);
    evidence_poses_.erase(id);
    evidence_node_stamps_.erase(id);
    constraints_.erase(
      std::remove_if(
        constraints_.begin(), constraints_.end(),
        [id](const auto & item) {return item.source_id == id || item.target_id == id;}),
      constraints_.end());
    for (auto iterator = evidence_constraints_.begin(); iterator != evidence_constraints_.end();) {
      if (iterator->second.source_id == id || iterator->second.target_id == id) {
        iterator = evidence_constraints_.erase(iterator);
      } else {
        ++iterator;
      }
    }
  }

  void RemoveConstraint(kt_int32s source_id, kt_int32s target_id) override
  {
    std::scoped_lock lock(mutex_);
    constraints_.erase(
      std::remove_if(
        constraints_.begin(), constraints_.end(),
        [source_id, target_id](const auto & item) {
          return item.source_id == source_id && item.target_id == target_id;
        }),
      constraints_.end());
    evidence_constraints_.erase(constraint_key(source_id, target_id));
  }

  void Clear() override
  {
    std::scoped_lock lock(mutex_);
    initial_poses_.clear();
    node_stamps_.clear();
    constraints_.clear();
    corrections_.clear();
    graph_.clear();
  }

  void Reset() override {Clear();}

  std::unordered_map<int, Eigen::Vector3d> * getGraph() override {return &graph_;}

  void ModifyNode(const int & unique_id, Eigen::Vector3d pose) override
  {
    std::scoped_lock lock(mutex_);
    initial_poses_[unique_id] = Pose2d{pose.x(), pose.y(), pose.z()};
    evidence_poses_[unique_id] = initial_poses_.at(unique_id);
  }

  void GetNodeOrientation(const int & unique_id, double & yaw) override
  {
    std::scoped_lock lock(mutex_);
    const auto iterator = initial_poses_.find(unique_id);
    if (iterator != initial_poses_.end()) {
      yaw = iterator->second.yaw;
    }
  }

private:
  template<typename T>
  static T declare_or_get(
    rclcpp_lifecycle::LifecycleNode & node, const std::string & name, const T & default_value)
  {
    if (!node.has_parameter(name)) {
      return node.declare_parameter<T>(name, default_value);
    }
    return node.get_parameter(name).get_value<T>();
  }

  static Pose2d from_karto(const karto::Pose2 & pose)
  {
    return {pose.GetX(), pose.GetY(), pose.GetHeading()};
  }

  static std::vector<ScanPoint2d> to_local_scan_points(
    const karto::LocalizedRangeScan & scan)
  {
    const auto * laser = scan.GetLaserRangeFinder();
    const auto * ranges = scan.GetRangeReadings();
    std::vector<ScanPoint2d> output;
    if (laser == nullptr || ranges == nullptr) {
      return output;
    }
    const auto count = scan.GetNumberOfRangeReadings();
    output.reserve(count);
    const auto offset = laser->GetOffsetPose();
    const double cosine = std::cos(offset.GetHeading());
    const double sine = std::sin(offset.GetHeading());
    for (std::size_t index = 0U; index < count; ++index) {
      const double range = ranges[index];
      if (!std::isfinite(range) || range <= laser->GetMinimumRange() ||
        range >= laser->GetRangeThreshold())
      {
        continue;
      }
      const double angle = laser->GetMinimumAngle() +
        static_cast<double>(index) * laser->GetAngularResolution();
      const double laser_x = range * std::cos(angle);
      const double laser_y = range * std::sin(angle);
      output.push_back({
        offset.GetX() + cosine * laser_x - sine * laser_y,
        offset.GetY() + sine * laser_x + cosine * laser_y});
    }
    return output;
  }

  static std::uint64_t constraint_key(int source_id, int target_id)
  {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(source_id)) << 32U) |
           static_cast<std::uint32_t>(target_id);
  }

  void write_graph_snapshot()
  {
    if (graph_log_path_.empty()) {
      return;
    }
    if (evidence_poses_.size() < best_graph_snapshot_nodes_ ||
      (evidence_poses_.size() == best_graph_snapshot_nodes_ &&
      evidence_constraints_.size() <= best_graph_snapshot_constraints_))
    {
      // slam_toolbox 会为局部校正临时重建较小的 ScanSolver 图；只保留迄今最完整的
      // 快照，防止进程结束前的尾段优化覆盖整段数据集位姿图。
      return;
    }
    const std::string temporary_path = graph_log_path_ + ".tmp";
    std::ofstream output(temporary_path, std::ios::out | std::ios::trunc);
    if (!output) {
      if (auto node = node_.lock()) {
        RCLCPP_WARN(
          node->get_logger(), "Cannot write GTSAM graph snapshot: %s", temporary_path.c_str());
      }
      return;
    }
    output << "# embodied_slam_pose_graph_v2 scan_overlap_ratio=optional\n"
           << std::setprecision(17);
    std::vector<int> ids;
    ids.reserve(evidence_poses_.size());
    for (const auto & [id, pose] : evidence_poses_) {
      (void)pose;
      ids.push_back(id);
    }
    std::sort(ids.begin(), ids.end());
    for (const int id : ids) {
      const auto & pose = evidence_poses_.at(id);
      const auto stamp = evidence_node_stamps_.find(id);
      output << "N " << id << ' '
             << (stamp == evidence_node_stamps_.end() ? 0.0 : stamp->second) << ' '
             << pose.x << ' ' << pose.y << ' ' << pose.yaw << '\n';
    }
    std::vector<std::uint64_t> constraint_keys;
    constraint_keys.reserve(evidence_constraints_.size());
    for (const auto & [key, constraint] : evidence_constraints_) {
      (void)constraint;
      constraint_keys.push_back(key);
    }
    std::sort(constraint_keys.begin(), constraint_keys.end());
    for (const auto key : constraint_keys) {
      const auto & constraint = evidence_constraints_.at(key);
      output << "C " << constraint.source_id << ' ' << constraint.target_id << ' '
             << constraint.relative_pose.x << ' ' << constraint.relative_pose.y << ' '
             << constraint.relative_pose.yaw;
      for (std::size_t row = 0; row < 3U; ++row) {
        for (std::size_t column = 0; column < 3U; ++column) {
          output << ' ' << constraint.covariance(row, column);
        }
      }
      if (constraint.scan_overlap_ratio.has_value()) {
        output << ' ' << *constraint.scan_overlap_ratio;
      }
      output << '\n';
    }
    output.close();
    if (!output || std::rename(temporary_path.c_str(), graph_log_path_.c_str()) != 0) {
      if (auto node = node_.lock()) {
        RCLCPP_WARN(
          node->get_logger(), "Cannot publish GTSAM graph snapshot: %s", graph_log_path_.c_str());
      }
    } else {
      best_graph_snapshot_nodes_ = evidence_poses_.size();
      best_graph_snapshot_constraints_ = evidence_constraints_.size();
    }
  }

  mutable std::mutex mutex_;
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  GtsamPoseGraphOptimizer optimizer_;
  std::unordered_map<int, Pose2d> initial_poses_;
  std::unordered_map<int, double> node_stamps_;
  std::vector<PoseGraphConstraint> constraints_;
  // ScanSolver 的工作集会被 slam_toolbox 分批 Clear；证据图独立累计并按 ID/边去重，
  // 因而既不改变在线优化语义，又能导出覆盖整段 bag 的固定后端输入。
  std::unordered_map<int, Pose2d> evidence_poses_;
  std::unordered_map<int, double> evidence_node_stamps_;
  std::unordered_map<std::uint64_t, PoseGraphConstraint> evidence_constraints_;
  std::size_t loop_constraint_min_id_separation_{20U};
  bool compute_scan_overlap_{false};
  ScanOverlapConfig scan_overlap_config_;
  std::string constraint_log_path_;
  std::ofstream constraint_log_;
  std::string graph_log_path_;
  std::size_t best_graph_snapshot_nodes_{0U};
  std::size_t best_graph_snapshot_constraints_{0U};
  IdPoseVector corrections_;
  std::unordered_map<int, Eigen::Vector3d> graph_;
};

}  // namespace embodied_slam

PLUGINLIB_EXPORT_CLASS(embodied_slam::GtsamScanSolver, karto::ScanSolver)
