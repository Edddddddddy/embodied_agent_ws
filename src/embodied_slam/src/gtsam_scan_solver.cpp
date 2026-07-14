#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <karto_sdk/Mapper.h>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "embodied_slam/gtsam_pose_graph.hpp"

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
    config.huber_k = declare_or_get(*node, "gtsam_huber_k", 1.345);
    config.minimum_covariance_eigenvalue =
      declare_or_get(*node, "gtsam_minimum_covariance_eigenvalue", 1e-8);
    const int configured_loop_id_separation =
      declare_or_get(*node, "gtsam_loop_constraint_min_id_separation", 20);
    loop_constraint_min_id_separation_ = static_cast<std::size_t>(
      std::max(2, configured_loop_id_separation));
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
    optimizer_ = GtsamPoseGraphOptimizer(config);
    RCLCPP_INFO(node->get_logger(), "Configured embodied_slam GTSAM pose-graph backend");
  }

  void AddNode(karto::Vertex<karto::LocalizedRangeScan> * vertex) override
  {
    if (vertex == nullptr || vertex->GetObject() == nullptr) {
      return;
    }
    std::scoped_lock lock(mutex_);
    const auto * scan = vertex->GetObject();
    initial_poses_[scan->GetUniqueId()] = from_karto(scan->GetCorrectedPose());
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
    std::scoped_lock lock(mutex_);
    constraints_.push_back(constraint);
    if (constraint_log_) {
      const auto id_separation = static_cast<std::size_t>(
        std::abs(constraint.source_id - constraint.target_id));
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
                      << "\"relative_yaw_rad\":" << constraint.relative_pose.yaw << "}\n";
      constraint_log_.flush();
    }
  }

  void Compute() override
  {
    std::scoped_lock lock(mutex_);
    try {
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
          "GTSAM optimized %zu nodes/%zu constraints: %.6f -> %.6f (%zu iterations)",
          result.poses.size(), constraints_.size(), result.initial_error, result.final_error,
          result.iterations);
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
    constraints_.erase(
      std::remove_if(
        constraints_.begin(), constraints_.end(),
        [id](const auto & item) {return item.source_id == id || item.target_id == id;}),
      constraints_.end());
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
  }

  void Clear() override
  {
    std::scoped_lock lock(mutex_);
    initial_poses_.clear();
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

  mutable std::mutex mutex_;
  std::weak_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  GtsamPoseGraphOptimizer optimizer_;
  std::unordered_map<int, Pose2d> initial_poses_;
  std::vector<PoseGraphConstraint> constraints_;
  std::size_t loop_constraint_min_id_separation_{20U};
  std::string constraint_log_path_;
  std::ofstream constraint_log_;
  IdPoseVector corrections_;
  std::unordered_map<int, Eigen::Vector3d> graph_;
};

}  // namespace embodied_slam

PLUGINLIB_EXPORT_CLASS(embodied_slam::GtsamScanSolver, karto::ScanSolver)
