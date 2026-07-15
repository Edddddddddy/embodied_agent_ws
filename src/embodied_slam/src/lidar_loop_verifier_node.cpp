#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <embodied_agent_interfaces/msg/lidar_loop_candidate_array.hpp>
#include <embodied_agent_interfaces/msg/lidar_loop_verification_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "embodied_agent_middleware/qos_profiles.hpp"
#include "embodied_slam/lidar_loop_verifier.hpp"
#include "embodied_slam/lidar_loop_verifier_factory.hpp"

namespace embodied_slam
{
namespace
{

std::int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

std::vector<LidarPoint2D> laserPoints(const sensor_msgs::msg::LaserScan & scan)
{
  std::vector<LidarPoint2D> points;
  points.reserve(scan.ranges.size());
  for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
    const double range = static_cast<double>(scan.ranges[index]);
    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      continue;
    }
    const double angle = static_cast<double>(scan.angle_min) +
      static_cast<double>(index) * static_cast<double>(scan.angle_increment);
    points.push_back({range * std::cos(angle), range * std::sin(angle)});
  }
  return points;
}

}  // namespace

class LidarLoopVerifierNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using CandidateArray = embodied_agent_interfaces::msg::LidarLoopCandidateArray;
  using VerificationArray = embodied_agent_interfaces::msg::LidarLoopVerificationArray;
  using LaserScan = sensor_msgs::msg::LaserScan;

  explicit LidarLoopVerifierNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("lidar_loop_verifier", "", options)
  {
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("candidate_topic", "/slam/loop_candidates");
    declare_parameter<std::string>("output_topic", "/slam/loop_verifications");
    declare_parameter<int>("maximum_cached_scans", 12000);
    declare_parameter<int>("maximum_pending_batches", 16);
    declare_parameter<int>("pending_scan_grace", 2);
    declare_parameter<int>("point_stride", 2);
    declare_parameter<int>("minimum_points", 30);
    declare_parameter<double>("minimum_inlier_ratio", 0.35);
    declare_parameter<double>("maximum_rmse_m", 0.18);
    declare_parameter<double>("minimum_observability_ratio", 0.005);
    declare_parameter<double>("maximum_translation_m", 2.0);
    RCLCPP_INFO(get_logger(), "LiDAR loop geometry verifier created (shadow-only)");
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    try {
      LiveLidarLoopVerifierConfig config;
      config.maximum_cached_scans = positiveSize("maximum_cached_scans");
      maximum_pending_batches_ = positiveSize("maximum_pending_batches");
      pending_scan_grace_ = positiveSize("pending_scan_grace");
      config.matcher.point_stride = positiveSize("point_stride");
      config.matcher.minimum_points = positiveSize("minimum_points");
      config.matcher.minimum_inlier_ratio =
        get_parameter("minimum_inlier_ratio").as_double();
      config.matcher.maximum_rmse_m = get_parameter("maximum_rmse_m").as_double();
      config.matcher.minimum_observability_ratio =
        get_parameter("minimum_observability_ratio").as_double();
      config.matcher.maximum_translation_m =
        get_parameter("maximum_translation_m").as_double();
      verifier_ = std::make_unique<LiveLidarLoopVerifier>(config);

      callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      publisher_ = create_publisher<VerificationArray>(
        get_parameter("output_topic").as_string(), embodied_agent_middleware::event_qos());
      rclcpp::SubscriptionOptions options;
      options.callback_group = callback_group_;
      scan_subscription_ = create_subscription<LaserScan>(
        get_parameter("scan_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&LidarLoopVerifierNode::onScan, this, std::placeholders::_1), options);
      candidate_subscription_ = create_subscription<CandidateArray>(
        get_parameter("candidate_topic").as_string(), embodied_agent_middleware::event_qos(),
        std::bind(&LidarLoopVerifierNode::onCandidates, this, std::placeholders::_1), options);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "configure failed: %s", error.what());
      resetRuntime();
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "configured; verified matches remain outside the pose graph");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    publisher_->on_activate();
    active_ = true;
    RCLCPP_INFO(get_logger(), "activated; caching LaserScan and verifying candidate batches");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    active_ = false;
    pending_batches_.clear();
    publisher_->on_deactivate();
    RCLCPP_INFO(get_logger(), "deactivated; pending candidate batches cleared");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    active_ = false;
    resetRuntime();
    RCLCPP_INFO(get_logger(), "cleaned up; scan cache and pending batches cleared");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    active_ = false;
    resetRuntime();
    return CallbackReturn::SUCCESS;
  }

private:
  struct PendingBatch
  {
    CandidateArray message;
    std::size_t enqueue_scan_sequence{0U};
  };

  std::size_t positiveSize(const std::string & name) const
  {
    const auto value = get_parameter(name).as_int();
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return static_cast<std::size_t>(value);
  }

  void onScan(const LaserScan::SharedPtr scan)
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    if (!active_ || !verifier_) {
      return;
    }
    ++scan_sequence_;
    try {
      verifier_->cacheScan(stampToNanoseconds(scan->header.stamp), laserPoints(*scan));
      drainPendingBatches();
    } catch (const std::exception & error) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "scan cache rejected input: %s", error.what());
    }
  }

  void onCandidates(const CandidateArray::SharedPtr batch)
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    if (!active_ || !verifier_ || !publisher_ || !publisher_->is_activated()) {
      return;
    }
    const auto query_stamp_ns = stampToNanoseconds(batch->header.stamp);
    if (verifier_->hasScan(query_stamp_ns)) {
      publishVerification(*batch);
      return;
    }

    // 两个订阅来自不同发布者，DDS 不保证跨 topic 回调顺序。候选先到时短暂等待
    // 同时间戳 LaserScan，防止把正常调度抖动误报成“查询帧已丢失”。
    if (pending_batches_.size() >= maximum_pending_batches_) {
      publishVerification(pending_batches_.front().message);
      pending_batches_.pop_front();
    }
    pending_batches_.push_back(PendingBatch{*batch, scan_sequence_});
  }

  void drainPendingBatches()
  {
    auto current = pending_batches_.begin();
    while (current != pending_batches_.end()) {
      const auto query_stamp_ns = stampToNanoseconds(current->message.header.stamp);
      const bool ready = verifier_->hasScan(query_stamp_ns);
      const bool expired = scan_sequence_ >=
        current->enqueue_scan_sequence + pending_scan_grace_;
      if (ready || expired) {
        publishVerification(current->message);
        current = pending_batches_.erase(current);
      } else {
        ++current;
      }
    }
  }

  void publishVerification(const CandidateArray & batch)
  {
    std::vector<LiveLidarLoopCandidateInput> candidates;
    candidates.reserve(batch.candidates.size());
    for (const auto & input : batch.candidates) {
      candidates.push_back({
        input.candidate_id, stampToNanoseconds(input.candidate_stamp), input.rank,
        input.similarity, input.yaw_offset_rad});
    }
    const auto results = verifier_->verify(
      stampToNanoseconds(batch.header.stamp), candidates);

    VerificationArray output;
    output.header = batch.header;
    output.query_id = batch.query_id;
    output.indexed_scans = batch.indexed_scans;
    // 即使 ICP 通过，仍需后端一致性/鲁棒核评估；此节点不持有写图接口。
    output.shadow_only = true;
    output.matching_mode = "scan_to_scan";
    output.verifications.reserve(results.size());
    for (const auto & result : results) {
      auto & item = output.verifications.emplace_back();
      item.candidate_id = result.candidate.candidate_id;
      const auto stamp_ns = result.candidate.candidate_stamp_ns;
      item.candidate_stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
      item.candidate_stamp.nanosec =
        static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
      item.rank = result.candidate.rank;
      item.descriptor_similarity = result.candidate.similarity;
      item.available = result.match.available;
      item.converged = result.match.converged;
      item.accepted = result.match.accepted;
      item.yaw_ambiguous = result.match.yaw_ambiguous;
      item.target_to_source.x = result.match.target_to_source.x;
      item.target_to_source.y = result.match.target_to_source.y;
      item.target_to_source.theta = result.match.target_to_source.yaw;
      item.source_points = static_cast<std::uint32_t>(result.match.source_points);
      item.target_points = static_cast<std::uint32_t>(result.match.target_points);
      item.correspondences = static_cast<std::uint32_t>(result.match.correspondences);
      item.inlier_ratio = result.match.inlier_ratio;
      item.bidirectional_overlap_ratio = result.match.bidirectional_overlap_ratio;
      item.rmse_m = result.match.rmse_m;
      item.observability_ratio = result.match.observability_ratio;
      item.rejection_reason = result.match.rejection_reason;
    }
    publisher_->publish(std::move(output));
  }

  void resetRuntime()
  {
    scan_subscription_.reset();
    candidate_subscription_.reset();
    publisher_.reset();
    verifier_.reset();
    callback_group_.reset();
    pending_batches_.clear();
    scan_sequence_ = 0U;
  }

  std::mutex runtime_mutex_;
  bool active_{false};
  std::size_t maximum_pending_batches_{16U};
  std::size_t pending_scan_grace_{2U};
  std::size_t scan_sequence_{0U};
  std::unique_ptr<LiveLidarLoopVerifier> verifier_;
  std::deque<PendingBatch> pending_batches_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<LaserScan>::SharedPtr scan_subscription_;
  rclcpp::Subscription<CandidateArray>::SharedPtr candidate_subscription_;
  rclcpp_lifecycle::LifecyclePublisher<VerificationArray>::SharedPtr publisher_;
};

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_lidar_loop_verifier_node(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<LidarLoopVerifierNode>(options);
}

}  // namespace embodied_slam

RCLCPP_COMPONENTS_REGISTER_NODE(embodied_slam::LidarLoopVerifierNode)
