#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <embodied_agent_interfaces/msg/lidar_loop_constraint_decision.hpp>
#include <embodied_agent_interfaces/msg/lidar_loop_verification_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "embodied_agent_middleware/qos_profiles.hpp"
#include "embodied_slam/lidar_loop_constraint_gate.hpp"
#include "embodied_slam/lidar_loop_constraint_gate_factory.hpp"

namespace embodied_slam
{
namespace
{

std::int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

}  // namespace

class LidarLoopConstraintGateNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using VerificationArray = embodied_agent_interfaces::msg::LidarLoopVerificationArray;
  using Decision = embodied_agent_interfaces::msg::LidarLoopConstraintDecision;

  explicit LidarLoopConstraintGateNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("lidar_loop_constraint_gate", "", options)
  {
    declare_parameter<std::string>("input_topic", "/slam/loop_verifications");
    declare_parameter<std::string>("output_topic", "/slam/loop_constraint_decisions");
    declare_parameter<bool>("commit_enabled", false);
    declare_parameter<bool>("require_scan_to_submap", true);
    declare_parameter<int>("minimum_submap_scans", 2);
    declare_parameter<int>("minimum_correspondences", 30);
    declare_parameter<double>("minimum_descriptor_similarity", 0.60);
    declare_parameter<double>("minimum_inlier_ratio", 0.35);
    declare_parameter<double>("minimum_overlap_ratio", 0.55);
    declare_parameter<double>("maximum_rmse_m", 0.15);
    declare_parameter<double>("minimum_observability_ratio", 0.005);
    declare_parameter<int>("minimum_commit_query_separation", 5);
    declare_parameter<int>("maximum_history", 4096);
    declare_parameter<double>("translation_variance", 0.04);
    declare_parameter<double>("yaw_variance", 0.04);
    declare_parameter<int>("minimum_temporal_confirmations", 4);
    declare_parameter<double>("maximum_temporal_query_gap_s", 2.0);
    declare_parameter<double>("maximum_temporal_pair_age_delta_s", 1.25);
    declare_parameter<double>("maximum_temporal_translation_delta_m", 0.55);
    declare_parameter<double>("maximum_temporal_yaw_delta_rad", 0.35);
    RCLCPP_INFO(get_logger(), "LiDAR loop constraint gate created (commit disabled by default)");
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      LidarLoopConstraintGateConfig config;
      config.commit_enabled = get_parameter("commit_enabled").as_bool();
      config.require_scan_to_submap = get_parameter("require_scan_to_submap").as_bool();
      config.minimum_submap_scans = positiveSize("minimum_submap_scans");
      config.minimum_correspondences = positiveSize("minimum_correspondences");
      config.minimum_descriptor_similarity =
        get_parameter("minimum_descriptor_similarity").as_double();
      config.minimum_inlier_ratio = get_parameter("minimum_inlier_ratio").as_double();
      config.minimum_overlap_ratio = get_parameter("minimum_overlap_ratio").as_double();
      config.maximum_rmse_m = get_parameter("maximum_rmse_m").as_double();
      config.minimum_observability_ratio =
        get_parameter("minimum_observability_ratio").as_double();
      const auto separation = get_parameter("minimum_commit_query_separation").as_int();
      if (separation < 0) {
        throw std::invalid_argument("minimum_commit_query_separation must be non-negative");
      }
      config.minimum_commit_query_separation = separation;
      config.maximum_history = positiveSize("maximum_history");
      config.translation_variance = get_parameter("translation_variance").as_double();
      config.yaw_variance = get_parameter("yaw_variance").as_double();
      config.temporal.minimum_confirmations = positiveSize("minimum_temporal_confirmations");
      config.temporal.maximum_query_gap_s =
        get_parameter("maximum_temporal_query_gap_s").as_double();
      config.temporal.maximum_pair_age_delta_s =
        get_parameter("maximum_temporal_pair_age_delta_s").as_double();
      config.temporal.maximum_translation_delta_m =
        get_parameter("maximum_temporal_translation_delta_m").as_double();
      config.temporal.maximum_yaw_delta_rad =
        get_parameter("maximum_temporal_yaw_delta_rad").as_double();
      gate_ = std::make_unique<LidarLoopConstraintGate>(config);
      commit_enabled_ = config.commit_enabled;

      publisher_ = create_publisher<Decision>(
        get_parameter("output_topic").as_string(), embodied_agent_middleware::event_qos());
      subscription_ = create_subscription<VerificationArray>(
        get_parameter("input_topic").as_string(), embodied_agent_middleware::event_qos(),
        [this](const VerificationArray::SharedPtr message) {onVerification(*message);});
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "configure failed: %s", error.what());
      resetRuntime();
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(
      get_logger(), "configured; commit_enabled=%s",
      commit_enabled_ ? "true" : "false");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    publisher_->on_activate();
    active_ = true;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    publisher_->on_deactivate();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    resetRuntime();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_ = false;
    resetRuntime();
    return CallbackReturn::SUCCESS;
  }

private:
  std::size_t positiveSize(const std::string & name) const
  {
    const auto value = get_parameter(name).as_int();
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return static_cast<std::size_t>(value);
  }

  void onVerification(const VerificationArray & batch)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_ || !gate_ || !publisher_ || !publisher_->is_activated()) {
      return;
    }
    std::vector<LidarLoopConstraintInput> inputs;
    inputs.reserve(batch.verifications.size());
    for (const auto & item : batch.verifications) {
      inputs.push_back({
        batch.query_id,
        stampToNanoseconds(batch.header.stamp),
        item.candidate_id,
        stampToNanoseconds(item.candidate_stamp),
        item.rank,
        batch.shadow_only,
        batch.matching_mode,
        item.available,
        item.converged,
        item.accepted,
        item.yaw_ambiguous,
        {item.target_to_source.x, item.target_to_source.y, item.target_to_source.theta},
        item.query_submap_scans,
        item.candidate_submap_scans,
        item.correspondences,
        item.descriptor_similarity,
        item.inlier_ratio,
        item.bidirectional_overlap_ratio,
        item.rmse_m,
        item.observability_ratio});
    }
    const auto decisions = gate_->evaluate(inputs);
    for (const auto & decision : decisions) {
      Decision output;
      output.header = batch.header;
      output.decision_sequence = decision.sequence;
      output.query_id = decision.input.query_id;
      output.candidate_id = decision.input.candidate_id;
      output.candidate_stamp.sec = static_cast<std::int32_t>(
        decision.input.candidate_stamp_ns / 1000000000LL);
      output.candidate_stamp.nanosec = static_cast<std::uint32_t>(
        decision.input.candidate_stamp_ns % 1000000000LL);
      output.rank = decision.input.rank;
      output.policy_approved = decision.policy_approved;
      output.commit_requested = decision.commit_requested;
      output.upstream_shadow_only = decision.input.upstream_shadow_only;
      output.matching_mode = decision.input.matching_mode;
      output.target_to_source.x = decision.input.target_to_source.x;
      output.target_to_source.y = decision.input.target_to_source.y;
      output.target_to_source.theta = decision.input.target_to_source.yaw;
      output.covariance = decision.covariance;
      output.quality_score = decision.quality_score;
      output.temporal_confirmation_count = decision.temporal.confirmation_count;
      output.temporal_confirmation_required = decision.temporal.confirmation_required;
      output.temporal_query_gap_s = decision.temporal.query_gap_s;
      output.temporal_pair_age_delta_s = decision.temporal.pair_age_delta_s;
      output.temporal_translation_delta_m = decision.temporal.translation_delta_m;
      output.temporal_yaw_delta_rad = decision.temporal.yaw_delta_rad;
      output.descriptor_similarity = decision.input.descriptor_similarity;
      output.inlier_ratio = decision.input.inlier_ratio;
      output.bidirectional_overlap_ratio = decision.input.overlap_ratio;
      output.rmse_m = decision.input.rmse_m;
      output.observability_ratio = decision.input.observability_ratio;
      output.reason = decision.reason;
      publisher_->publish(std::move(output));
    }
  }

  void resetRuntime()
  {
    subscription_.reset();
    publisher_.reset();
    gate_.reset();
    commit_enabled_ = false;
  }

  std::mutex mutex_;
  bool active_{false};
  bool commit_enabled_{false};
  std::unique_ptr<LidarLoopConstraintGate> gate_;
  rclcpp_lifecycle::LifecyclePublisher<Decision>::SharedPtr publisher_;
  rclcpp::Subscription<VerificationArray>::SharedPtr subscription_;
};

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> makeLidarLoopConstraintGateNode(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<LidarLoopConstraintGateNode>(options);
}

}  // namespace embodied_slam

RCLCPP_COMPONENTS_REGISTER_NODE(embodied_slam::LidarLoopConstraintGateNode)
