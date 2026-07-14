#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <embodied_agent_interfaces/msg/lidar_loop_candidate.hpp>
#include <embodied_agent_interfaces/msg/lidar_loop_candidate_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "embodied_agent_middleware/qos_profiles.hpp"
#include "embodied_slam/lidar_loop_candidate_factory.hpp"
#include "embodied_slam/lidar_loop_runtime.hpp"

namespace embodied_slam
{

class LidarLoopCandidateNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using CandidateArray = embodied_agent_interfaces::msg::LidarLoopCandidateArray;
  using LaserScan = sensor_msgs::msg::LaserScan;

  explicit LidarLoopCandidateNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("lidar_loop_candidate", "", options)
  {
    declare_parameter<std::string>("input_topic", "/scan");
    declare_parameter<std::string>("output_topic", "/slam/loop_candidates");
    declare_parameter<int>("radial_bins", 20);
    declare_parameter<int>("angular_bins", 60);
    declare_parameter<double>("maximum_range_m", 20.0);
    declare_parameter<int>("minimum_points", 30);
    declare_parameter<double>("minimum_temporal_separation_s", 60.0);
    declare_parameter<int>("coarse_preselection_count", 30);
    declare_parameter<double>("minimum_similarity", 0.0);
    declare_parameter<int>("top_k", 10);
    declare_parameter<double>("sample_interval_s", 0.5);
    RCLCPP_INFO(get_logger(), "LiDAR loop candidate lifecycle component created (shadow-only)");
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    try {
      LiveLidarLoopDetectorConfig config;
      config.descriptor.radial_bins = positive_size("radial_bins");
      config.descriptor.angular_bins = positive_size("angular_bins");
      config.descriptor.maximum_range_m = get_parameter("maximum_range_m").as_double();
      config.descriptor.minimum_points = positive_size("minimum_points");
      config.index.minimum_temporal_separation_s =
        get_parameter("minimum_temporal_separation_s").as_double();
      config.index.coarse_preselection_count = positive_size("coarse_preselection_count");
      config.index.minimum_similarity = get_parameter("minimum_similarity").as_double();
      config.top_k = positive_size("top_k");
      config.sample_interval_s = get_parameter("sample_interval_s").as_double();
      detector_ = std::make_unique<LiveLidarLoopDetector>(config);

      callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      publisher_ = create_publisher<CandidateArray>(
        get_parameter("output_topic").as_string(), embodied_agent_middleware::event_qos());
      rclcpp::SubscriptionOptions options;
      options.callback_group = callback_group_;
      subscription_ = create_subscription<LaserScan>(
        get_parameter("input_topic").as_string(), rclcpp::SensorDataQoS(),
        std::bind(&LidarLoopCandidateNode::on_scan, this, std::placeholders::_1), options);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "configure failed: %s", error.what());
      reset_runtime();
      return CallbackReturn::FAILURE;
    }
    RCLCPP_INFO(get_logger(), "configured; candidates cannot write pose-graph constraints");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    publisher_->on_activate();
    active_ = true;
    RCLCPP_INFO(get_logger(), "activated; consuming LaserScan");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    active_ = false;
    publisher_->on_deactivate();
    RCLCPP_INFO(get_logger(), "deactivated; scan callbacks are ignored");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    active_ = false;
    reset_runtime();
    RCLCPP_INFO(get_logger(), "cleaned up; in-memory descriptor index cleared");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    active_ = false;
    reset_runtime();
    return CallbackReturn::SUCCESS;
  }

private:
  std::size_t positive_size(const std::string & name) const
  {
    const auto value = get_parameter(name).as_int();
    if (value <= 0) {
      throw std::invalid_argument(name + " must be positive");
    }
    return static_cast<std::size_t>(value);
  }

  void on_scan(const LaserScan::SharedPtr scan)
  {
    std::lock_guard<std::mutex> lock(runtime_mutex_);
    if (!active_ || !detector_ || !publisher_ || !publisher_->is_activated()) {
      return;
    }

    std::vector<LidarPoint2D> points;
    points.reserve(scan->ranges.size());
    for (std::size_t index = 0; index < scan->ranges.size(); ++index) {
      const double range = static_cast<double>(scan->ranges[index]);
      if (!std::isfinite(range) || range < scan->range_min || range > scan->range_max) {
        continue;
      }
      const double angle = static_cast<double>(scan->angle_min) +
        static_cast<double>(index) * static_cast<double>(scan->angle_increment);
      points.push_back({range * std::cos(angle), range * std::sin(angle)});
    }

    try {
      const double stamp_s = rclcpp::Time(scan->header.stamp).seconds();
      const auto batch = detector_->ingest(stamp_s, points);
      if (!batch) {
        return;
      }
      CandidateArray output;
      output.header = scan->header;
      output.query_id = batch->query_id;
      output.indexed_scans = static_cast<std::uint32_t>(batch->indexed_scans);
      // 该标志是接口契约而不只是日志：下游必须先做 scan matching/overlap 验证，
      // 当前组件绝不把外观相似度直接升级为后端图约束。
      output.shadow_only = true;
      output.candidates.reserve(batch->candidates.size());
      for (std::size_t rank = 0; rank < batch->candidates.size(); ++rank) {
        const auto & source = batch->candidates[rank];
        auto & candidate = output.candidates.emplace_back();
        candidate.candidate_id = source.scan_id;
        const auto stamp_ns = static_cast<std::int64_t>(source.stamp_s * 1e9);
        candidate.candidate_stamp.sec = static_cast<std::int32_t>(stamp_ns / 1000000000LL);
        candidate.candidate_stamp.nanosec =
          static_cast<std::uint32_t>(stamp_ns % 1000000000LL);
        candidate.rank = static_cast<std::uint32_t>(rank + 1);
        candidate.similarity = source.similarity;
        candidate.sector_shift = static_cast<std::uint32_t>(source.sector_shift);
        candidate.yaw_offset_rad = source.yaw_offset_rad;
        candidate.ring_key_distance = source.ring_key_distance;
      }
      publisher_->publish(std::move(output));
    } catch (const std::exception & error) {
      // 时间回退常见于 bag 重播/仿真 reset。拒绝这一帧比静默重置索引更安全，
      // 因为后者会让证据序列在无提示时失去可追溯性。
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "scan rejected: %s", error.what());
    }
  }

  void reset_runtime()
  {
    subscription_.reset();
    publisher_.reset();
    detector_.reset();
    callback_group_.reset();
  }

  std::mutex runtime_mutex_;
  bool active_{false};
  std::unique_ptr<LiveLidarLoopDetector> detector_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Subscription<LaserScan>::SharedPtr subscription_;
  rclcpp_lifecycle::LifecyclePublisher<CandidateArray>::SharedPtr publisher_;
};

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> make_lidar_loop_candidate_node(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<LidarLoopCandidateNode>(options);
}

}  // namespace embodied_slam

RCLCPP_COMPONENTS_REGISTER_NODE(embodied_slam::LidarLoopCandidateNode)
