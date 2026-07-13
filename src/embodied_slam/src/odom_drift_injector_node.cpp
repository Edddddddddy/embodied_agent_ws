#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "embodied_agent_middleware/qos_profiles.hpp"
#include "embodied_slam/drift_model.hpp"
#include "embodied_slam/trajectory_metrics.hpp"

namespace embodied_slam
{

namespace
{
Pose2d pose_from_odometry(const nav_msgs::msg::Odometry & message)
{
  const auto & orientation = message.pose.pose.orientation;
  const double sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
  const double cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
  return Pose2d{
    message.pose.pose.position.x,
    message.pose.pose.position.y,
    std::atan2(sin_yaw, cos_yaw)};
}

void set_yaw(geometry_msgs::msg::Quaternion & quaternion, double yaw)
{
  quaternion.x = 0.0;
  quaternion.y = 0.0;
  quaternion.z = std::sin(yaw * 0.5);
  quaternion.w = std::cos(yaw * 0.5);
}

diagnostic_msgs::msg::KeyValue metric_value(const std::string & key, double value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = std::to_string(value);
  return item;
}
}  // namespace

class OdomDriftInjectorNode : public rclcpp::Node
{
public:
  OdomDriftInjectorNode()
  : Node("odom_drift_injector"), tf_broadcaster_(*this), static_tf_broadcaster_(*this)
  {
    DriftConfig config;
    config.linear_scale = declare_parameter("linear_scale", 1.04);
    config.lateral_scale = declare_parameter("lateral_scale", 1.0);
    config.yaw_bias_per_meter = declare_parameter("yaw_bias_per_meter", 0.035);
    config.translation_noise_stddev = declare_parameter("translation_noise_stddev", 0.002);
    config.yaw_noise_stddev = declare_parameter("yaw_noise_stddev", 0.001);
    config.random_seed = static_cast<std::uint32_t>(declare_parameter("random_seed", 42));
    drift_model_ = DriftModel(config);

    reference_odom_topic_ = declare_parameter("reference_odom_topic", "/odom");
    input_scan_topic_ = declare_parameter("input_scan_topic", "/scan");
    output_odom_topic_ = declare_parameter("output_odom_topic", "/slam/odom");
    output_scan_topic_ = declare_parameter("output_scan_topic", "/slam/scan");
    odom_frame_ = declare_parameter("odom_frame", "slam_odom");
    base_frame_ = declare_parameter("base_frame", "slam_base_link");
    laser_frame_ = declare_parameter("laser_frame", "slam_laser");
    laser_z_ = declare_parameter("laser_z", 0.18);
    max_path_samples_ = static_cast<std::size_t>(declare_parameter("max_path_samples", 6000));

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      output_odom_topic_, embodied_agent_middleware::sensor_qos(20));
    scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      output_scan_topic_, embodied_agent_middleware::sensor_qos(10));
    reference_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/slam/reference_path", embodied_agent_middleware::state_qos());
    raw_path_pub_ = create_publisher<nav_msgs::msg::Path>(
      "/slam/raw_path", embodied_agent_middleware::state_qos());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", embodied_agent_middleware::diagnostics_qos());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      reference_odom_topic_, embodied_agent_middleware::sensor_qos(20),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {on_odometry(*message);});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      input_scan_topic_, embodied_agent_middleware::sensor_qos(10),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {on_scan(*message);});

    publish_laser_transform();
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1), [this]() {publish_diagnostics();});
    RCLCPP_INFO(
      get_logger(), "drift injector ready: %s -> %s, frames=%s/%s/%s",
      reference_odom_topic_.c_str(), output_odom_topic_.c_str(), odom_frame_.c_str(),
      base_frame_.c_str(), laser_frame_.c_str());
  }

private:
  void on_odometry(const nav_msgs::msg::Odometry & reference_message)
  {
    const Pose2d reference = pose_from_odometry(reference_message);
    const Pose2d estimate = drift_model_.update(reference);
    reference_samples_.push_back(reference);
    estimate_samples_.push_back(estimate);
    if (reference_samples_.size() > max_path_samples_) {
      reference_samples_.erase(reference_samples_.begin());
      estimate_samples_.erase(estimate_samples_.begin());
    }

    nav_msgs::msg::Odometry output = reference_message;
    output.header.frame_id = odom_frame_;
    output.child_frame_id = base_frame_;
    output.pose.pose.position.x = estimate.x;
    output.pose.pose.position.y = estimate.y;
    set_yaw(output.pose.pose.orientation, estimate.yaw);
    odom_pub_->publish(output);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = output.header;
    transform.child_frame_id = base_frame_;
    transform.transform.translation.x = estimate.x;
    transform.transform.translation.y = estimate.y;
    transform.transform.translation.z = 0.0;
    set_yaw(transform.transform.rotation, estimate.yaw);
    tf_broadcaster_.sendTransform(transform);

    append_path_pose(reference_path_, output.header.stamp, reference);
    append_path_pose(raw_path_, output.header.stamp, estimate);
    if (reference_samples_.size() % 5U == 0U) {
      reference_path_pub_->publish(reference_path_);
      raw_path_pub_->publish(raw_path_);
    }
  }

  void on_scan(const sensor_msgs::msg::LaserScan & input)
  {
    sensor_msgs::msg::LaserScan output = input;
    // LaserScan 数值仍位于同一传感器坐标系，只改为隔离 TF 树中的等价 frame。
    output.header.frame_id = laser_frame_;
    scan_pub_->publish(output);
  }

  void append_path_pose(nav_msgs::msg::Path & path, const builtin_interfaces::msg::Time & stamp,
    const Pose2d & pose)
  {
    path.header.frame_id = odom_frame_;
    path.header.stamp = stamp;
    geometry_msgs::msg::PoseStamped item;
    item.header = path.header;
    item.pose.position.x = pose.x;
    item.pose.position.y = pose.y;
    set_yaw(item.pose.orientation, pose.yaw);
    path.poses.push_back(item);
    if (path.poses.size() > max_path_samples_) {
      path.poses.erase(path.poses.begin());
    }
  }

  void publish_laser_transform()
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = now();
    transform.header.frame_id = base_frame_;
    transform.child_frame_id = laser_frame_;
    transform.transform.translation.z = laser_z_;
    transform.transform.rotation.w = 1.0;
    static_tf_broadcaster_.sendTransform(transform);
  }

  void publish_diagnostics()
  {
    const TrajectoryMetrics metrics = evaluate_trajectory(reference_samples_, estimate_samples_);
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "embodied_slam/odom_drift";
    status.hardware_id = "gazebo_reference";
    status.level = reference_samples_.empty() ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN : diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = reference_samples_.empty() ? "waiting_for_odometry" : "injecting_controlled_drift";
    status.values = {
      metric_value("sample_count", static_cast<double>(metrics.sample_count)),
      metric_value("reference_path_length_m", metrics.reference_path_length_m),
      metric_value("raw_path_length_m", metrics.estimate_path_length_m),
      metric_value("ate_rmse_m", metrics.ate_rmse_m),
      metric_value("final_position_error_m", metrics.final_position_error_m),
      metric_value("final_yaw_error_rad", metrics.final_yaw_error_rad),
      metric_value("raw_closure_error_m", metrics.closure_error_m)};
    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    array.status.push_back(status);
    diagnostics_pub_->publish(array);
  }

  DriftModel drift_model_;
  std::string reference_odom_topic_;
  std::string input_scan_topic_;
  std::string output_odom_topic_;
  std::string output_scan_topic_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string laser_frame_;
  double laser_z_{0.18};
  std::size_t max_path_samples_{6000U};
  std::vector<Pose2d> reference_samples_;
  std::vector<Pose2d> estimate_samples_;
  nav_msgs::msg::Path reference_path_;
  nav_msgs::msg::Path raw_path_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr reference_path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raw_path_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  tf2_ros::StaticTransformBroadcaster static_tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace embodied_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_slam::OdomDriftInjectorNode>());
  rclcpp::shutdown();
  return 0;
}
