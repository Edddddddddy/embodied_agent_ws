#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

#include "embodied_agent_middleware/qos_profiles.hpp"
#include "embodied_slam/closed_loop_controller.hpp"

namespace embodied_slam
{

class ClosedLoopDriverNode : public rclcpp::Node
{
public:
  ClosedLoopDriverNode()
  : Node("slam_closed_loop_driver"), controller_(load_config())
  {
    start_delay_s_ = declare_parameter("start_delay_s", 5.0);
    stop_distance_m_ = declare_parameter("stop_distance_m", 0.28);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel", embodied_agent_middleware::command_qos(10));
    complete_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/slam/route_complete", embodied_agent_middleware::state_qos());
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", embodied_agent_middleware::sensor_qos(20),
      [this](const nav_msgs::msg::Odometry::SharedPtr message) {on_odometry(*message);});
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", embodied_agent_middleware::sensor_qos(10),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr message) {on_scan(*message);});
    timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {step();});
  }

  ~ClosedLoopDriverNode() override
  {
    publish_stop();
  }

private:
  ClosedLoopConfig load_config()
  {
    ClosedLoopConfig config;
    config.segment_length_m = declare_parameter("segment_length_m", 2.0);
    config.linear_speed_mps = declare_parameter("linear_speed_mps", 0.18);
    config.angular_speed_rps = declare_parameter("angular_speed_rps", 0.45);
    config.position_tolerance_m = declare_parameter("position_tolerance_m", 0.04);
    config.angle_tolerance_rad = declare_parameter("angle_tolerance_rad", 0.035);
    config.loop_count = static_cast<std::size_t>(declare_parameter("loop_count", 1));
    return config;
  }

  void on_odometry(const nav_msgs::msg::Odometry & message)
  {
    const auto & orientation = message.pose.pose.orientation;
    const double sin_yaw = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);
    const double cos_yaw = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z);
    pose_ = Pose2d{
      message.pose.pose.position.x,
      message.pose.pose.position.y,
      std::atan2(sin_yaw, cos_yaw)};
    if (!has_pose_) {
      first_pose_time_ = now();
    }
    has_pose_ = true;
  }

  void on_scan(const sensor_msgs::msg::LaserScan & message)
  {
    front_distance_m_ = std::numeric_limits<double>::infinity();
    for (std::size_t index = 0U; index < message.ranges.size(); ++index) {
      const double angle = message.angle_min + static_cast<double>(index) * message.angle_increment;
      const double value = message.ranges[index];
      if (std::abs(angle) <= 0.30 && std::isfinite(value) && value >= message.range_min &&
        value <= message.range_max)
      {
        front_distance_m_ = std::min(front_distance_m_, value);
      }
    }
  }

  void step()
  {
    if (!has_pose_ || complete_) {
      publish_stop();
      return;
    }
    if ((now() - first_pose_time_).seconds() < start_delay_s_) {
      publish_stop();
      return;
    }
    if (front_distance_m_ < stop_distance_m_) {
      // 固定路线仍保留雷达硬停车，避免为了采集数据而越过基础安全边界。
      publish_stop();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "route paused: obstacle %.3fm", front_distance_m_);
      return;
    }

    const DriveCommand command = controller_.update(pose_);
    geometry_msgs::msg::Twist twist;
    twist.linear.x = command.linear_x;
    twist.angular.z = command.angular_z;
    cmd_pub_->publish(twist);
    if (command.complete) {
      complete_ = true;
      complete_pub_->publish(std_msgs::msg::Bool().set__data(true));
      RCLCPP_INFO(get_logger(), "closed loop route complete: sides=%zu", controller_.completed_sides());
    }
  }

  void publish_stop()
  {
    if (cmd_pub_) {
      cmd_pub_->publish(geometry_msgs::msg::Twist{});
    }
  }

  ClosedLoopController controller_;
  Pose2d pose_;
  bool has_pose_{false};
  bool complete_{false};
  double start_delay_s_{5.0};
  double stop_distance_m_{0.28};
  double front_distance_m_{std::numeric_limits<double>::infinity()};
  rclcpp::Time first_pose_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr complete_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace embodied_slam

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_slam::ClosedLoopDriverNode>());
  rclcpp::shutdown();
  return 0;
}
