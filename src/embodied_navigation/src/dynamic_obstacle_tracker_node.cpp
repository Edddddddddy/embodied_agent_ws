#include <memory>
#include <string>
#include <vector>

#include <embodied_agent_interfaces/msg/dynamic_obstacle.hpp>
#include <embodied_agent_interfaces/msg/dynamic_obstacle_array.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "embodied_navigation/dynamic_obstacle_tracker.hpp"

namespace embodied_navigation
{

class DynamicObstacleTrackerNode : public rclcpp::Node
{
public:
  DynamicObstacleTrackerNode()
  : Node("dynamic_obstacle_tracker"), tracker_(load_config())
  {
    input_topic_ = declare_parameter("input_topic", "/perception/dynamic_obstacle_detections");
    output_topic_ = declare_parameter("output_topic", "/perception/dynamic_obstacles");
    publisher_ = create_publisher<embodied_agent_interfaces::msg::DynamicObstacleArray>(
      output_topic_, rclcpp::QoS(10).reliable());
    subscription_ = create_subscription<geometry_msgs::msg::PoseArray>(
      input_topic_, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::PoseArray::SharedPtr message) {on_detections(*message);});
  }

private:
  TrackerConfig load_config()
  {
    TrackerConfig config;
    config.association_distance_m = declare_parameter("association_distance_m", 0.8);
    config.velocity_smoothing = declare_parameter("velocity_smoothing", 0.65);
    config.track_timeout_s = declare_parameter("track_timeout_s", 1.0);
    config.default_radius_m = declare_parameter("default_radius_m", 0.25);
    config.motion_model = motion_model_from_string(
      declare_parameter("motion_model", std::string("constant_velocity")));
    config.measurement_noise_variance = declare_parameter(
      "measurement_noise_variance", 0.01);
    config.process_noise_variance = declare_parameter("process_noise_variance", 0.2);
    config.imm_stationary_process_noise = declare_parameter(
      "imm_stationary_process_noise", 0.01);
    config.imm_maneuver_process_noise = declare_parameter(
      "imm_maneuver_process_noise", 1.0);
    config.imm_stay_probability = declare_parameter("imm_stay_probability", 0.94);
    config.imm_stationary_velocity_decay = declare_parameter(
      "imm_stationary_velocity_decay", 0.2);
    return config;
  }

  void on_detections(const geometry_msgs::msg::PoseArray & message)
  {
    std::vector<Point2d> observations;
    observations.reserve(message.poses.size());
    for (const auto & pose : message.poses) {
      observations.push_back(Point2d{pose.position.x, pose.position.y});
    }
    const rclcpp::Time stamp(message.header.stamp);
    const double timestamp_s = stamp.nanoseconds() == 0 ? now().seconds() : stamp.seconds();
    const auto & tracks = tracker_.update(observations, timestamp_s);

    embodied_agent_interfaces::msg::DynamicObstacleArray output;
    output.header = message.header;
    if (output.header.stamp.sec == 0 && output.header.stamp.nanosec == 0) {
      output.header.stamp = now();
    }
    output.obstacles.reserve(tracks.size());
    for (const auto & track : tracks) {
      embodied_agent_interfaces::msg::DynamicObstacle obstacle;
      obstacle.track_id = track.id;
      obstacle.position.x = track.position.x;
      obstacle.position.y = track.position.y;
      obstacle.velocity.x = track.velocity.x;
      obstacle.velocity.y = track.velocity.y;
      obstacle.radius = static_cast<float>(track.radius);
      obstacle.confidence = static_cast<float>(track.confidence);
      output.obstacles.push_back(std::move(obstacle));
    }
    publisher_->publish(output);
  }

  DynamicObstacleTracker tracker_;
  std::string input_topic_;
  std::string output_topic_;
  rclcpp::Publisher<embodied_agent_interfaces::msg::DynamicObstacleArray>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr subscription_;
};

}  // namespace embodied_navigation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_navigation::DynamicObstacleTrackerNode>());
  rclcpp::shutdown();
  return 0;
}
