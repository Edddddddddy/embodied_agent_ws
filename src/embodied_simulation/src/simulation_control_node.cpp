#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include "embodied_simulation/simulation_controller.hpp"

namespace embodied_simulation
{

class SimulationControlNode : public rclcpp::Node
{
public:
  SimulationControlNode()
  : Node("simulation_control"), controller_(load_config())
  {
    using std::placeholders::_1;
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    mode_pub_ = create_publisher<std_msgs::msg::String>("/robot/control_mode", 10);
    state_pub_ = create_publisher<std_msgs::msg::String>("/robot/simulation_state", 10);
    action_ack_pub_ = create_publisher<std_msgs::msg::String>("/robot/action_ack", 10);
    action_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot/action_command", 10,
      std::bind(&SimulationControlNode::on_action, this, _1));
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot/control_mode_request", 10,
      std::bind(&SimulationControlNode::on_mode_request, this, _1));
    emergency_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/robot/emergency_stop", 10,
      std::bind(&SimulationControlNode::on_emergency_stop, this, _1));

    auto scan_qos = rclcpp::SensorDataQoS().keep_last(5);
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", scan_qos,
      std::bind(&SimulationControlNode::on_scan, this, _1));

    const auto period = std::chrono::duration<double>(
      1.0 / get_parameter("control_rate_hz").as_double());
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SimulationControlNode::control_tick, this));
    publish_mode();
    RCLCPP_INFO(get_logger(), "simulation control ready; mode=manual");
  }

private:
  ControllerConfig load_config()
  {
    ControllerConfig config;
    config.max_linear_speed = declare_parameter("max_linear_speed", 0.22);
    config.max_angular_speed = declare_parameter("max_angular_speed", 1.0);
    config.linear_acceleration = declare_parameter("linear_acceleration", 0.5);
    config.angular_acceleration = declare_parameter("angular_acceleration", 2.0);
    config.emergency_distance = declare_parameter("emergency_distance", 0.22);
    config.obstacle_distance = declare_parameter("obstacle_distance", 0.50);
    config.wall_target_distance = declare_parameter("wall_target_distance", 0.40);
    config.autonomous_linear_speed = declare_parameter("autonomous_linear_speed", 0.14);
    config.obstacle_turn_speed = declare_parameter("obstacle_turn_speed", 0.65);
    config.scan_timeout = declare_parameter("scan_timeout", 0.50);
    config.wall_kp = declare_parameter("wall_kp", 1.8);
    config.wall_ki = declare_parameter("wall_ki", 0.0);
    config.wall_kd = declare_parameter("wall_kd", 0.15);
    declare_parameter("control_rate_hz", 20.0);
    return config;
  }

  double now_seconds() const
  {
    return get_clock()->now().seconds();
  }

  void on_action(const std_msgs::msg::String::SharedPtr message)
  {
    try {
      const auto command = nlohmann::json::parse(message->data);
      const std::string name = command.at("name").get<std::string>();
      const auto & arguments = command.at("arguments");
      if (name == "move") {
        controller_.set_manual_command(
          arguments.at("linear_x").get<double>(), 0.0,
          arguments.at("duration_s").get<double>(), now_seconds());
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "turn") {
        controller_.set_manual_command(
          0.0, arguments.at("angular_z").get<double>(),
          arguments.at("duration_s").get<double>(), now_seconds());
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "stop") {
        controller_.stop();
        publish_mode();
        publish_action_ack(name, "accepted");
      } else if (name == "set_mode") {
        const bool accepted = set_mode(arguments.at("mode").get<std::string>());
        publish_action_ack(name, accepted ? "accepted" : "rejected");
      } else {
        publish_action_ack(name, "ignored");
      }
    } catch (const std::exception & error) {
      RCLCPP_WARN(get_logger(), "ignored malformed trusted action: %s", error.what());
      publish_action_ack("unknown", "rejected", error.what());
    }
  }

  void on_mode_request(const std_msgs::msg::String::SharedPtr message)
  {
    set_mode(message->data);
  }

  void on_emergency_stop(const std_msgs::msg::Empty::SharedPtr)
  {
    controller_.stop();
    publish_mode();
    publish_action_ack("stop", "accepted", "emergency_stop");
    RCLCPP_WARN(get_logger(), "emergency stop received; switched to manual");
  }

  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr message)
  {
    controller_.update_scan(
      message->ranges, message->angle_min, message->angle_increment,
      message->range_min, message->range_max, now_seconds());
  }

  bool set_mode(const std::string & mode)
  {
    if (!controller_.set_mode(mode)) {
      RCLCPP_WARN(get_logger(), "unsupported control mode: %s", mode.c_str());
      return false;
    }
    publish_mode();
    RCLCPP_INFO(get_logger(), "control mode changed to %s", mode.c_str());
    return true;
  }

  void publish_action_ack(
    const std::string & action,
    const std::string & status,
    const std::string & detail = "")
  {
    nlohmann::json payload{
      {"action", action},
      {"backend", "simulation"},
      {"sequence", ++action_sequence_},
      {"status", status},
    };
    if (!detail.empty()) {
      payload["detail"] = detail;
    }
    std_msgs::msg::String message;
    message.data = payload.dump();
    action_ack_pub_->publish(message);
  }

  void publish_mode()
  {
    std_msgs::msg::String message;
    message.data = SimulationController::mode_name(controller_.mode());
    mode_pub_->publish(message);
  }

  void control_tick()
  {
    const auto output = controller_.step(now_seconds());
    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = output.velocity.linear_x;
    velocity.angular.z = output.velocity.angular_z;
    cmd_vel_pub_->publish(velocity);

    nlohmann::json state{
      {"mode", SimulationController::mode_name(output.mode)},
      {"sensor_stale", output.sensor_stale},
      {"safety_stopped", output.safety_stopped},
      {"front_distance", std::isfinite(output.front_distance) ?
        nlohmann::json(output.front_distance) : nlohmann::json(nullptr)},
      {"right_distance", std::isfinite(output.right_distance) ?
        nlohmann::json(output.right_distance) : nlohmann::json(nullptr)},
      {"reason", output.reason},
      {"linear_x", output.velocity.linear_x},
      {"angular_z", output.velocity.angular_z},
    };
    std_msgs::msg::String state_message;
    state_message.data = state.dump();
    state_pub_->publish(state_message);
  }

  SimulationController controller_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_ack_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::uint64_t action_sequence_{0};
};

}  // namespace embodied_simulation

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_simulation::SimulationControlNode>());
  rclcpp::shutdown();
  return 0;
}
