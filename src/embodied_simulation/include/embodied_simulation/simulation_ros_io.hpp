#pragma once

#include <cstdint>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <embodied_agent_interfaces/msg/behavior_tree_status.hpp>
#include <embodied_agent_interfaces/msg/component_health.hpp>
#include <embodied_agent_interfaces/msg/robot_action_ack.hpp>
#include <embodied_agent_interfaces/msg/simulation_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/string.hpp>

#include "embodied_simulation/command_behavior_tree.hpp"
#include "embodied_simulation/executor_diagnostics.hpp"
#include "embodied_simulation/simulation_controller.hpp"

namespace embodied_simulation
{

class SimulationRosIo
{
public:
  explicit SimulationRosIo(rclcpp_lifecycle::LifecycleNode & node);

  void configure();
  void activate(const std::string & mode, const std::string & backend);
  void deactivate(const std::string & detail);
  void reset();

  void publish_velocity(double linear_x, double angular_z);
  void publish_zero_velocity();
  void publish_state(const ControllerOutput & output);
  void publish_mode(const std::string & mode);
  void publish_action_ack(
    const std::string & action, const std::string & status,
    const std::string & backend, const std::string & detail = "");
  void publish_bt_status(
    const CommandTreeResult & result, const std::string & command_id);
  void publish_health(std::uint8_t state, const std::string & detail);
  void publish_diagnostics(const ExecutorDiagnosticSnapshot & snapshot);

private:
  static std::uint8_t action_ack_status(const std::string & status);
  static const char * tree_outcome_name(CommandTreeOutcome outcome);
  static std::uint8_t tree_outcome_value(CommandTreeOutcome outcome);

  rclcpp_lifecycle::LifecycleNode & node_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::SimulationState>::SharedPtr state_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::RobotActionAck>::SharedPtr action_ack_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::BehaviorTreeStatus>::SharedPtr bt_status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::ComponentHealth>::SharedPtr health_pub_;
  std::uint64_t action_sequence_{0};
  std::string last_bt_status_;
};

}  // namespace embodied_simulation
