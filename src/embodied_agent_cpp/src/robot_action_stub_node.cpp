#include <memory>
#include <string>

#include "embodied_agent_interfaces/msg/robot_action_ack.hpp"
#include "embodied_agent_interfaces/msg/robot_command.hpp"
#include "embodied_agent_middleware/qos_profiles.hpp"
#include "rclcpp/rclcpp.hpp"

namespace embodied_agent_cpp
{

std::string action_name(const embodied_agent_interfaces::msg::RobotCommand & command)
{
  switch (command.action_type) {
    case embodied_agent_interfaces::msg::RobotCommand::MOVE: return "move";
    case embodied_agent_interfaces::msg::RobotCommand::TURN: return "turn";
    case embodied_agent_interfaces::msg::RobotCommand::STOP: return "stop";
    case embodied_agent_interfaces::msg::RobotCommand::WAVE: return "wave";
    case embodied_agent_interfaces::msg::RobotCommand::SET_LED: return "set_led";
    case embodied_agent_interfaces::msg::RobotCommand::SET_MODE: return "set_mode";
    case embodied_agent_interfaces::msg::RobotCommand::NAVIGATE_TO: return "navigate_to";
    case embodied_agent_interfaces::msg::RobotCommand::FOLLOW_WAYPOINTS: return "follow_waypoints";
    case embodied_agent_interfaces::msg::RobotCommand::CANCEL_NAVIGATION:
      return "cancel_navigation";
    case embodied_agent_interfaces::msg::RobotCommand::ARC: return "arc";
    default: return "unknown";
  }
}

class RobotActionStubNode : public rclcpp::Node
{
public:
  RobotActionStubNode()
  : Node("robot_action_stub")
  {
    acknowledgement_publisher_ =
      create_publisher<embodied_agent_interfaces::msg::RobotActionAck>(
      "/robot/action_ack", embodied_agent_middleware::event_qos());
    command_subscription_ =
      create_subscription<embodied_agent_interfaces::msg::RobotCommand>(
      "/robot/action_command_typed",
      embodied_agent_middleware::command_qos(),
      [this](const embodied_agent_interfaces::msg::RobotCommand::SharedPtr command) {
        RCLCPP_INFO(
          get_logger(), "simulating typed action: command_id=%s type=%u",
          command->command_id.c_str(), command->action_type);
        embodied_agent_interfaces::msg::RobotActionAck output;
        output.stamp = now();
        output.action = action_name(*command);
        output.backend = "stub";
        output.status = output.STATUS_ACCEPTED;
        output.detail = "command_id=" + command->command_id;
        acknowledgement_publisher_->publish(output);
      });
    RCLCPP_INFO(get_logger(), "C++ robot action stub ready");
  }

private:
  rclcpp::Publisher<embodied_agent_interfaces::msg::RobotActionAck>::SharedPtr
    acknowledgement_publisher_;
  rclcpp::Subscription<embodied_agent_interfaces::msg::RobotCommand>::SharedPtr
    command_subscription_;
};

}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_agent_cpp::RobotActionStubNode>());
  rclcpp::shutdown();
  return 0;
}
