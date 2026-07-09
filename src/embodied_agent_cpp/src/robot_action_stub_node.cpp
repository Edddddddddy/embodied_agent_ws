#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "embodied_agent_interfaces/msg/robot_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace embodied_agent_cpp
{

class RobotActionStubNode : public rclcpp::Node
{
public:
  RobotActionStubNode()
  : Node("robot_action_stub")
  {
    acknowledgement_publisher_ = create_publisher<std_msgs::msg::String>("/robot/action_ack", 10);
    command_subscription_ =
      create_subscription<embodied_agent_interfaces::msg::RobotCommand>(
      "/robot/action_command_typed",
      10,
      [this](const embodied_agent_interfaces::msg::RobotCommand::SharedPtr command) {
        nlohmann::json acknowledgement = {
          {"action_type", command->action_type},
          {"command_id", command->command_id},
          {"status", "accepted"},
        };
        RCLCPP_INFO(
          get_logger(), "simulating typed action: command_id=%s type=%u",
          command->command_id.c_str(), command->action_type);
        std_msgs::msg::String output;
        output.data = acknowledgement.dump();
        acknowledgement_publisher_->publish(output);
      });
    RCLCPP_INFO(get_logger(), "C++ robot action stub ready");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr acknowledgement_publisher_;
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
