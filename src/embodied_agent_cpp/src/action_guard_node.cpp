#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "embodied_agent_cpp/action_validator.hpp"

namespace embodied_agent_cpp
{

class ActionGuardNode : public rclcpp::Node
{
public:
  ActionGuardNode()
  : Node("action_guard")
  {
    command_publisher_ = create_publisher<std_msgs::msg::String>("/robot/action_command", 10);
    rejection_publisher_ = create_publisher<std_msgs::msg::String>("/robot/action_rejected", 10);
    candidate_subscription_ = create_subscription<std_msgs::msg::String>(
      "/agent/action_candidate",
      10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        const auto result = validator_.validate(message->data);
        std_msgs::msg::String output;
        if (!result.valid) {
          output.data = result.error;
          rejection_publisher_->publish(output);
          RCLCPP_WARN(get_logger(), "action rejected: %s", result.error.c_str());
          return;
        }
        output.data = result.command.dump();
        command_publisher_->publish(output);
        RCLCPP_INFO(get_logger(), "action accepted: %s", output.data.c_str());
      });
    RCLCPP_INFO(get_logger(), "C++ action guard ready");
  }

private:
  ActionValidator validator_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr rejection_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr candidate_subscription_;
};

}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_agent_cpp::ActionGuardNode>());
  rclcpp::shutdown();
  return 0;
}

