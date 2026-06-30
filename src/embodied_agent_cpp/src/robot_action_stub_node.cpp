#include <memory>
#include <string>

#include <nlohmann/json.hpp>

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
    command_subscription_ = create_subscription<std_msgs::msg::String>(
      "/robot/action_command",
      10,
      [this](const std_msgs::msg::String::SharedPtr message) {
        nlohmann::json acknowledgement;
        try {
          const auto command = nlohmann::json::parse(message->data);
          acknowledgement = {
            {"name", command.at("name")},
            {"status", "accepted"},
          };
          RCLCPP_INFO(get_logger(), "simulating action: %s", message->data.c_str());
        } catch (const nlohmann::json::exception & error) {
          acknowledgement = {
            {"status", "rejected"},
            {"error", error.what()},
          };
        }
        std_msgs::msg::String output;
        output.data = acknowledgement.dump();
        acknowledgement_publisher_->publish(output);
      });
    RCLCPP_INFO(get_logger(), "C++ robot action stub ready");
  }

private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr acknowledgement_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscription_;
};

}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_agent_cpp::RobotActionStubNode>());
  rclcpp::shutdown();
  return 0;
}

