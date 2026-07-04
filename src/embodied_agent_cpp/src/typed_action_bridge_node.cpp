#include <chrono>
#include <cstdint>
#include <memory>
#include <string>

#include <embodied_agent_interfaces/action/execute_robot_command.hpp>
#include <embodied_agent_interfaces/msg/robot_command.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>

namespace embodied_agent_cpp
{

class TypedActionBridgeNode : public rclcpp::Node
{
public:
  using ExecuteRobotCommand =
    embodied_agent_interfaces::action::ExecuteRobotCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteRobotCommand>;

  TypedActionBridgeNode()
  : Node("typed_action_bridge")
  {
    client_ = rclcpp_action::create_client<ExecuteRobotCommand>(
      this, "robot/execute_command");
    feedback_pub_ = create_publisher<std_msgs::msg::String>(
      "robot/action_feedback", rclcpp::QoS(10).reliable());
    result_pub_ = create_publisher<std_msgs::msg::String>(
      "robot/action_result", rclcpp::QoS(10).reliable());
    command_sub_ = create_subscription<
      embodied_agent_interfaces::msg::RobotCommand>(
      "robot/action_command_typed", rclcpp::QoS(10).reliable(),
      std::bind(&TypedActionBridgeNode::on_command, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "typed command to Action bridge ready");
  }

private:
  void publish_result(
    const std::string & command_id,
    bool success,
    std::uint8_t status,
    const std::string & message)
  {
    std_msgs::msg::String output;
    output.data = nlohmann::json{
      {"command_id", command_id},
      {"success", success},
      {"status", status},
      {"message", message},
    }.dump();
    result_pub_->publish(output);
  }

  void on_command(
    const embodied_agent_interfaces::msg::RobotCommand::SharedPtr command)
  {
    if (!client_->wait_for_action_server(std::chrono::seconds(1))) {
      publish_result(
        command->command_id, false,
        ExecuteRobotCommand::Result::STATUS_REJECTED,
        "action_server_unavailable");
      RCLCPP_WARN(get_logger(), "typed action server is unavailable");
      return;
    }

    ExecuteRobotCommand::Goal goal;
    goal.command = *command;
    const std::string command_id = command->command_id;
    rclcpp_action::Client<ExecuteRobotCommand>::SendGoalOptions options;
    // Topic 适合传递瞬时命令，但“移动一秒/转九十度”是可取消、带反馈的长动作。
    // bridge 把强类型 RobotCommand 转成 ROS 2 Action goal，同时把 feedback/result 转回可观察 topic。
    options.goal_response_callback =
      [this, command_id](const GoalHandle::SharedPtr & handle) {
        if (!handle) {
          publish_result(
            command_id, false,
            ExecuteRobotCommand::Result::STATUS_REJECTED,
            "goal_rejected");
        }
      };
    options.feedback_callback =
      [this, command_id](
        GoalHandle::SharedPtr,
        const std::shared_ptr<const ExecuteRobotCommand::Feedback> feedback) {
        std_msgs::msg::String output;
        output.data = nlohmann::json{
          {"command_id", command_id},
          {"phase", feedback->phase},
          {"progress", feedback->progress},
          {"detail", feedback->detail},
        }.dump();
        feedback_pub_->publish(output);
      };
    options.result_callback =
      [this, command_id](const GoalHandle::WrappedResult & wrapped) {
        if (!wrapped.result) {
          publish_result(
            command_id, false,
            ExecuteRobotCommand::Result::STATUS_REJECTED,
            "missing_action_result");
          return;
        }
        publish_result(
          command_id, wrapped.result->success,
          wrapped.result->status, wrapped.result->message);
      };
    client_->async_send_goal(goal, options);
  }

  rclcpp_action::Client<ExecuteRobotCommand>::SharedPtr client_;
  rclcpp::Subscription<embodied_agent_interfaces::msg::RobotCommand>::SharedPtr
    command_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr feedback_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_pub_;
};

}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_agent_cpp::TypedActionBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
