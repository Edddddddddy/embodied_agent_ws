#include <atomic>
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

#include "embodied_agent_cpp/action_validator.hpp"
#include "embodied_agent_interfaces/msg/robot_command.hpp"

namespace embodied_agent_cpp
{

class ActionGuardNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  ActionGuardNode()
  : LifecycleNode("action_guard")
  {
    RCLCPP_INFO(get_logger(), "ActionGuard lifecycle node created");
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    typed_command_publisher_ =
      create_publisher<embodied_agent_interfaces::msg::RobotCommand>(
      "/robot/action_command_typed", 10);
    rejection_publisher_ = create_publisher<std_msgs::msg::String>(
      "/robot/action_rejected", 10);
    candidate_subscription_ = create_subscription<
      embodied_agent_interfaces::msg::RobotCommand>(
      "/agent/action_candidate", 10,
      [this](
        const embodied_agent_interfaces::msg::RobotCommand::SharedPtr message) {
        on_candidate(message);
      });
    command_sequence_ = 0;
    RCLCPP_INFO(get_logger(), "ActionGuard configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    typed_command_publisher_->on_activate();
    rejection_publisher_->on_activate();
    RCLCPP_INFO(get_logger(), "ActionGuard activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    typed_command_publisher_->on_deactivate();
    rejection_publisher_->on_deactivate();
    RCLCPP_INFO(get_logger(), "ActionGuard deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    candidate_subscription_.reset();
    rejection_publisher_.reset();
    typed_command_publisher_.reset();
    command_sequence_ = 0;
    RCLCPP_INFO(get_logger(), "ActionGuard cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    candidate_subscription_.reset();
    rejection_publisher_.reset();
    typed_command_publisher_.reset();
    RCLCPP_INFO(get_logger(), "ActionGuard shut down");
    return CallbackReturn::SUCCESS;
  }

private:
  bool is_active() const
  {
    return get_current_state().id() ==
           lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
  }

  void on_candidate(
    const embodied_agent_interfaces::msg::RobotCommand::SharedPtr message)
  {
    if (!is_active()) {
      RCLCPP_DEBUG(get_logger(), "ignored action candidate while inactive");
      return;
    }
    const std::string command_id = "guard-" + std::to_string(++command_sequence_);
    // ActionGuard 是 Agent 输出和机器人执行之间的安全边界。候选消息虽然已经
    // 强类型化，数值范围、无关字段和动作白名单仍必须在 C++ 侧重新校验。
    auto result = validator_.validate(*message, command_id, "agent");
    std_msgs::msg::String output;
    if (!result.valid) {
      output.data = result.error;
      rejection_publisher_->publish(output);
      RCLCPP_WARN(get_logger(), "action rejected: %s", result.error.c_str());
      return;
    }
    result.command.header.stamp = now();
    typed_command_publisher_->publish(result.command);
    RCLCPP_INFO(
      get_logger(), "action accepted: command_id=%s type=%u",
      result.command.command_id.c_str(), result.command.action_type);
  }

  ActionValidator validator_;
  std::atomic_uint64_t command_sequence_{0};
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::RobotCommand>::SharedPtr
    typed_command_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
    rejection_publisher_;
  rclcpp::Subscription<
    embodied_agent_interfaces::msg::RobotCommand>::SharedPtr candidate_subscription_;
};

}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<embodied_agent_cpp::ActionGuardNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
