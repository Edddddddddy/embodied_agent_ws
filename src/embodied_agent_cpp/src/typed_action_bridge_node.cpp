#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <embodied_agent_interfaces/action/execute_robot_command.hpp>
#include <embodied_agent_interfaces/msg/component_health.hpp>
#include <embodied_agent_interfaces/msg/robot_command.hpp>
#include <embodied_agent_interfaces/msg/robot_command_feedback.hpp>
#include <embodied_agent_interfaces/msg/robot_command_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "embodied_agent_cpp/action_scheduler.hpp"
#include "embodied_agent_cpp/typed_action_bridge_factory.hpp"
#include "embodied_agent_middleware/qos_profiles.hpp"

namespace embodied_agent_cpp
{

class TypedActionBridgeNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using ExecuteRobotCommand = embodied_agent_interfaces::action::ExecuteRobotCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteRobotCommand>;
  using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;
  using RobotCommandFeedback = embodied_agent_interfaces::msg::RobotCommandFeedback;
  using RobotCommandResult = embodied_agent_interfaces::msg::RobotCommandResult;

  explicit TypedActionBridgeNode(const rclcpp::NodeOptions & options)
  : LifecycleNode("typed_action_bridge", "", options)
  {
    max_pending_commands_ = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("max_pending_commands", 16)));
    clear_queue_on_failure_ = declare_parameter<bool>("clear_queue_on_failure", true);
    cancel_timeout_s_ = declare_parameter<double>("cancel_timeout_s", 2.0);
    action_server_wait_timeout_ms_ =
      declare_parameter<int>("action_server_wait_timeout_ms", 1000);
    RCLCPP_INFO(get_logger(), "typed Action bridge lifecycle node created");
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    scheduler_ = std::make_unique<ActionScheduler>(
      max_pending_commands_, clear_queue_on_failure_);

    // 命令入口串行化，Action 回调允许并行；共享状态仍由细粒度 mutex 保护。
    // 这使 MultiThreadedExecutor 下的并发契约显式可审计，而不是依赖默认 callback group。
    command_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    action_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::Reentrant);
    diagnostics_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    client_ = rclcpp_action::create_client<ExecuteRobotCommand>(
      this, "robot/execute_command", action_callback_group_);
    feedback_pub_ = create_publisher<RobotCommandFeedback>(
      "robot/action_feedback", embodied_agent_middleware::event_qos());
    result_pub_ = create_publisher<RobotCommandResult>(
      "robot/action_result", embodied_agent_middleware::event_qos());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", embodied_agent_middleware::diagnostics_qos());
    health_pub_ = create_publisher<embodied_agent_interfaces::msg::ComponentHealth>(
      "system/component_health", embodied_agent_middleware::state_qos());
    rclcpp::SubscriptionOptions subscription_options;
    subscription_options.callback_group = command_callback_group_;
    command_sub_ = create_subscription<RobotCommand>(
      "robot/action_command_typed", embodied_agent_middleware::command_qos(),
      std::bind(&TypedActionBridgeNode::on_command, this, std::placeholders::_1),
      subscription_options);
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TypedActionBridgeNode::on_cancel_watchdog, this),
      action_callback_group_);
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TypedActionBridgeNode::publish_diagnostics, this),
      diagnostics_callback_group_);
    health_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      [this]() {publish_health();},
      diagnostics_callback_group_);
    watchdog_timer_->cancel();
    diagnostics_timer_->cancel();
    health_timer_->cancel();
    RCLCPP_INFO(get_logger(), "typed Action bridge configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    feedback_pub_->on_activate();
    result_pub_->on_activate();
    diagnostics_pub_->on_activate();
    health_pub_->on_activate();
    watchdog_timer_->reset();
    diagnostics_timer_->reset();
    health_timer_->reset();
    publish_health();
    RCLCPP_INFO(
      get_logger(),
      "typed Action scheduler ready: max_pending=%zu clear_on_failure=%s cancel_timeout=%.2fs",
      max_pending_commands_, clear_queue_on_failure_ ? "true" : "false",
      cancel_timeout_s_);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    watchdog_timer_->cancel();
    diagnostics_timer_->cancel();
    health_timer_->cancel();
    std::vector<SchedulerEvent> events;
    {
      std::lock_guard<std::mutex> lock(scheduler_mutex_);
      events = scheduler_->clear_all("lifecycle_deactivated");
    }
    // 先取消真实 Action、发布被清理命令的终态和诊断，再关闭 managed publishers。
    process_events(std::move(events));
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      // deactivate 已把所有命令置为 canceled；不能让旧 cancel watchdog 在下次
      // activate 后清掉一个新 goal。迟到的旧 Action result 会被 scheduler 忽略。
      active_goal_id_.clear();
      active_goal_handle_.reset();
      cancel_pending_id_.clear();
    }
    publish_diagnostics();
    publish_health(
      embodied_agent_interfaces::msg::ComponentHealth::STATE_STOPPED,
      "lifecycle_deactivated");
    feedback_pub_->on_deactivate();
    result_pub_->on_deactivate();
    diagnostics_pub_->on_deactivate();
    health_pub_->on_deactivate();
    RCLCPP_INFO(get_logger(), "typed Action bridge deactivated and queue cleared");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    reset_runtime();
    RCLCPP_INFO(get_logger(), "typed Action bridge cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    reset_runtime();
    return CallbackReturn::SUCCESS;
  }

private:
  bool is_active() const
  {
    return health_pub_ && health_pub_->is_activated();
  }

  void publish_health()
  {
    if (!is_active() || !client_) {
      return;
    }
    const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    const bool server_ready = client_->action_server_is_ready();
    const auto state = server_ready ?
      embodied_agent_interfaces::msg::ComponentHealth::STATE_READY :
      (action_server_ever_ready_ ?
      embodied_agent_interfaces::msg::ComponentHealth::STATE_DEGRADED :
      embodied_agent_interfaces::msg::ComponentHealth::STATE_STARTING);
    const std::string detail = server_ready ?
      "execute_command_action_server_ready" :
      (action_server_ever_ready_ ?
      "execute_command_action_server_disconnected" :
      "waiting_for_execute_command_action_server");
    action_server_ever_ready_ = action_server_ever_ready_ || server_ready;
    if (state == last_health_state_ && detail == last_health_detail_ &&
      now_s - last_health_publish_s_ < 1.0)
    {
      return;
    }
    publish_health(state, detail);
  }

  void publish_health(const std::uint8_t state, const std::string & detail)
  {
    if (!health_pub_ || !health_pub_->is_activated()) {
      return;
    }
    const double now_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
    if (state == last_health_state_ && detail == last_health_detail_ &&
      now_s - last_health_publish_s_ < 1.0)
    {
      return;
    }
    embodied_agent_interfaces::msg::ComponentHealth message;
    message.stamp = now();
    message.component = "typed_action_bridge";
    message.state = state;
    message.detail = detail;
    health_pub_->publish(message);
    last_health_state_ = state;
    last_health_detail_ = detail;
    last_health_publish_s_ = now_s;
  }

  void on_command(const RobotCommand::SharedPtr command)
  {
    if (!is_active()) {
      // inactive 状态仍保留订阅，用明确拒绝替代“消息悄悄进入下一次 activate”。
      RCLCPP_WARN(
        get_logger(), "rejected command while lifecycle inactive: command_id=%s",
        command->command_id.c_str());
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_error_ = "lifecycle_inactive";
      return;
    }
    std::vector<SchedulerEvent> events;
    {
      std::lock_guard<std::mutex> lock(scheduler_mutex_);
      events = scheduler_->enqueue(*command);
    }
    process_events(std::move(events));
  }

  void complete_active(
    const std::string & command_id,
    const bool success,
    const std::uint8_t status,
    const std::string & message)
  {
    std::vector<SchedulerEvent> events;
    {
      std::lock_guard<std::mutex> lock(scheduler_mutex_);
      if (!scheduler_) {
        return;
      }
      events = scheduler_->complete(command_id, success, status, message);
    }
    if (events.empty()) {
      RCLCPP_WARN(
        get_logger(), "ignored stale Action result: command_id=%s",
        command_id.c_str());
      return;
    }
    process_events(std::move(events));
  }

  void process_events(std::vector<SchedulerEvent> events)
  {
    for (const auto & event : events) {
      switch (event.kind) {
        case SchedulerEventKind::kDispatch:
          dispatch_goal(event.command);
          break;
        case SchedulerEventKind::kCancelActive:
          request_cancel(event.command_id);
          break;
        case SchedulerEventKind::kResult:
          publish_result(event);
          break;
        case SchedulerEventKind::kRejectedInput:
          record_input_rejection(event);
          break;
      }
    }
  }

  void dispatch_goal(const RobotCommand & command)
  {
    const auto wait_timeout = std::chrono::milliseconds(
      std::max(0, action_server_wait_timeout_ms_));
    if (!client_->wait_for_action_server(wait_timeout)) {
      RCLCPP_ERROR(
        get_logger(), "Action server unavailable: command_id=%s",
        command.command_id.c_str());
      complete_active(
        command.command_id, false, RobotCommandResult::STATUS_REJECTED,
        "action_server_unavailable");
      return;
    }

    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_id_ = command.command_id;
      active_goal_handle_.reset();
    }

    ExecuteRobotCommand::Goal goal;
    goal.command = command;
    const std::string command_id = command.command_id;
    rclcpp_action::Client<ExecuteRobotCommand>::SendGoalOptions options;
    options.goal_response_callback =
      [this, command_id](const GoalHandle::SharedPtr & handle) {
        if (!handle) {
          clear_goal_state(command_id);
          complete_active(
            command_id, false, RobotCommandResult::STATUS_REJECTED,
            "goal_rejected");
          return;
        }

        bool cancel_immediately = false;
        {
          std::lock_guard<std::mutex> lock(goal_mutex_);
          if (active_goal_id_ != command_id) {
            // watchdog 已推进到下一条命令时，迟到的旧 goal 不能继续执行。
            cancel_immediately = true;
          } else {
            active_goal_handle_ = handle;
            cancel_immediately = cancel_pending_id_ == command_id;
          }
        }
        if (cancel_immediately) {
          client_->async_cancel_goal(handle);
        }
      };
    options.feedback_callback =
      [this, command_id](
        GoalHandle::SharedPtr,
        const std::shared_ptr<const ExecuteRobotCommand::Feedback> feedback) {
        RobotCommandFeedback output;
        output.header.stamp = now();
        output.command_id = command_id;
        output.phase = feedback->phase;
        output.progress = feedback->progress;
        output.detail = feedback->detail;
        feedback_pub_->publish(output);
      };
    options.result_callback =
      [this, command_id](const GoalHandle::WrappedResult & wrapped) {
        clear_goal_state(command_id);
        if (!wrapped.result) {
          complete_active(
            command_id, false, RobotCommandResult::STATUS_REJECTED,
            "missing_action_result");
          return;
        }
        complete_active(
          command_id, wrapped.result->success,
          wrapped.result->status, wrapped.result->message);
      };

    RCLCPP_INFO(
      get_logger(), "dispatching Action goal: command_id=%s type=%u",
      command_id.c_str(), static_cast<unsigned int>(command.action_type));
    client_->async_send_goal(goal, options);
  }

  void request_cancel(const std::string & command_id)
  {
    GoalHandle::SharedPtr handle;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      cancel_pending_id_ = command_id;
      cancel_requested_at_ = std::chrono::steady_clock::now();
      if (active_goal_id_ == command_id) {
        handle = active_goal_handle_;
      }
    }
    RCLCPP_WARN(
      get_logger(), "priority command canceling active goal: command_id=%s",
      command_id.c_str());
    if (handle) {
      client_->async_cancel_goal(handle);
    }
  }

  void on_cancel_watchdog()
  {
    std::string timed_out_id;
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      if (cancel_pending_id_.empty()) {
        return;
      }
      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - cancel_requested_at_).count();
      if (elapsed < std::max(0.1, cancel_timeout_s_)) {
        return;
      }
      timed_out_id = cancel_pending_id_;
      cancel_pending_id_.clear();
      active_goal_id_.clear();
      active_goal_handle_.reset();
    }
    RCLCPP_ERROR(
      get_logger(), "Action cancel result timed out: command_id=%s",
      timed_out_id.c_str());
    complete_active(
      timed_out_id, false, RobotCommandResult::STATUS_TIMED_OUT,
      "cancel_result_timeout");
  }

  void clear_goal_state(const std::string & command_id)
  {
    std::lock_guard<std::mutex> lock(goal_mutex_);
    if (active_goal_id_ == command_id) {
      active_goal_id_.clear();
      active_goal_handle_.reset();
    }
    if (cancel_pending_id_ == command_id) {
      cancel_pending_id_.clear();
    }
  }

  void publish_result(const SchedulerEvent & event)
  {
    RobotCommandResult output;
    output.header.stamp = now();
    output.command_id = event.command_id;
    output.success = event.success;
    output.status = event.status;
    output.message = event.message;
    result_pub_->publish(output);
    if (!event.success) {
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_error_ = event.message;
    }
  }

  void record_input_rejection(const SchedulerEvent & event)
  {
    RCLCPP_ERROR(
      get_logger(), "scheduler input rejected: command_id=%s reason=%s",
      event.command_id.c_str(), event.message.c_str());
    std::lock_guard<std::mutex> lock(status_mutex_);
    last_error_ = event.message;
  }

  static diagnostic_msgs::msg::KeyValue key_value(
    const std::string & key,
    const std::string & value)
  {
    diagnostic_msgs::msg::KeyValue output;
    output.key = key;
    output.value = value;
    return output;
  }

  void publish_diagnostics()
  {
    if (!scheduler_ || !diagnostics_pub_ || !diagnostics_pub_->is_activated()) {
      return;
    }
    ActionSchedulerSnapshot snapshot;
    {
      std::lock_guard<std::mutex> lock(scheduler_mutex_);
      snapshot = scheduler_->snapshot();
    }
    std::string last_error;
    {
      std::lock_guard<std::mutex> lock(status_mutex_);
      last_error = last_error_;
    }

    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = get_fully_qualified_name() + std::string(": action_scheduler");
    status.hardware_id = "robot_action_client";
    status.level = snapshot.cancel_requested ?
      diagnostic_msgs::msg::DiagnosticStatus::WARN :
      diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = snapshot.state;
    status.values = {
      key_value("state", snapshot.state),
      key_value("active_command_id", snapshot.active_command_id),
      key_value("active_action_type", std::to_string(snapshot.active_action_type)),
      key_value("pending_count", std::to_string(snapshot.pending_count)),
      key_value("max_pending", std::to_string(snapshot.max_pending)),
      key_value("cancel_requested", snapshot.cancel_requested ? "true" : "false"),
      key_value("accepted_count", std::to_string(snapshot.accepted_count)),
      key_value("rejected_count", std::to_string(snapshot.rejected_count)),
      key_value("completed_count", std::to_string(snapshot.completed_count)),
      key_value("cleared_count", std::to_string(snapshot.cleared_count)),
      key_value("last_error", last_error),
    };

    diagnostic_msgs::msg::DiagnosticArray output;
    output.header.stamp = now();
    output.status.push_back(std::move(status));
    diagnostics_pub_->publish(output);
  }

  void reset_runtime()
  {
    watchdog_timer_.reset();
    diagnostics_timer_.reset();
    health_timer_.reset();
    command_sub_.reset();
    client_.reset();
    feedback_pub_.reset();
    result_pub_.reset();
    diagnostics_pub_.reset();
    health_pub_.reset();
    scheduler_.reset();
    command_callback_group_.reset();
    action_callback_group_.reset();
    diagnostics_callback_group_.reset();
    {
      std::lock_guard<std::mutex> lock(goal_mutex_);
      active_goal_id_.clear();
      active_goal_handle_.reset();
      cancel_pending_id_.clear();
    }
    last_health_state_ = 255;
    last_health_detail_.clear();
    last_health_publish_s_ = 0.0;
  }

  bool clear_queue_on_failure_{true};
  std::size_t max_pending_commands_{16};
  double cancel_timeout_s_{2.0};
  int action_server_wait_timeout_ms_{1000};
  std::unique_ptr<ActionScheduler> scheduler_;
  std::mutex scheduler_mutex_;

  std::mutex goal_mutex_;
  std::string active_goal_id_;
  GoalHandle::SharedPtr active_goal_handle_;
  std::string cancel_pending_id_;
  std::chrono::steady_clock::time_point cancel_requested_at_{};

  std::mutex status_mutex_;
  std::string last_error_;

  rclcpp_action::Client<ExecuteRobotCommand>::SharedPtr client_;
  rclcpp::Subscription<RobotCommand>::SharedPtr command_sub_;
  rclcpp_lifecycle::LifecyclePublisher<RobotCommandFeedback>::SharedPtr feedback_pub_;
  rclcpp_lifecycle::LifecyclePublisher<RobotCommandResult>::SharedPtr result_pub_;
  rclcpp_lifecycle::LifecyclePublisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
  diagnostics_pub_;
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::ComponentHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  rclcpp::CallbackGroup::SharedPtr command_callback_group_;
  rclcpp::CallbackGroup::SharedPtr action_callback_group_;
  rclcpp::CallbackGroup::SharedPtr diagnostics_callback_group_;
  bool action_server_ever_ready_{false};
  std::uint8_t last_health_state_{255};
  std::string last_health_detail_;
  double last_health_publish_s_{0.0};
};

}  // namespace embodied_agent_cpp

std::shared_ptr<rclcpp_lifecycle::LifecycleNode> embodied_agent_cpp::make_typed_action_bridge_node(
  const rclcpp::NodeOptions & options)
{
  return std::make_shared<embodied_agent_cpp::TypedActionBridgeNode>(options);
}

RCLCPP_COMPONENTS_REGISTER_NODE(embodied_agent_cpp::TypedActionBridgeNode)
