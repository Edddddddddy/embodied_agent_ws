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

#include "embodied_agent_cpp/action_scheduler.hpp"
#include "embodied_agent_middleware/qos_profiles.hpp"

namespace embodied_agent_cpp
{

class TypedActionBridgeNode : public rclcpp::Node
{
public:
  using ExecuteRobotCommand = embodied_agent_interfaces::action::ExecuteRobotCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecuteRobotCommand>;
  using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;
  using RobotCommandFeedback = embodied_agent_interfaces::msg::RobotCommandFeedback;
  using RobotCommandResult = embodied_agent_interfaces::msg::RobotCommandResult;

  TypedActionBridgeNode()
  : Node("typed_action_bridge")
  {
    const auto max_pending = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("max_pending_commands", 16)));
    clear_queue_on_failure_ = declare_parameter<bool>("clear_queue_on_failure", true);
    cancel_timeout_s_ = declare_parameter<double>("cancel_timeout_s", 2.0);
    action_server_wait_timeout_ms_ =
      declare_parameter<int>("action_server_wait_timeout_ms", 1000);
    scheduler_ = std::make_unique<ActionScheduler>(
      max_pending, clear_queue_on_failure_);

    client_ = rclcpp_action::create_client<ExecuteRobotCommand>(
      this, "robot/execute_command");
    feedback_pub_ = create_publisher<RobotCommandFeedback>(
      "robot/action_feedback", embodied_agent_middleware::event_qos());
    result_pub_ = create_publisher<RobotCommandResult>(
      "robot/action_result", embodied_agent_middleware::event_qos());
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
      "/diagnostics", embodied_agent_middleware::diagnostics_qos());
    health_pub_ = create_publisher<embodied_agent_interfaces::msg::ComponentHealth>(
      "system/component_health", embodied_agent_middleware::state_qos());
    command_sub_ = create_subscription<RobotCommand>(
      "robot/action_command_typed", embodied_agent_middleware::command_qos(),
      std::bind(&TypedActionBridgeNode::on_command, this, std::placeholders::_1));
    watchdog_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&TypedActionBridgeNode::on_cancel_watchdog, this));
    diagnostics_timer_ = create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&TypedActionBridgeNode::publish_diagnostics, this));
    health_timer_ = create_wall_timer(
      std::chrono::milliseconds(250),
      std::bind(&TypedActionBridgeNode::publish_health, this));
    publish_health();
    RCLCPP_INFO(
      get_logger(),
      "typed Action scheduler ready: max_pending=%zu clear_on_failure=%s cancel_timeout=%.2fs",
      max_pending, clear_queue_on_failure_ ? "true" : "false",
      cancel_timeout_s_);
  }

private:
  void publish_health()
  {
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

  bool clear_queue_on_failure_{true};
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
  rclcpp::Publisher<RobotCommandFeedback>::SharedPtr feedback_pub_;
  rclcpp::Publisher<RobotCommandResult>::SharedPtr result_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::Publisher<embodied_agent_interfaces::msg::ComponentHealth>::SharedPtr health_pub_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr health_timer_;
  bool action_server_ever_ready_{false};
  std::uint8_t last_health_state_{255};
  std::string last_health_detail_;
  double last_health_publish_s_{0.0};
};

}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<embodied_agent_cpp::TypedActionBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
