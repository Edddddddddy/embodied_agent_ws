#include <atomic>
#include <algorithm>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"

#include "embodied_agent_cpp/action_authority.hpp"
#include "embodied_agent_cpp/action_validator.hpp"
#include "embodied_agent_cpp/control_authority_tracker.hpp"
#include "embodied_agent_cpp/guarded_command_outbox.hpp"
#include "embodied_agent_interfaces/msg/component_health.hpp"
#include "embodied_agent_interfaces/msg/control_authority_state.hpp"
#include "embodied_agent_interfaces/msg/robot_command.hpp"
#include "embodied_agent_middleware/qos_profiles.hpp"

namespace embodied_agent_cpp
{
namespace
{

using AuthorityState = embodied_agent_interfaces::msg::ControlAuthorityState;
using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

std::optional<ControlAuthority> decode_authority(const std::uint8_t authority)
{
  switch (authority) {
    case AuthorityState::HOLD:
      return ControlAuthority::kHold;
    case AuthorityState::AUTONOMY:
      return ControlAuthority::kAutonomy;
    case AuthorityState::KEYBOARD:
      return ControlAuthority::kKeyboard;
    case AuthorityState::ESTOP:
      return ControlAuthority::kEstop;
    default:
      return std::nullopt;
  }
}

bool is_priority_stop(const RobotCommand & command) noexcept
{
  return command.priority && command.action_type == RobotCommand::STOP;
}

}  // namespace

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
    const auto max_pending = static_cast<std::size_t>(std::max<std::int64_t>(
      1, declare_parameter<int>("downstream_buffer_size", 32)));
    downstream_wait_timeout_s_ = std::max(
      0.1, declare_parameter<double>("downstream_wait_timeout_s", 3.0));
    authority_gate_enabled_ = declare_parameter<bool>(
      "authority_gate_enabled", false);
    authority_state_timeout_s_ = std::max(
      0.1, declare_parameter<double>("authority_state_timeout_s", 1.0));
    authority_tracker_ = std::make_unique<ControlAuthorityTracker>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(authority_state_timeout_s_)));
    const auto authority_state_topic = declare_parameter<std::string>(
      "authority_state_topic", "/control/authority/state");
    outbox_ = std::make_unique<GuardedCommandOutbox>(max_pending);
    typed_command_publisher_ =
      create_publisher<embodied_agent_interfaces::msg::RobotCommand>(
      "/robot/action_command_typed", embodied_agent_middleware::command_qos());
    rejection_publisher_ = create_publisher<std_msgs::msg::String>(
      "/robot/action_rejected", embodied_agent_middleware::event_qos());
    health_publisher_ =
      create_publisher<embodied_agent_interfaces::msg::ComponentHealth>(
      "system/component_health", embodied_agent_middleware::state_qos());
    candidate_subscription_ = create_subscription<
      RobotCommand>(
      "/agent/action_candidate", embodied_agent_middleware::command_qos(),
      [this](
        const RobotCommand::SharedPtr message) {
        on_candidate(message);
      });
    authority_subscription_ = create_subscription<AuthorityState>(
      authority_state_topic, embodied_agent_middleware::state_qos(),
      [this](const AuthorityState::SharedPtr message) {
        const auto authority = decode_authority(message->authority);
        if (!authority) {
          RCLCPP_ERROR(
            get_logger(), "invalid control authority state=%u",
            static_cast<unsigned int>(message->authority));
          return;
        }
        ControlAuthoritySnapshot snapshot;
        snapshot.authority = *authority;
        snapshot.estop_latched = message->estop_latched;
        snapshot.manager_epoch = message->manager_epoch;
        snapshot.transition_sequence = message->transition_sequence;
        snapshot.pending_autonomy_revocation_sequence =
        message->pending_autonomy_revocation_sequence;
        snapshot.autonomy_quiescence_acknowledged =
        message->autonomy_quiescence_acknowledged;
        snapshot.active_source = message->active_source;
        snapshot.reason = message->reason;
        const auto update = authority_tracker_->update(
          snapshot, ControlAuthorityTracker::Clock::now());
        if (!update.accepted) {
          RCLCPP_ERROR(
            get_logger(),
            "control authority state rejected: epoch=%llu sequence=%llu reason=%s",
            static_cast<unsigned long long>(message->manager_epoch),
            static_cast<unsigned long long>(message->transition_sequence),
            std::string(authority_update_reason(update.decision)).c_str());
          return;
        }
        if (update.generation_changed) {
          RCLCPP_INFO(
            get_logger(),
            "control authority generation=%llu state=%u lease_discontinuity=%s",
            static_cast<unsigned long long>(authority_tracker_->generation()),
            static_cast<unsigned int>(message->authority),
            update.lease_discontinuity ? "true" : "false");
        }
        // 状态迁移或失联后的新心跳会改变 generation；立即清除旧普通动作，
        // 不等待 scheduler discovery，也不影响仍需送达的 priority STOP。
        flush_outbox();
      });
    outbox_timer_ = create_wall_timer(
      std::chrono::milliseconds(50), [this]() {flush_outbox();});
    outbox_timer_->cancel();
    command_sequence_ = 0;
    RCLCPP_INFO(get_logger(), "ActionGuard configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    typed_command_publisher_->on_activate();
    rejection_publisher_->on_activate();
    health_publisher_->on_activate();
    pipeline_ever_ready_ = false;
    publish_health(
      embodied_agent_interfaces::msg::ComponentHealth::STATE_STARTING,
      "waiting_for_action_pipeline_discovery");
    outbox_timer_->reset();
    RCLCPP_INFO(get_logger(), "ActionGuard activated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    outbox_timer_->cancel();
    outbox_->clear();
    publish_health(
      embodied_agent_interfaces::msg::ComponentHealth::STATE_STOPPED,
      "lifecycle_deactivated");
    health_publisher_->on_deactivate();
    typed_command_publisher_->on_deactivate();
    rejection_publisher_->on_deactivate();
    RCLCPP_INFO(get_logger(), "ActionGuard deactivated");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    outbox_timer_.reset();
    outbox_.reset();
    candidate_subscription_.reset();
    authority_subscription_.reset();
    authority_tracker_.reset();
    rejection_publisher_.reset();
    health_publisher_.reset();
    typed_command_publisher_.reset();
    command_sequence_ = 0;
    RCLCPP_INFO(get_logger(), "ActionGuard cleaned up");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    outbox_timer_.reset();
    outbox_.reset();
    candidate_subscription_.reset();
    authority_subscription_.reset();
    authority_tracker_.reset();
    rejection_publisher_.reset();
    health_publisher_.reset();
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
    const RobotCommand::SharedPtr message)
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
    const auto authority_decision = evaluate_action_authority(
      authority_gate_enabled_,
      authority_state_ready(),
      authority_tracker_ ?
      authority_tracker_->authority() : ControlAuthority::kHold,
      is_priority_stop(result.command));
    if (authority_decision != ActionAuthorityDecision::kAllow) {
      output.data = std::string(action_authority_reason(authority_decision));
      rejection_publisher_->publish(output);
      RCLCPP_WARN(
        get_logger(), "action rejected by control authority: command_id=%s reason=%s",
        result.command.command_id.c_str(), output.data.c_str());
      return;
    }
    const std::uint64_t authority_generation =
      authority_gate_enabled_ && authority_tracker_ ?
      authority_tracker_->generation() : 0U;
    if (!outbox_->enqueue(
        result.command, steady_now_seconds(), authority_generation))
    {
      output.data = "action_downstream_buffer_full:" + result.command.command_id;
      rejection_publisher_->publish(output);
      RCLCPP_ERROR(
        get_logger(), "action downstream buffer full: command_id=%s",
        result.command.command_id.c_str());
      return;
    }
    RCLCPP_INFO(
      get_logger(), "action accepted: command_id=%s type=%u",
      result.command.command_id.c_str(), result.command.action_type);
    flush_outbox();
  }

  double steady_now_seconds() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now().time_since_epoch()).count();
  }

  bool authority_state_ready() const
  {
    return authority_tracker_ &&
           authority_tracker_->fresh(ControlAuthorityTracker::Clock::now());
  }

  std::optional<std::uint64_t> allowed_authority_generation() const
  {
    if (!authority_gate_enabled_) {
      return 0U;
    }
    if (
      authority_tracker_ &&
      authority_tracker_->fresh_autonomy(ControlAuthorityTracker::Clock::now()))
    {
      return authority_tracker_->generation();
    }
    return std::nullopt;
  }

  void flush_outbox()
  {
    if (!is_active() || !outbox_) {
      return;
    }
    const bool downstream_ready =
      typed_command_publisher_->get_subscription_count() > 0;
    const bool upstream_ready =
      candidate_subscription_->get_publisher_count() > 0;
    const bool authority_ready =
      !authority_gate_enabled_ || authority_state_ready();
    const bool pipeline_ready =
      upstream_ready && downstream_ready && authority_ready;
    const auto drain = outbox_->drain(
      downstream_ready,
      steady_now_seconds(), downstream_wait_timeout_s_,
      allowed_authority_generation());
    if (pipeline_ready) {
      pipeline_ever_ready_ = true;
      publish_health(
        embodied_agent_interfaces::msg::ComponentHealth::STATE_READY,
        "agent_guard_scheduler_matched");
    } else {
      // Lifecycle ACTIVE 只表示回调可运行；DDS discovery 完成前发布的 volatile
      // 动作不会被补发。readiness 必须同时证明 Agent→Guard 和 Guard→Scheduler
      // 两段端点已匹配，自动任务才能安全投递第一条非幂等运动命令。
      const std::string detail = !authority_ready ?
        (authority_tracker_ && authority_tracker_->observed() ?
        "control_authority_state_stale" :
        "waiting_for_control_authority_state") :
        (!upstream_ready ?
        "waiting_for_agent_publisher" : "waiting_for_action_scheduler");
      publish_health(
        pipeline_ever_ready_ ?
        embodied_agent_interfaces::msg::ComponentHealth::STATE_DEGRADED :
        embodied_agent_interfaces::msg::ComponentHealth::STATE_STARTING,
        pipeline_ever_ready_ ? "action_pipeline_disconnected" : detail);
    }
    for (const auto & command_id : drain.expired_command_ids) {
      std_msgs::msg::String rejection;
      rejection.data = "action_downstream_unavailable:" + command_id;
      rejection_publisher_->publish(rejection);
      RCLCPP_ERROR(
        get_logger(), "action expired before scheduler discovery: command_id=%s",
        command_id.c_str());
      publish_health(
        embodied_agent_interfaces::msg::ComponentHealth::STATE_DEGRADED,
        "action_scheduler_unavailable");
    }
    for (const auto & command_id : drain.invalidated_command_ids) {
      std_msgs::msg::String rejection;
      rejection.data = "action_authority_generation_invalidated:" + command_id;
      rejection_publisher_->publish(rejection);
      RCLCPP_WARN(
        get_logger(),
        "buffered action invalidated by control authority: command_id=%s",
        command_id.c_str());
    }
    for (const auto & command_id : drain.superseded_stop_command_ids) {
      std_msgs::msg::String rejection;
      rejection.data = "priority_stop_superseded:" + command_id;
      rejection_publisher_->publish(rejection);
      RCLCPP_WARN(
        get_logger(),
        "buffered priority STOP superseded by a newer exact command: command_id=%s",
        command_id.c_str());
    }
    for (auto command : drain.ready) {
      command.header.stamp = now();
      typed_command_publisher_->publish(command);
    }
  }

  void publish_health(const std::uint8_t state, const std::string & detail)
  {
    const double now_s = steady_now_seconds();
    if (!health_publisher_ || !health_publisher_->is_activated() ||
      (last_health_state_ == state && last_health_detail_ == detail &&
      now_s - last_health_publish_s_ < 1.0))
    {
      return;
    }
    embodied_agent_interfaces::msg::ComponentHealth message;
    message.stamp = now();
    message.component = "action_guard";
    message.state = state;
    message.detail = detail;
    health_publisher_->publish(message);
    last_health_state_ = state;
    last_health_detail_ = detail;
    last_health_publish_s_ = now_s;
  }

  ActionValidator validator_;
  std::atomic_uint64_t command_sequence_{0};
  double downstream_wait_timeout_s_{3.0};
  double authority_state_timeout_s_{1.0};
  bool authority_gate_enabled_{false};
  bool pipeline_ever_ready_{false};
  std::uint8_t last_health_state_{255};
  std::string last_health_detail_;
  double last_health_publish_s_{0.0};
  std::unique_ptr<GuardedCommandOutbox> outbox_;
  std::unique_ptr<ControlAuthorityTracker> authority_tracker_;
  rclcpp::TimerBase::SharedPtr outbox_timer_;
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::RobotCommand>::SharedPtr
    typed_command_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr
    rejection_publisher_;
  rclcpp_lifecycle::LifecyclePublisher<
    embodied_agent_interfaces::msg::ComponentHealth>::SharedPtr health_publisher_;
  rclcpp::Subscription<
    RobotCommand>::SharedPtr candidate_subscription_;
  rclcpp::Subscription<AuthorityState>::SharedPtr authority_subscription_;
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
