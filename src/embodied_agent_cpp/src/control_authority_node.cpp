#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>

#include "embodied_agent_interfaces/msg/control_authority_state.hpp"
#include "embodied_agent_interfaces/msg/robot_command.hpp"
#include "embodied_agent_interfaces/srv/acknowledge_autonomy_quiescence.hpp"
#include "embodied_agent_interfaces/srv/set_control_authority.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"

#include "embodied_agent_cpp/control_authority.hpp"
#include "embodied_agent_middleware/qos_profiles.hpp"

namespace embodied_agent_cpp
{
namespace
{

using AuthorityStateMessage =
  embodied_agent_interfaces::msg::ControlAuthorityState;
using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;
using AcknowledgeAutonomyQuiescence =
  embodied_agent_interfaces::srv::AcknowledgeAutonomyQuiescence;
using SetControlAuthority =
  embodied_agent_interfaces::srv::SetControlAuthority;

std::uint64_t make_manager_epoch()
{
  const auto ticks = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto value = static_cast<std::uint64_t>(ticks);
  return value == 0U ? 1U : value;
}

std::optional<ControlAuthorityCommand> decode_command(const std::uint8_t command)
{
  switch (command) {
    case SetControlAuthority::Request::TAKE_KEYBOARD:
      return ControlAuthorityCommand::kTakeKeyboard;
    case SetControlAuthority::Request::RELEASE_KEYBOARD:
      return ControlAuthorityCommand::kReleaseKeyboard;
    case SetControlAuthority::Request::ENTER_HOLD:
      return ControlAuthorityCommand::kEnterHold;
    case SetControlAuthority::Request::RESUME_AUTONOMY:
      return ControlAuthorityCommand::kResumeAutonomy;
    case SetControlAuthority::Request::EMERGENCY_STOP:
      return ControlAuthorityCommand::kEmergencyStop;
    case SetControlAuthority::Request::RESET_EMERGENCY_STOP:
      return ControlAuthorityCommand::kResetEmergencyStop;
    default:
      return std::nullopt;
  }
}

std::uint8_t encode_authority(const ControlAuthority authority)
{
  switch (authority) {
    case ControlAuthority::kHold:
      return AuthorityStateMessage::HOLD;
    case ControlAuthority::kAutonomy:
      return AuthorityStateMessage::AUTONOMY;
    case ControlAuthority::kKeyboard:
      return AuthorityStateMessage::KEYBOARD;
    case ControlAuthority::kEstop:
      return AuthorityStateMessage::ESTOP;
  }
  return AuthorityStateMessage::HOLD;
}

}  // namespace

class ControlAuthorityNode final : public rclcpp::Node
{
public:
  ControlAuthorityNode()
  : Node("control_authority"),
    manager_epoch_(make_manager_epoch()),
    service_name_(declare_parameter(
        "service_name", std::string("/control/set_authority"))),
    acknowledgement_service_name_(declare_parameter(
        "acknowledgement_service_name",
        std::string("/control/acknowledge_autonomy_quiescence"))),
    state_topic_(declare_parameter(
        "state_topic", std::string("/control/authority/state"))),
    action_candidate_topic_(declare_parameter(
        "action_candidate_topic", std::string("/agent/action_candidate"))),
    compatibility_estop_topic_(declare_parameter(
        "compatibility_estop_topic", std::string("/robot/emergency_stop"))),
    state_heartbeat_ms_(declare_parameter("state_heartbeat_ms", 200)),
    bootstrap_quiescence_acknowledged_(declare_parameter(
        // manager 可能单独崩溃重启，而旧 Explore/Nav2 goal 仍然存活。
        // 默认必须等待同 epoch 的 typed quiescence ACK，不能把进程启动
        // 误当成“整套运动栈已经静默”的证据。
        "bootstrap_quiescence_acknowledged", false)),
    machine_(manager_epoch_, bootstrap_quiescence_acknowledged_)
  {
    if (state_heartbeat_ms_ < 50 || state_heartbeat_ms_ > 2000) {
      throw std::invalid_argument("state_heartbeat_ms must be in [50, 2000]");
    }

    state_publisher_ = create_publisher<AuthorityStateMessage>(
      state_topic_, embodied_agent_middleware::state_qos());
    stop_publisher_ = create_publisher<RobotCommand>(
      action_candidate_topic_, embodied_agent_middleware::command_qos());
    compatibility_estop_publisher_ = create_publisher<std_msgs::msg::Empty>(
      compatibility_estop_topic_, embodied_agent_middleware::command_qos(10));

    service_ = create_service<SetControlAuthority>(
      service_name_,
      [this](
        const SetControlAuthority::Request::SharedPtr request,
        SetControlAuthority::Response::SharedPtr response)
      {
        handle_request(*request, *response);
      });
    acknowledgement_service_ = create_service<AcknowledgeAutonomyQuiescence>(
      acknowledgement_service_name_,
      [this](
        const AcknowledgeAutonomyQuiescence::Request::SharedPtr request,
        AcknowledgeAutonomyQuiescence::Response::SharedPtr response)
      {
        handle_acknowledgement(*request, *response);
      });

    state_heartbeat_ = create_wall_timer(
      std::chrono::milliseconds(state_heartbeat_ms_),
      [this]() {
        ControlAuthoritySnapshot snapshot;
        {
          std::lock_guard<std::mutex> lock(machine_mutex_);
          snapshot = machine_.snapshot();
        }
        // 权限门用“状态租约”判断 manager 是否存活；只靠 transient-local 快照无法
        // 区分正常 HOLD 与 manager 崩溃，所以相同 transition_sequence 也要周期重发。
        state_publisher_->publish(to_message(snapshot));
      });

    const auto initial = machine_.snapshot();
    state_publisher_->publish(to_message(initial));
    RCLCPP_INFO(
      get_logger(),
      "control authority ready: epoch=%llu service=%s ack_service=%s state=%s",
      static_cast<unsigned long long>(manager_epoch_),
      service_name_.c_str(), acknowledgement_service_name_.c_str(),
      state_topic_.c_str());
  }

private:
  AuthorityStateMessage to_message(const ControlAuthoritySnapshot & snapshot)
  {
    AuthorityStateMessage message;
    message.stamp = now();
    message.authority = encode_authority(snapshot.authority);
    message.estop_latched = snapshot.estop_latched;
    message.manager_epoch = snapshot.manager_epoch;
    message.transition_sequence = snapshot.transition_sequence;
    message.pending_autonomy_revocation_sequence =
      snapshot.pending_autonomy_revocation_sequence;
    message.autonomy_quiescence_acknowledged =
      snapshot.autonomy_quiescence_acknowledged;
    message.active_source = snapshot.active_source;
    message.reason = snapshot.reason;
    return message;
  }

  void handle_request(
    const SetControlAuthority::Request & request,
    SetControlAuthority::Response & response)
  {
    const auto command = decode_command(request.command);
    if (!command) {
      ControlAuthoritySnapshot snapshot;
      {
        std::lock_guard<std::mutex> lock(machine_mutex_);
        snapshot = machine_.snapshot();
      }
      response.accepted = false;
      response.changed = false;
      response.stop_requested = false;
      response.message = "unsupported_authority_command";
      response.state = to_message(snapshot);
      return;
    }

    ControlAuthorityTransition transition;
    {
      std::lock_guard<std::mutex> lock(machine_mutex_);
      transition = machine_.request(ControlAuthorityRequest{
          *command, request.requester, request.reason});
    }
    response.accepted = transition.accepted;
    response.changed = transition.changed;
    response.stop_requested = transition.stop_requested;
    response.message = transition.message;
    response.state = to_message(transition.state);

    if (!transition.accepted) {
      RCLCPP_WARN(
        get_logger(), "authority request rejected: command=%u requester=%s reason=%s",
        static_cast<unsigned int>(request.command), request.requester.c_str(),
        transition.message.c_str());
      return;
    }
    if (!transition.changed) {
      return;
    }

    // 先切换 typed 权限状态，让速度门立即归零，再发布 STOP/cancel 边沿终止上游任务。
    state_publisher_->publish(response.state);
    if (transition.stop_requested) {
      publish_stop_intent(transition.state.transition_sequence, request.requester);
    }
    if (transition.state.authority == ControlAuthority::kEstop) {
      compatibility_estop_publisher_->publish(std_msgs::msg::Empty{});
    }

    RCLCPP_INFO(
      get_logger(),
      "authority transition=%llu state=%u source=%s stop=%s reason=%s",
      static_cast<unsigned long long>(transition.state.transition_sequence),
      static_cast<unsigned int>(response.state.authority),
      transition.state.active_source.c_str(),
      transition.stop_requested ? "true" : "false",
      transition.state.reason.c_str());
  }

  void handle_acknowledgement(
    const AcknowledgeAutonomyQuiescence::Request & request,
    AcknowledgeAutonomyQuiescence::Response & response)
  {
    ControlAuthorityTransition transition;
    {
      std::lock_guard<std::mutex> lock(machine_mutex_);
      transition = machine_.acknowledge_autonomy_quiescence(
        AutonomyQuiescenceAcknowledgement{
          request.manager_epoch,
          request.autonomy_revocation_sequence,
          request.requester,
          request.detail});
    }
    response.accepted = transition.accepted;
    response.changed = transition.changed;
    response.message = transition.message;
    response.state = to_message(transition.state);

    if (!transition.accepted) {
      RCLCPP_WARN(
        get_logger(),
        "autonomy quiescence ACK rejected: epoch=%llu revocation=%llu requester=%s reason=%s",
        static_cast<unsigned long long>(request.manager_epoch),
        static_cast<unsigned long long>(request.autonomy_revocation_sequence),
        request.requester.c_str(), transition.message.c_str());
      return;
    }
    if (transition.changed) {
      // 先发布 ACK 后的 typed state；恢复请求只有在看到同代 ack=true 后才会放行。
      state_publisher_->publish(response.state);
    }
    RCLCPP_INFO(
      get_logger(),
      "autonomy quiescence ACK accepted: epoch=%llu revocation=%llu requester=%s changed=%s",
      static_cast<unsigned long long>(request.manager_epoch),
      static_cast<unsigned long long>(request.autonomy_revocation_sequence),
      request.requester.c_str(), transition.changed ? "true" : "false");
  }

  void publish_stop_intent(
    const std::uint64_t transition_sequence,
    const std::string & requester)
  {
    RobotCommand stop;
    stop.header.stamp = now();
    // 把 manager epoch 纳入幂等键，防止 manager 重启后与调度器里尚未过期的
    // 旧 STOP command_id 冲突，导致本代真正的停止意图被去重。
    stop.command_id = "authority-stop-" + std::to_string(manager_epoch_) +
      "-" + std::to_string(transition_sequence);
    stop.source = requester.empty() ? "control_authority" :
      "control_authority:" + requester;
    stop.priority = true;
    stop.action_type = RobotCommand::STOP;
    stop_publisher_->publish(stop);
  }

  std::uint64_t manager_epoch_;
  std::string service_name_;
  std::string acknowledgement_service_name_;
  std::string state_topic_;
  std::string action_candidate_topic_;
  std::string compatibility_estop_topic_;
  int state_heartbeat_ms_;
  bool bootstrap_quiescence_acknowledged_;

  std::mutex machine_mutex_;
  ControlAuthorityMachine machine_;
  rclcpp::Publisher<AuthorityStateMessage>::SharedPtr state_publisher_;
  rclcpp::Publisher<RobotCommand>::SharedPtr stop_publisher_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr
    compatibility_estop_publisher_;
  rclcpp::Service<SetControlAuthority>::SharedPtr service_;
  rclcpp::Service<AcknowledgeAutonomyQuiescence>::SharedPtr
    acknowledgement_service_;
  rclcpp::TimerBase::SharedPtr state_heartbeat_;
};

}  // namespace embodied_agent_cpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<embodied_agent_cpp::ControlAuthorityNode>());
  } catch (const std::exception & error) {
    std::cerr << "control_authority failed: " << error.what() << '\n';
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
