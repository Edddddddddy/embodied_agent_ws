#include "embodied_agent_cpp/control_authority.hpp"

#include <stdexcept>
#include <string>

namespace embodied_agent_cpp
{
namespace
{

std::string transition_reason(
  const ControlAuthorityRequest & request,
  const std::string & fallback)
{
  return request.reason.empty() ? fallback : request.reason;
}

}  // namespace

ControlAuthorityMachine::ControlAuthorityMachine(
  const std::uint64_t manager_epoch,
  const bool bootstrap_quiescence_acknowledged)
{
  if (manager_epoch == 0U) {
    throw std::invalid_argument("manager_epoch must be greater than zero");
  }
  state_.manager_epoch = manager_epoch;
  // 进程刚启动并不能证明旧 Explore/Nav2 goal 已经终止。默认保持未确认，
  // 只有能证明整套运动栈同时冷启动的测试才显式传 true。
  state_.autonomy_quiescence_acknowledged =
    bootstrap_quiescence_acknowledged;
}

ControlAuthorityTransition ControlAuthorityMachine::request(
  const ControlAuthorityRequest & request)
{
  if (request.requester.empty()) {
    return reject("requester_must_not_be_empty");
  }

  switch (request.command) {
    case ControlAuthorityCommand::kTakeKeyboard:
      if (state_.authority == ControlAuthority::kEstop) {
        return reject("reset_estop_before_keyboard_takeover");
      }
      if (state_.authority == ControlAuthority::kKeyboard) {
        if (state_.active_source != request.requester) {
          return reject("keyboard_owned_by_another_requester");
        }
        return idempotent("keyboard_already_active");
      }
      // 只有从 AUTONOMY 接管时才取消旧自动事务；HOLD 已保证没有运动所有者。
      return transition_to(
        ControlAuthority::kKeyboard, request.requester,
        transition_reason(request, "keyboard_takeover"), "keyboard_granted",
        state_.authority == ControlAuthority::kAutonomy);

    case ControlAuthorityCommand::kReleaseKeyboard:
      if (state_.authority == ControlAuthority::kEstop) {
        return reject("reset_estop_before_keyboard_release");
      }
      if (state_.authority == ControlAuthority::kHold) {
        return idempotent("keyboard_already_released");
      }
      if (state_.authority != ControlAuthority::kKeyboard) {
        return reject("keyboard_is_not_active");
      }
      if (state_.active_source != request.requester) {
        return reject("keyboard_owned_by_another_requester");
      }
      // deadman 松权必须停在 HOLD，并生成一次零速意图；禁止旧 Nav2 goal 自动复活。
      return transition_to(
        ControlAuthority::kHold, "", transition_reason(request, "keyboard_released"),
        "keyboard_released_to_hold", true);

    case ControlAuthorityCommand::kEnterHold:
      if (state_.authority == ControlAuthority::kEstop) {
        return reject("reset_estop_before_entering_hold");
      }
      if (state_.authority == ControlAuthority::kHold) {
        return idempotent("already_holding");
      }
      return transition_to(
        ControlAuthority::kHold, "", transition_reason(request, "hold_requested"),
        "entered_hold", true);

    case ControlAuthorityCommand::kResumeAutonomy:
      if (state_.authority == ControlAuthority::kEstop) {
        return reject("reset_estop_before_resuming");
      }
      if (state_.authority == ControlAuthority::kKeyboard) {
        return reject("release_keyboard_before_resuming");
      }
      if (state_.authority == ControlAuthority::kAutonomy) {
        return idempotent("autonomy_already_active");
      }
      if (!state_.autonomy_quiescence_acknowledged) {
        // HOLD 只能说明速度门已归零，不能证明 Nav2 goal/SLAM explorer 已到达
        // terminal 状态；缺少同代 ACK 时恢复会让旧任务重新发布速度。
        return reject("autonomy_quiescence_not_acknowledged");
      }
      // RESUME 是恢复自动控制的唯一入口，RESET 本身绝不会走到这里。
      return transition_to(
        ControlAuthority::kAutonomy, "autonomy",
        transition_reason(request, "autonomy_resumed"), "autonomy_resumed", false);

    case ControlAuthorityCommand::kEmergencyStop:
      if (state_.authority == ControlAuthority::kEstop) {
        return idempotent("emergency_stop_already_latched");
      }
      return transition_to(
        ControlAuthority::kEstop, request.requester,
        transition_reason(request, "emergency_stop"), "emergency_stop_latched", true);

    case ControlAuthorityCommand::kResetEmergencyStop:
      if (state_.authority != ControlAuthority::kEstop) {
        return reject("emergency_stop_is_not_latched");
      }
      // RESET 只解除锁存并留在 HOLD；用户必须再发 RESUME 才能重新授权自动控制。
      return transition_to(
        ControlAuthority::kHold, "", transition_reason(request, "emergency_stop_reset"),
        "emergency_stop_reset_to_hold", false);
  }

  return reject("unsupported_authority_command");
}

ControlAuthorityTransition
ControlAuthorityMachine::acknowledge_autonomy_quiescence(
  const AutonomyQuiescenceAcknowledgement & acknowledgement)
{
  if (acknowledgement.requester.empty()) {
    return reject("requester_must_not_be_empty");
  }
  if (acknowledgement.manager_epoch != state_.manager_epoch) {
    return reject("manager_epoch_mismatch");
  }
  if (state_.authority == ControlAuthority::kAutonomy) {
    return reject("autonomy_is_active");
  }
  if (
    acknowledgement.autonomy_revocation_sequence !=
    state_.pending_autonomy_revocation_sequence)
  {
    return reject("autonomy_revocation_sequence_mismatch");
  }
  if (state_.autonomy_quiescence_acknowledged) {
    return idempotent("autonomy_quiescence_already_acknowledged");
  }

  // ACK 本身也是可观察状态变更，必须递增 transition_sequence；否则订阅端
  // 会把“同序号但 payload 改变”判为冲突心跳。pending 序号仍指向原撤销边沿。
  state_.autonomy_quiescence_acknowledged = true;
  ++state_.transition_sequence;
  state_.reason = acknowledgement.detail.empty() ?
    "autonomy_quiescence_acknowledged" : acknowledgement.detail;

  ControlAuthorityTransition output;
  output.accepted = true;
  output.changed = true;
  output.message = "autonomy_quiescence_acknowledged";
  output.state = state_;
  return output;
}

const ControlAuthoritySnapshot & ControlAuthorityMachine::snapshot() const noexcept
{
  return state_;
}

ControlAuthorityTransition ControlAuthorityMachine::transition_to(
  const ControlAuthority authority,
  const std::string & active_source,
  const std::string & reason,
  const std::string & message,
  const bool stop_requested)
{
  const bool revokes_autonomy =
    state_.authority == ControlAuthority::kAutonomy &&
    authority != ControlAuthority::kAutonomy;
  state_.authority = authority;
  state_.estop_latched = authority == ControlAuthority::kEstop;
  ++state_.transition_sequence;
  if (revokes_autonomy) {
    // 绑定“撤销请求”和“任务终止确认”的核心令牌。之后即使在
    // KEYBOARD/HOLD/ESTOP 间迁移，也不能覆盖这个待确认序号。
    state_.pending_autonomy_revocation_sequence =
      state_.transition_sequence;
    state_.autonomy_quiescence_acknowledged = false;
  } else if (authority == ControlAuthority::kAutonomy) {
    // ACK 是一次性恢复许可；进入 AUTONOMY 后立即消费，下一次撤销必须重新确认。
    state_.pending_autonomy_revocation_sequence = 0U;
    state_.autonomy_quiescence_acknowledged = false;
  }
  state_.active_source = active_source;
  state_.reason = reason;

  ControlAuthorityTransition output;
  output.accepted = true;
  output.changed = true;
  output.stop_requested = stop_requested;
  output.message = message;
  output.state = state_;
  return output;
}

ControlAuthorityTransition ControlAuthorityMachine::idempotent(
  const std::string & message) const
{
  ControlAuthorityTransition output;
  output.accepted = true;
  output.message = message;
  output.state = state_;
  return output;
}

ControlAuthorityTransition ControlAuthorityMachine::reject(
  const std::string & message) const
{
  ControlAuthorityTransition output;
  output.message = message;
  output.state = state_;
  return output;
}

}  // namespace embodied_agent_cpp
