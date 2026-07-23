#include "embodied_agent_cpp/velocity_authority_gate.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace embodied_agent_cpp
{
namespace
{

void require_positive(
  const std::chrono::milliseconds value,
  const char * name)
{
  if (value <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument(std::string(name) + " must be greater than zero");
  }
}

void require_positive(const double value, const char * name)
{
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::invalid_argument(std::string(name) + " must be finite and greater than zero");
  }
}

bool timestamp_is_fresh(
  const VelocityAuthorityGate::TimePoint timestamp,
  const std::chrono::milliseconds timeout,
  const VelocityAuthorityGate::TimePoint now) noexcept
{
  return now >= timestamp && now - timestamp <= timeout;
}

}  // namespace

VelocityAuthorityGate::VelocityAuthorityGate(
  const VelocityAuthorityGateConfig & config)
: config_(config),
  authority_tracker_(config.authority_lease)
{
  require_positive(config_.authority_lease, "authority_lease");
  require_positive(config_.autonomy_timeout, "autonomy_timeout");
  require_positive(config_.keyboard_timeout, "keyboard_timeout");
  require_positive(config_.max_abs_linear_x, "max_abs_linear_x");
  require_positive(config_.max_abs_angular_z, "max_abs_angular_z");
  if (
    !std::isfinite(config_.planar_axis_epsilon) ||
    config_.planar_axis_epsilon < 0.0)
  {
    throw std::invalid_argument(
            "planar_axis_epsilon must be finite and greater than or equal to zero");
  }
  if (config_.expected_keyboard_source.empty()) {
    throw std::invalid_argument("expected_keyboard_source must not be empty");
  }
}

bool VelocityAuthorityGate::update_authority(
  const ControlAuthoritySnapshot & snapshot,
  const TimePoint received_at)
{
  // 所有控制权消费者必须复用同一套 epoch/序号/租约规则。尤其是 manager
  // 重启后的旧 epoch 不能通过“先发 seq=0/HOLD、再发 AUTONOMY”的 ABA
  // 序列重新取得底盘权限。
  const auto update = authority_tracker_.update(snapshot, received_at);
  if (!update.accepted) {
    return false;
  }
  if (!update.generation_changed) {
    return true;
  }

  authority_transition_at_ = received_at;
  // 每次切权都销毁旧速度缓存；仅比较回调接收时间无法证明 Twist 是在切权后
  // 发布的，因此恢复运动源还必须经过下方的零速/静默隔离期。
  autonomy_.reset();
  keyboard_.reset();
  autonomy_quarantine_active_ = false;
  keyboard_quarantine_active_ = false;
  autonomy_quarantine_quiet_since_.reset();
  keyboard_quarantine_quiet_since_.reset();
  if (snapshot.authority == ControlAuthority::kAutonomy) {
    start_source_quarantine(
      autonomy_, autonomy_quarantine_active_,
      autonomy_quarantine_quiet_since_, received_at);
  }
  return true;
}

bool VelocityAuthorityGate::update_autonomy(
  const VelocityCommand & velocity,
  const TimePoint received_at)
{
  return update_source(
    autonomy_, autonomy_quarantine_active_,
    autonomy_quarantine_quiet_since_, velocity,
    config_.autonomy_timeout, received_at);
}

bool VelocityAuthorityGate::update_keyboard(
  const VelocityCommand & velocity,
  const TimePoint received_at)
{
  return update_source(
    keyboard_, keyboard_quarantine_active_,
    keyboard_quarantine_quiet_since_, velocity,
    config_.keyboard_timeout, received_at);
}

VelocityGateOutput VelocityAuthorityGate::evaluate(const TimePoint now) const
{
  if (!authority_tracker_.observed() || !authority_transition_at_) {
    return VelocityGateOutput{
      VelocityCommand{}, false, VelocityGateDecision::kAuthorityUnseen};
  }
  if (!authority_tracker_.fresh(now)) {
    return VelocityGateOutput{
      VelocityCommand{}, false, VelocityGateDecision::kAuthorityStale};
  }

  switch (authority_tracker_.authority()) {
    case ControlAuthority::kHold:
      return VelocityGateOutput{
        VelocityCommand{}, false, VelocityGateDecision::kHold};
    case ControlAuthority::kEstop:
      return VelocityGateOutput{
        VelocityCommand{}, false, VelocityGateDecision::kEmergencyStop};
    case ControlAuthority::kAutonomy:
      return evaluate_source(
        autonomy_, autonomy_quarantine_active_,
        config_.autonomy_timeout, now,
        VelocityGateDecision::kAutonomyAuthorized,
        VelocityGateDecision::kAutonomyInputUnseen,
        VelocityGateDecision::kAutonomyInputStale,
        VelocityGateDecision::kAutonomyInputBeforeTransition,
        VelocityGateDecision::kAutonomyInputQuarantined);
    case ControlAuthority::kKeyboard:
      if (
        !authority_tracker_.snapshot() ||
        authority_tracker_.snapshot()->active_source !=
        config_.expected_keyboard_source)
      {
        return VelocityGateOutput{
          VelocityCommand{}, false, VelocityGateDecision::kKeyboardSourceMismatch};
      }
      return evaluate_source(
        keyboard_, keyboard_quarantine_active_,
        config_.keyboard_timeout, now,
        VelocityGateDecision::kKeyboardAuthorized,
        VelocityGateDecision::kKeyboardInputUnseen,
        VelocityGateDecision::kKeyboardInputStale,
        VelocityGateDecision::kKeyboardInputBeforeTransition,
        VelocityGateDecision::kKeyboardInputQuarantined);
  }

  return VelocityGateOutput{
    VelocityCommand{}, false, VelocityGateDecision::kHold};
}

bool VelocityAuthorityGate::update_source(
  std::optional<TimedVelocity> & source,
  bool & quarantine_active,
  std::optional<TimePoint> & quarantine_quiet_since,
  const VelocityCommand & velocity,
  const std::chrono::milliseconds timeout,
  const TimePoint received_at)
{
  // TurtleBot3 是平面差速底盘；拒绝侧移/升降/翻滚/俯仰以及越界速度，不能只
  // 截断单轴后继续执行，否则上游消息类型或坐标系错误会被掩盖成另一条运动指令。
  if (
    !is_finite(velocity) ||
    !is_planar(velocity, config_.planar_axis_epsilon) ||
    !is_within_safety_envelope(
      velocity, config_.max_abs_linear_x, config_.max_abs_angular_z))
  {
    source.reset();
    if (quarantine_active) {
      quarantine_quiet_since = received_at;
    }
    return false;
  }

  if (quarantine_active) {
    source.reset();
    if (is_zero(velocity)) {
      // 零速只作为新代际握手，不作为可复用速度缓存；下一条样本才可能运动。
      quarantine_active = false;
      quarantine_quiet_since.reset();
      return true;
    }
    if (
      quarantine_quiet_since &&
      received_at >= *quarantine_quiet_since &&
      received_at - *quarantine_quiet_since >= timeout)
    {
      // 完整静默窗后的第一条非零样本只负责结束隔离，仍然丢弃。这样一个恰好
      // 在边界后送达的旧 DDS 样本不能直接驱动底盘，只有其后的样本可执行。
      quarantine_active = false;
      quarantine_quiet_since.reset();
      return false;
    }
    // 隔离期内任何非零样本都可能来自撤权前的 reliable 队列；绝不输出，并从
    // 本次接收重新计算静默窗。连续旧流量会一直保持 fail-closed。
    quarantine_quiet_since = received_at;
    return false;
  }

  source = TimedVelocity{velocity, received_at};
  return true;
}

void VelocityAuthorityGate::start_source_quarantine(
  std::optional<TimedVelocity> & source,
  bool & quarantine_active,
  std::optional<TimePoint> & quarantine_quiet_since,
  const TimePoint transition_at)
{
  source.reset();
  quarantine_active = true;
  quarantine_quiet_since = transition_at;
}

VelocityGateOutput VelocityAuthorityGate::evaluate_source(
  const std::optional<TimedVelocity> & source,
  const bool quarantine_active,
  const std::chrono::milliseconds timeout,
  const TimePoint now,
  const VelocityGateDecision authorized,
  const VelocityGateDecision unseen,
  const VelocityGateDecision stale,
  const VelocityGateDecision before_transition,
  const VelocityGateDecision quarantined) const
{
  if (quarantine_active) {
    return VelocityGateOutput{VelocityCommand{}, false, quarantined};
  }
  if (!source) {
    return VelocityGateOutput{VelocityCommand{}, false, unseen};
  }
  // 切权前缓存的速度即使尚未超时也不能复用，否则 RESET/RESUME 会造成“幽灵复驶”。
  if (source->received_at < *authority_transition_at_) {
    return VelocityGateOutput{VelocityCommand{}, false, before_transition};
  }
  if (!timestamp_is_fresh(source->received_at, timeout, now)) {
    return VelocityGateOutput{VelocityCommand{}, false, stale};
  }
  return VelocityGateOutput{source->velocity, true, authorized};
}

const char * to_string(const VelocityGateDecision decision) noexcept
{
  switch (decision) {
    case VelocityGateDecision::kAutonomyAuthorized:
      return "autonomy_authorized";
    case VelocityGateDecision::kKeyboardAuthorized:
      return "keyboard_authorized";
    case VelocityGateDecision::kAuthorityUnseen:
      return "authority_unseen";
    case VelocityGateDecision::kAuthorityStale:
      return "authority_stale";
    case VelocityGateDecision::kHold:
      return "hold";
    case VelocityGateDecision::kEmergencyStop:
      return "emergency_stop";
    case VelocityGateDecision::kAutonomyInputUnseen:
      return "autonomy_input_unseen";
    case VelocityGateDecision::kAutonomyInputStale:
      return "autonomy_input_stale";
    case VelocityGateDecision::kAutonomyInputBeforeTransition:
      return "autonomy_input_before_transition";
    case VelocityGateDecision::kAutonomyInputQuarantined:
      return "autonomy_input_quarantined";
    case VelocityGateDecision::kKeyboardSourceMismatch:
      return "keyboard_source_mismatch";
    case VelocityGateDecision::kKeyboardInputUnseen:
      return "keyboard_input_unseen";
    case VelocityGateDecision::kKeyboardInputStale:
      return "keyboard_input_stale";
    case VelocityGateDecision::kKeyboardInputBeforeTransition:
      return "keyboard_input_before_transition";
    case VelocityGateDecision::kKeyboardInputQuarantined:
      return "keyboard_input_quarantined";
  }
  return "unknown";
}

bool is_finite(const VelocityCommand & velocity) noexcept
{
  return std::isfinite(velocity.linear_x) &&
         std::isfinite(velocity.linear_y) &&
         std::isfinite(velocity.linear_z) &&
         std::isfinite(velocity.angular_x) &&
         std::isfinite(velocity.angular_y) &&
         std::isfinite(velocity.angular_z);
}

bool is_zero(const VelocityCommand & velocity) noexcept
{
  return velocity.linear_x == 0.0 &&
         velocity.linear_y == 0.0 &&
         velocity.linear_z == 0.0 &&
         velocity.angular_x == 0.0 &&
         velocity.angular_y == 0.0 &&
         velocity.angular_z == 0.0;
}

bool is_planar(
  const VelocityCommand & velocity,
  const double axis_epsilon) noexcept
{
  return std::isfinite(axis_epsilon) &&
         axis_epsilon >= 0.0 &&
         std::abs(velocity.linear_y) <= axis_epsilon &&
         std::abs(velocity.linear_z) <= axis_epsilon &&
         std::abs(velocity.angular_x) <= axis_epsilon &&
         std::abs(velocity.angular_y) <= axis_epsilon;
}

bool is_within_safety_envelope(
  const VelocityCommand & velocity,
  const double max_abs_linear_x,
  const double max_abs_angular_z) noexcept
{
  return std::isfinite(max_abs_linear_x) &&
         std::isfinite(max_abs_angular_z) &&
         max_abs_linear_x > 0.0 &&
         max_abs_angular_z > 0.0 &&
         std::abs(velocity.linear_x) <= max_abs_linear_x &&
         std::abs(velocity.angular_z) <= max_abs_angular_z;
}

}  // namespace embodied_agent_cpp
