#include "embodied_agent_cpp/control_authority_tracker.hpp"

#include <stdexcept>

namespace embodied_agent_cpp
{

ControlAuthorityTracker::ControlAuthorityTracker(
  const std::chrono::milliseconds lease)
: lease_(lease)
{
  if (lease_ <= std::chrono::milliseconds::zero()) {
    throw std::invalid_argument("authority lease must be greater than zero");
  }
}

AuthorityUpdateResult ControlAuthorityTracker::update(
  const ControlAuthoritySnapshot & snapshot,
  const TimePoint received_at)
{
  if (!payload_is_valid(snapshot)) {
    return {false, false, false, AuthorityUpdateDecision::kRejectedInvalidPayload};
  }

  if (!state_ || !received_at_) {
    return accept(
      snapshot, received_at, AuthorityUpdateDecision::kAcceptedInitial, true, false);
  }
  if (received_at < *received_at_) {
    return {false, false, false, AuthorityUpdateDecision::kRejectedReorderedReceipt};
  }

  if (snapshot.manager_epoch != state_->manager_epoch) {
    if (retired_epochs_.count(snapshot.manager_epoch) > 0U) {
      return {false, false, false, AuthorityUpdateDecision::kRejectedRetiredEpoch};
    }
    // 新 manager 只能从明确的 seq=0/HOLD 建立信任，防止旧进程迟到的
    // AUTONOMY 状态被误认为一次合法重启。
    if (!safe_manager_bootstrap(snapshot)) {
      return {
        false, false, false,
        AuthorityUpdateDecision::kRejectedUnsafeEpochBootstrap};
    }
    retired_epochs_.insert(state_->manager_epoch);
    return accept(
      snapshot, received_at, AuthorityUpdateDecision::kAcceptedManagerRestart,
      true, true);
  }

  // 租约一旦中断，当前 epoch 就永久失去运动授权。不能让同一 manager 在
  // SIGSTOP/调度长停顿后仅靠迟到心跳重新获得 AUTONOMY；恢复必须经过新
  // epoch 的 seq=0/HOLD bootstrap，才能与最终速度门保持同一安全语义。
  if (received_at - *received_at_ > lease_) {
    return {
      false, false, true,
      AuthorityUpdateDecision::kRejectedLeaseDiscontinuity};
  }

  if (snapshot.transition_sequence < state_->transition_sequence) {
    return {false, false, false, AuthorityUpdateDecision::kRejectedOldSequence};
  }
  if (snapshot.transition_sequence == state_->transition_sequence) {
    if (!payload_matches(snapshot)) {
      return {
        false, false, false,
        AuthorityUpdateDecision::kRejectedConflictingHeartbeat};
    }
    received_at_ = received_at;
    return {
      true, false, false, AuthorityUpdateDecision::kAcceptedHeartbeat};
  }

  return accept(
    snapshot, received_at, AuthorityUpdateDecision::kAcceptedTransition,
    true, false);
}

bool ControlAuthorityTracker::observed() const noexcept
{
  return state_.has_value() && received_at_.has_value();
}

bool ControlAuthorityTracker::fresh(const TimePoint now) const noexcept
{
  return state_ && received_at_ && now >= *received_at_ &&
         now - *received_at_ <= lease_;
}

bool ControlAuthorityTracker::fresh_autonomy(const TimePoint now) const noexcept
{
  return fresh(now) && state_->authority == ControlAuthority::kAutonomy;
}

std::uint64_t ControlAuthorityTracker::generation() const noexcept
{
  return generation_;
}

ControlAuthority ControlAuthorityTracker::authority() const noexcept
{
  return state_ ? state_->authority : ControlAuthority::kHold;
}

const std::optional<ControlAuthoritySnapshot> &
ControlAuthorityTracker::snapshot() const noexcept
{
  return state_;
}

bool ControlAuthorityTracker::payload_is_valid(
  const ControlAuthoritySnapshot & snapshot) const noexcept
{
  if (snapshot.manager_epoch == 0U ||
    snapshot.estop_latched != (snapshot.authority == ControlAuthority::kEstop) ||
    snapshot.pending_autonomy_revocation_sequence >
    snapshot.transition_sequence)
  {
    return false;
  }
  if (
    snapshot.authority == ControlAuthority::kAutonomy &&
    (
      snapshot.pending_autonomy_revocation_sequence != 0U ||
      snapshot.autonomy_quiescence_acknowledged))
  {
    return false;
  }
  if (snapshot.authority == ControlAuthority::kHold) {
    return snapshot.active_source.empty();
  }
  return !snapshot.active_source.empty();
}

bool ControlAuthorityTracker::payload_matches(
  const ControlAuthoritySnapshot & snapshot) const noexcept
{
  return state_->authority == snapshot.authority &&
         state_->estop_latched == snapshot.estop_latched &&
         state_->manager_epoch == snapshot.manager_epoch &&
         state_->transition_sequence == snapshot.transition_sequence &&
         state_->pending_autonomy_revocation_sequence ==
         snapshot.pending_autonomy_revocation_sequence &&
         state_->autonomy_quiescence_acknowledged ==
         snapshot.autonomy_quiescence_acknowledged &&
         state_->active_source == snapshot.active_source &&
         state_->reason == snapshot.reason;
}

bool ControlAuthorityTracker::safe_manager_bootstrap(
  const ControlAuthoritySnapshot & snapshot) const noexcept
{
  return snapshot.manager_epoch != 0U &&
         snapshot.transition_sequence == 0U &&
         snapshot.authority == ControlAuthority::kHold &&
         !snapshot.estop_latched &&
         snapshot.pending_autonomy_revocation_sequence == 0U &&
         snapshot.active_source.empty() &&
         snapshot.reason == "initialized";
}

AuthorityUpdateResult ControlAuthorityTracker::accept(
  const ControlAuthoritySnapshot & snapshot,
  const TimePoint received_at,
  const AuthorityUpdateDecision decision,
  const bool generation_changed,
  const bool lease_discontinuity)
{
  state_ = snapshot;
  received_at_ = received_at;
  if (generation_changed) {
    ++generation_;
  }
  return {true, generation_changed, lease_discontinuity, decision};
}

std::string_view authority_update_reason(
  const AuthorityUpdateDecision decision) noexcept
{
  switch (decision) {
    case AuthorityUpdateDecision::kAcceptedInitial:
      return "accepted_initial";
    case AuthorityUpdateDecision::kAcceptedHeartbeat:
      return "accepted_heartbeat";
    case AuthorityUpdateDecision::kAcceptedTransition:
      return "accepted_transition";
    case AuthorityUpdateDecision::kAcceptedManagerRestart:
      return "accepted_manager_restart";
    case AuthorityUpdateDecision::kRejectedInvalidPayload:
      return "invalid_payload";
    case AuthorityUpdateDecision::kRejectedRetiredEpoch:
      return "retired_epoch";
    case AuthorityUpdateDecision::kRejectedUnsafeEpochBootstrap:
      return "unsafe_epoch_bootstrap";
    case AuthorityUpdateDecision::kRejectedLeaseDiscontinuity:
      return "lease_discontinuity";
    case AuthorityUpdateDecision::kRejectedOldSequence:
      return "old_sequence";
    case AuthorityUpdateDecision::kRejectedConflictingHeartbeat:
      return "conflicting_heartbeat";
    case AuthorityUpdateDecision::kRejectedReorderedReceipt:
      return "reordered_receipt";
  }
  return "unknown";
}

}  // namespace embodied_agent_cpp
