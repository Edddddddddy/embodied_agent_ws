#include "embodied_agent_cpp/action_authority.hpp"

namespace embodied_agent_cpp
{

ActionAuthorityDecision evaluate_action_authority(
  const bool gate_enabled,
  const bool state_observed,
  const ControlAuthority authority,
  const bool priority_stop) noexcept
{
  if (!gate_enabled || priority_stop) {
    return ActionAuthorityDecision::kAllow;
  }
  if (!state_observed) {
    return ActionAuthorityDecision::kStateUnavailable;
  }
  switch (authority) {
    case ControlAuthority::kAutonomy:
      return ActionAuthorityDecision::kAllow;
    case ControlAuthority::kHold:
      return ActionAuthorityDecision::kHold;
    case ControlAuthority::kKeyboard:
      return ActionAuthorityDecision::kKeyboard;
    case ControlAuthority::kEstop:
      return ActionAuthorityDecision::kEmergencyStop;
  }
  return ActionAuthorityDecision::kStateUnavailable;
}

std::string_view action_authority_reason(
  const ActionAuthorityDecision decision) noexcept
{
  switch (decision) {
    case ActionAuthorityDecision::kAllow:
      return "allowed";
    case ActionAuthorityDecision::kStateUnavailable:
      return "control_authority_state_unavailable";
    case ActionAuthorityDecision::kHold:
      return "control_authority_hold";
    case ActionAuthorityDecision::kKeyboard:
      return "control_authority_keyboard_active";
    case ActionAuthorityDecision::kEmergencyStop:
      return "control_authority_emergency_stop";
  }
  return "control_authority_state_unavailable";
}

}  // namespace embodied_agent_cpp
