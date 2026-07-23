#pragma once

#include <cstdint>
#include <string_view>

#include "embodied_agent_cpp/control_authority.hpp"

namespace embodied_agent_cpp
{

enum class ActionAuthorityDecision : std::uint8_t
{
  kAllow = 0,
  kStateUnavailable = 1,
  kHold = 2,
  kKeyboard = 3,
  kEmergencyStop = 4,
};

/// 决定一个已通过数值校验的动作是否拥有进入执行队列的权限。
///
/// priority STOP 是失效安全路径：即使控制权管理器尚未就绪或处于急停，
/// 也必须允许停止意图继续向下游传播；其他动作只有 AUTONOMY 状态可执行。
ActionAuthorityDecision evaluate_action_authority(
  bool gate_enabled,
  bool state_observed,
  ControlAuthority authority,
  bool priority_stop) noexcept;

std::string_view action_authority_reason(
  ActionAuthorityDecision decision) noexcept;

}  // namespace embodied_agent_cpp
