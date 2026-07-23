#pragma once

#include <string>

namespace embodied_agent_cpp
{

// 客户端只关心跨进程后可稳定观察的终态，避免把 rclcpp_action 的传输枚举
// 直接泄漏到验收脚本和上层业务代码中。
enum class ActionTerminalOutcome
{
  kSucceeded,
  kCanceled,
  kTimedOut,
  kRejected,
  kBlocked,
  kFailed,
  kGoalRejected,
  kServerUnavailable,
  kClientTimedOut,
};

const char * outcome_name(ActionTerminalOutcome outcome);
ActionTerminalOutcome outcome_from_action_status(
  bool success, unsigned int action_status);
bool outcome_matches_expectation(
  ActionTerminalOutcome outcome, const std::string & expected);

}  // namespace embodied_agent_cpp
