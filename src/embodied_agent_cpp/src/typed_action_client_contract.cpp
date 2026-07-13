#include "embodied_agent_cpp/typed_action_client_contract.hpp"

namespace embodied_agent_cpp
{

const char * outcome_name(ActionTerminalOutcome outcome)
{
  switch (outcome) {
    case ActionTerminalOutcome::kSucceeded: return "succeeded";
    case ActionTerminalOutcome::kCanceled: return "canceled";
    case ActionTerminalOutcome::kTimedOut: return "timed_out";
    case ActionTerminalOutcome::kRejected: return "rejected";
    case ActionTerminalOutcome::kBlocked: return "blocked";
    case ActionTerminalOutcome::kGoalRejected: return "goal_rejected";
    case ActionTerminalOutcome::kServerUnavailable: return "server_unavailable";
    case ActionTerminalOutcome::kClientTimedOut: return "client_timed_out";
    case ActionTerminalOutcome::kFailed: return "failed";
  }
  return "failed";
}

ActionTerminalOutcome outcome_from_action_status(
  bool success, unsigned int action_status)
{
  // ExecuteRobotCommand.action 中 1..5 是公开稳定的业务错误码；传输层即使返回
  // ABORTED，也要保留 timed_out/blocked 等更具体的服务端语义。
  switch (action_status) {
    case 1: return success ? ActionTerminalOutcome::kSucceeded : ActionTerminalOutcome::kFailed;
    case 2: return ActionTerminalOutcome::kRejected;
    case 3: return ActionTerminalOutcome::kCanceled;
    case 4: return ActionTerminalOutcome::kTimedOut;
    case 5: return ActionTerminalOutcome::kBlocked;
    default: return ActionTerminalOutcome::kFailed;
  }
}

bool outcome_matches_expectation(
  ActionTerminalOutcome outcome, const std::string & expected)
{
  return expected == outcome_name(outcome);
}

}  // namespace embodied_agent_cpp
