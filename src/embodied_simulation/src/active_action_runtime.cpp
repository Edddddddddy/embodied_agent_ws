#include "embodied_simulation/active_action_runtime.hpp"

#include <algorithm>
#include <utility>

namespace embodied_simulation
{

ActiveActionRuntime::ActiveActionRuntime(CommandBehaviorTree * behavior_tree)
: behavior_tree_(behavior_tree)
{
}

void ActiveActionRuntime::start(
  std::string action_name, double duration_s, double timeout_s,
  double started_at_s, bool uses_external_result)
{
  action_name_ = std::move(action_name);
  uses_external_result_ = uses_external_result;
  execution_.emplace(duration_s, timeout_s, started_at_s);
}

ActiveActionDecision ActiveActionRuntime::update(const ActiveActionInput & input) const
{
  if (!execution_) {
    return {
      ActionExecutionState::kBlocked, 0.0, "action_runtime_not_started", std::nullopt};
  }

  const auto timed = execution_->update(
    input.now_s, input.cancel_requested, input.safety_stopped);
  auto effective = timed;
  if (uses_external_result_ &&
    timed.state == ActionExecutionState::kRunning && input.external_update)
  {
    effective = *input.external_update;
    effective.progress = std::max(effective.progress, timed.progress);
  }

  // Nav2 这类外部 action 的失败原因不能压平成 blocked；外部 detail 优先透传，
  // 但本地 hard timeout 仍拥有最高优先级，保证失联的 Action Server 最终可收敛。
  const bool local_hard_timeout =
    timed.state == ActionExecutionState::kTimedOut;
  const std::string detail =
    local_hard_timeout ? "timed_out" :
    !input.external_detail.empty() ? input.external_detail :
    effective.state == ActionExecutionState::kSucceeded ? "succeeded" :
    effective.state == ActionExecutionState::kCanceled ? "canceled" :
    input.safety_reason;

  if (!behavior_tree_) {
    return {effective.state, effective.progress, detail, std::nullopt};
  }

  const auto tree_result = behavior_tree_->tick(
    input.safety_stopped, effective.state, detail);
  if (tree_result.outcome == CommandTreeOutcome::kRunning) {
    return {
      ActionExecutionState::kRunning, effective.progress,
      tree_result.stage + ":" + detail, tree_result};
  }
  return {
    tree_state(tree_result.outcome), effective.progress, tree_result.detail, tree_result};
}

std::optional<CommandTreeResult> ActiveActionRuntime::cancel_tree(
  const std::string & reason)
{
  if (!behavior_tree_ || !active()) {
    return std::nullopt;
  }
  return behavior_tree_->cancel(reason);
}

void ActiveActionRuntime::reset()
{
  execution_.reset();
  action_name_.clear();
  uses_external_result_ = false;
}

ActionExecutionState ActiveActionRuntime::tree_state(CommandTreeOutcome outcome)
{
  switch (outcome) {
    case CommandTreeOutcome::kSucceeded:
      return ActionExecutionState::kSucceeded;
    case CommandTreeOutcome::kCanceled:
      return ActionExecutionState::kCanceled;
    case CommandTreeOutcome::kTimedOut:
      return ActionExecutionState::kTimedOut;
    case CommandTreeOutcome::kRunning:
      return ActionExecutionState::kRunning;
    case CommandTreeOutcome::kRejected:
    case CommandTreeOutcome::kBlocked:
    case CommandTreeOutcome::kFailed:
      return ActionExecutionState::kBlocked;
  }
  return ActionExecutionState::kBlocked;
}

}  // namespace embodied_simulation
