#include "embodied_simulation/action_execution.hpp"

#include <algorithm>

namespace embodied_simulation
{

ActionExecution::ActionExecution(
  double duration_s, double timeout_s, double started_at_s)
: duration_s_(std::max(0.0, duration_s)),
  timeout_s_(std::max(0.0, timeout_s)),
  started_at_s_(started_at_s)
{
}

ActionExecutionUpdate ActionExecution::update(
  double now_s, bool cancel_requested, bool blocked) const
{
  const double elapsed = std::max(0.0, now_s - started_at_s_);
  const double progress = duration_s_ <= 0.0 ? 1.0 :
    std::clamp(elapsed / duration_s_, 0.0, 1.0);

  if (cancel_requested) {
    return {ActionExecutionState::kCanceled, progress};
  }
  if (blocked) {
    return {ActionExecutionState::kBlocked, progress};
  }
  if (elapsed >= timeout_s_ && timeout_s_ < duration_s_) {
    return {ActionExecutionState::kTimedOut, progress};
  }
  if (elapsed >= duration_s_) {
    return {ActionExecutionState::kSucceeded, 1.0};
  }
  return {ActionExecutionState::kRunning, progress};
}

}  // namespace embodied_simulation
