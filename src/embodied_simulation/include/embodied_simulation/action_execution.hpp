#pragma once

namespace embodied_simulation
{

enum class ActionExecutionState
{
  kRunning,
  kSucceeded,
  kCanceled,
  kBlocked,
  kTimedOut,
};

struct ActionExecutionUpdate
{
  ActionExecutionState state{ActionExecutionState::kRunning};
  double progress{0.0};
};

class ActionExecution
{
public:
  ActionExecution(double duration_s, double timeout_s, double started_at_s);
  ActionExecutionUpdate update(
    double now_s, bool cancel_requested = false, bool blocked = false) const;

private:
  double duration_s_;
  double timeout_s_;
  double started_at_s_;
};

}  // namespace embodied_simulation
