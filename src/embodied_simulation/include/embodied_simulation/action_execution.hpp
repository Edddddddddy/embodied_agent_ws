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
  // 该值对象只描述时间/取消/阻塞语义，不产生 ROS 副作用，因此 BT 和 Action server
  // 可以共享完全相同的终态判定。优先级由实现固定，调用方无需重复判断。
  ActionExecution(double duration_s, double timeout_s, double started_at_s);
  ActionExecutionUpdate update(
    double now_s, bool cancel_requested = false, bool blocked = false) const;

private:
  double duration_s_;
  double timeout_s_;
  double started_at_s_;
};

}  // namespace embodied_simulation
