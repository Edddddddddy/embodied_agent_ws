#include "embodied_agent_cpp/action_scheduler.hpp"

#include <algorithm>
#include <utility>

#include <embodied_agent_interfaces/msg/robot_command_result.hpp>

namespace embodied_agent_cpp
{

using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;
using RobotCommandResult = embodied_agent_interfaces::msg::RobotCommandResult;

ActionScheduler::ActionScheduler(
  const std::size_t max_pending,
  const bool clear_queue_on_failure)
: max_pending_(std::max<std::size_t>(1, max_pending)),
  clear_queue_on_failure_(clear_queue_on_failure)
{
}

std::vector<SchedulerEvent> ActionScheduler::enqueue(const RobotCommand & command)
{
  std::vector<SchedulerEvent> events;
  if (command.command_id.empty()) {
    ++rejected_count_;
    SchedulerEvent rejected;
    rejected.kind = SchedulerEventKind::kRejectedInput;
    rejected.message = "missing_command_id";
    events.push_back(std::move(rejected));
    return events;
  }
  if (contains_id(command.command_id)) {
    ++rejected_count_;
    // 重复 ID 与原始请求不可区分，不能在 result topic 上发布同 ID 的失败，
    // 否则原始请求会误消费该终态。这里只通过日志/diagnostics 拒绝输入。
    SchedulerEvent rejected;
    rejected.kind = SchedulerEventKind::kRejectedInput;
    rejected.command_id = command.command_id;
    rejected.message = "duplicate_command_id";
    events.push_back(std::move(rejected));
    return events;
  }

  if (is_priority(command)) {
    ++accepted_count_;
    // STOP/CANCEL 属于控制面指令：丢弃尚未执行的普通动作，并取消当前 Action。
    // 等当前 Action 返回终态后再派发优先命令，保证同一时刻只有一个 goal 在执行。
    clear_pending(events, "queue_cleared_by_priority_command");
    if (!active_) {
      events.push_back(dispatch(command));
      return events;
    }
    pending_.push_front(command);
    if (!cancel_requested_) {
      cancel_requested_ = true;
      SchedulerEvent cancel;
      cancel.kind = SchedulerEventKind::kCancelActive;
      cancel.command_id = active_->command_id;
      events.push_back(std::move(cancel));
    }
    return events;
  }

  if (!active_) {
    ++accepted_count_;
    events.push_back(dispatch(command));
    return events;
  }
  if (pending_.size() >= max_pending_) {
    ++rejected_count_;
    events.push_back(result(
      command.command_id, false, RobotCommandResult::STATUS_REJECTED,
      "scheduler_queue_full"));
    return events;
  }

  ++accepted_count_;
  pending_.push_back(command);
  return events;
}

std::vector<SchedulerEvent> ActionScheduler::complete(
  const std::string & command_id,
  const bool success,
  const std::uint8_t status,
  const std::string & message)
{
  std::vector<SchedulerEvent> events;
  if (!active_ || active_->command_id != command_id) {
    // 取消 watchdog 可能先于迟到的 Action result 完成；旧结果必须忽略，不能推进新队列。
    return events;
  }

  const bool was_cancel_requested = cancel_requested_;
  active_.reset();
  cancel_requested_ = false;
  ++completed_count_;
  events.push_back(result(command_id, success, status, message));

  if (!success && clear_queue_on_failure_ && !was_cancel_requested) {
    clear_pending(events, "queue_cleared_after_action_failure");
    return events;
  }
  dispatch_next(events);
  return events;
}

std::vector<SchedulerEvent> ActionScheduler::clear_all(const std::string & reason)
{
  std::vector<SchedulerEvent> events;
  if (active_) {
    SchedulerEvent cancel;
    cancel.kind = SchedulerEventKind::kCancelActive;
    cancel.command_id = active_->command_id;
    events.push_back(std::move(cancel));
    events.push_back(result(
      active_->command_id, false, RobotCommandResult::STATUS_CANCELED, reason));
    active_.reset();
    cancel_requested_ = false;
    ++completed_count_;
  }
  clear_pending(events, reason);
  return events;
}

ActionSchedulerSnapshot ActionScheduler::snapshot() const
{
  ActionSchedulerSnapshot output;
  output.state = cancel_requested_ ? "canceling" : (active_ ? "executing" : "idle");
  if (active_) {
    output.active_command_id = active_->command_id;
    output.active_action_type = active_->action_type;
  }
  output.pending_count = pending_.size();
  output.max_pending = max_pending_;
  output.cancel_requested = cancel_requested_;
  output.accepted_count = accepted_count_;
  output.rejected_count = rejected_count_;
  output.completed_count = completed_count_;
  output.cleared_count = cleared_count_;
  return output;
}

bool ActionScheduler::is_priority(const RobotCommand & command) const
{
  return command.priority &&
         (command.action_type == RobotCommand::STOP ||
         command.action_type == RobotCommand::CANCEL_NAVIGATION);
}

bool ActionScheduler::contains_id(const std::string & command_id) const
{
  if (active_ && active_->command_id == command_id) {
    return true;
  }
  return std::any_of(
    pending_.begin(), pending_.end(),
    [&command_id](const RobotCommand & command) {
      return command.command_id == command_id;
    });
}

SchedulerEvent ActionScheduler::dispatch(RobotCommand command)
{
  active_ = command;
  SchedulerEvent event;
  event.kind = SchedulerEventKind::kDispatch;
  event.command_id = command.command_id;
  event.command = std::move(command);
  return event;
}

SchedulerEvent ActionScheduler::result(
  const std::string & command_id,
  const bool success,
  const std::uint8_t status,
  const std::string & message) const
{
  SchedulerEvent event;
  event.kind = SchedulerEventKind::kResult;
  event.command_id = command_id;
  event.success = success;
  event.status = status;
  event.message = message;
  return event;
}

void ActionScheduler::clear_pending(
  std::vector<SchedulerEvent> & events,
  const std::string & reason)
{
  while (!pending_.empty()) {
    const auto command_id = pending_.front().command_id;
    pending_.pop_front();
    ++cleared_count_;
    events.push_back(result(
      command_id, false, RobotCommandResult::STATUS_CANCELED, reason));
  }
}

void ActionScheduler::dispatch_next(std::vector<SchedulerEvent> & events)
{
  if (active_ || pending_.empty()) {
    return;
  }
  auto command = pending_.front();
  pending_.pop_front();
  events.push_back(dispatch(std::move(command)));
}

}  // namespace embodied_agent_cpp
