#include "embodied_agent_cpp/guarded_command_outbox.hpp"

#include <algorithm>
#include <utility>

namespace embodied_agent_cpp
{
namespace
{

bool is_priority_stop(
  const embodied_agent_interfaces::msg::RobotCommand & command) noexcept
{
  return command.priority &&
         command.action_type ==
         embodied_agent_interfaces::msg::RobotCommand::STOP;
}

}  // namespace

GuardedCommandOutbox::GuardedCommandOutbox(const std::size_t max_pending)
: max_pending_(std::max<std::size_t>(1, max_pending))
{
}

bool GuardedCommandOutbox::enqueue(
  const embodied_agent_interfaces::msg::RobotCommand & command,
  const double now_s,
  const std::uint64_t authority_generation)
{
  if (is_priority_stop(command)) {
    // STOP 必须越过 discovery 缓冲中的普通动作，而且 outbox 只保留“最新一条”
    // STOP。不能在满 STOP 队列时返回成功却静默丢掉新 command_id：静默屏障会
    // 等待该精确 ID 的 result，最终只能超时且无法审计是哪条 STOP 被覆盖。
    const auto removed_begin = std::remove_if(
      pending_.begin(), pending_.end(),
      [this, &command](const Entry & entry) {
        if (is_priority_stop(entry.command)) {
          if (entry.command.command_id != command.command_id) {
            superseded_stop_command_ids_.push_back(entry.command.command_id);
          }
          return true;
        }
        preempted_command_ids_.push_back(entry.command.command_id);
        return true;
      });
    pending_.erase(removed_begin, pending_.end());
    pending_.push_back({command, now_s, authority_generation});
    return true;
  }
  if (pending_.size() >= max_pending_) {
    return false;
  }
  pending_.push_back({command, now_s, authority_generation});
  return true;
}

GuardedCommandDrain GuardedCommandOutbox::drain(
  const bool downstream_ready,
  const double now_s,
  const double ttl_s,
  const std::optional<std::uint64_t> allowed_authority_generation)
{
  GuardedCommandDrain output;
  output.invalidated_command_ids.swap(preempted_command_ids_);
  output.superseded_stop_command_ids.swap(superseded_stop_command_ids_);
  const double safe_ttl = std::max(0.0, ttl_s);
  std::deque<Entry> retained;
  while (!pending_.empty()) {
    Entry entry = std::move(pending_.front());
    pending_.pop_front();
    if (now_s - entry.enqueued_at_s >= safe_ttl) {
      output.expired_command_ids.push_back(entry.command.command_id);
      continue;
    }
    // priority STOP 是失效安全边沿，不受普通动作 generation 约束。这样即使
    // manager 已切到 HOLD/ESTOP，等待 discovery 的 STOP 仍能到达 Scheduler。
    if (
      !is_priority_stop(entry.command) &&
      (
        !allowed_authority_generation ||
        entry.authority_generation != *allowed_authority_generation))
    {
      output.invalidated_command_ids.push_back(entry.command.command_id);
      continue;
    }
    retained.push_back(std::move(entry));
  }
  pending_ = std::move(retained);
  if (!downstream_ready) {
    return output;
  }
  while (!pending_.empty()) {
    output.ready.push_back(std::move(pending_.front().command));
    pending_.pop_front();
  }
  return output;
}

void GuardedCommandOutbox::clear()
{
  pending_.clear();
  preempted_command_ids_.clear();
  superseded_stop_command_ids_.clear();
}

}  // namespace embodied_agent_cpp
