#include "embodied_agent_cpp/guarded_command_outbox.hpp"

#include <algorithm>

namespace embodied_agent_cpp
{

GuardedCommandOutbox::GuardedCommandOutbox(const std::size_t max_pending)
: max_pending_(std::max<std::size_t>(1, max_pending))
{
}

bool GuardedCommandOutbox::enqueue(
  const embodied_agent_interfaces::msg::RobotCommand & command,
  const double now_s)
{
  if (pending_.size() >= max_pending_) {
    return false;
  }
  pending_.push_back({command, now_s});
  return true;
}

GuardedCommandDrain GuardedCommandOutbox::drain(
  const bool downstream_ready, const double now_s, const double ttl_s)
{
  GuardedCommandDrain output;
  const double safe_ttl = std::max(0.0, ttl_s);
  while (!pending_.empty() && now_s - pending_.front().enqueued_at_s >= safe_ttl) {
    output.expired_command_ids.push_back(pending_.front().command.command_id);
    pending_.pop_front();
  }
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
}

}  // namespace embodied_agent_cpp
