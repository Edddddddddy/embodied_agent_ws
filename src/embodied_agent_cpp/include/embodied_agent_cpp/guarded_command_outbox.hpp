#pragma once

#include <cstddef>
#include <deque>
#include <string>
#include <vector>

#include <embodied_agent_interfaces/msg/robot_command.hpp>

namespace embodied_agent_cpp
{

struct GuardedCommandDrain
{
  std::vector<embodied_agent_interfaces::msg::RobotCommand> ready;
  std::vector<std::string> expired_command_ids;
};

// DDS reliable QoS 只保证“已匹配端点”之间可靠，并不会回放 discovery 前发布的动作。
// 该 outbox 在进程启动窗口短暂保存已通过 Guard 的命令，同时用容量和 TTL 防止旧动作迟到执行。
class GuardedCommandOutbox
{
public:
  explicit GuardedCommandOutbox(std::size_t max_pending = 32);

  bool enqueue(
    const embodied_agent_interfaces::msg::RobotCommand & command,
    double now_s);
  GuardedCommandDrain drain(
    bool downstream_ready, double now_s, double ttl_s);
  void clear();
  std::size_t size() const {return pending_.size();}

private:
  struct Entry
  {
    embodied_agent_interfaces::msg::RobotCommand command;
    double enqueued_at_s{0.0};
  };

  std::size_t max_pending_;
  std::deque<Entry> pending_;
};

}  // namespace embodied_agent_cpp
