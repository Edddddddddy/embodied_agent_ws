#pragma once

#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

#include <embodied_agent_interfaces/msg/robot_command.hpp>

namespace embodied_agent_cpp
{

struct GuardedCommandDrain
{
  std::vector<embodied_agent_interfaces::msg::RobotCommand> ready;
  std::vector<std::string> expired_command_ids;
  std::vector<std::string> invalidated_command_ids;
  std::vector<std::string> superseded_stop_command_ids;
};

// DDS reliable QoS 只保证“已匹配端点”之间可靠，并不会回放 discovery 前发布的动作。
// 该 outbox 在进程启动窗口短暂保存已通过 Guard 的命令，同时用容量、TTL 和
// authority generation 防止切权或 manager 失联前的动作迟到执行。
class GuardedCommandOutbox
{
public:
  explicit GuardedCommandOutbox(std::size_t max_pending = 32);

  bool enqueue(
    const embodied_agent_interfaces::msg::RobotCommand & command,
    double now_s,
    std::uint64_t authority_generation);
  GuardedCommandDrain drain(
    bool downstream_ready,
    double now_s,
    double ttl_s,
    std::optional<std::uint64_t> allowed_authority_generation);
  void clear();
  std::size_t size() const {return pending_.size();}

private:
  struct Entry
  {
    embodied_agent_interfaces::msg::RobotCommand command;
    double enqueued_at_s{0.0};
    std::uint64_t authority_generation{0};
  };

  std::size_t max_pending_;
  std::deque<Entry> pending_;
  std::vector<std::string> preempted_command_ids_;
  std::vector<std::string> superseded_stop_command_ids_;
};

}  // namespace embodied_agent_cpp
