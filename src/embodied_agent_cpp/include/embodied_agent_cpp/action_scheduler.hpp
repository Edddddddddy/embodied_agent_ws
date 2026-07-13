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

enum class SchedulerEventKind
{
  kDispatch,
  kCancelActive,
  kResult,
  kRejectedInput,
};

struct SchedulerEvent
{
  SchedulerEventKind kind{SchedulerEventKind::kResult};
  embodied_agent_interfaces::msg::RobotCommand command;
  std::string command_id;
  bool success{false};
  std::uint8_t status{0};
  std::string message;
};

struct ActionSchedulerSnapshot
{
  std::string state{"idle"};
  std::string active_command_id;
  std::uint8_t active_action_type{0};
  std::size_t pending_count{0};
  std::size_t max_pending{0};
  bool cancel_requested{false};
  std::uint64_t accepted_count{0};
  std::uint64_t rejected_count{0};
  std::uint64_t completed_count{0};
  std::uint64_t cleared_count{0};
};

/// 受信 RobotCommand 的单执行槽调度器。
///
/// 接口只接收新命令和 Action 终态，内部统一维护 FIFO、优先取消、失败清队列和
/// command_id 关联。它不依赖 ROS executor 或 Action Client，因此全部竞态语义都能
/// 在毫秒级 C++ 单元测试中固定下来。
class ActionScheduler
{
public:
  explicit ActionScheduler(
    std::size_t max_pending = 16,
    bool clear_queue_on_failure = true);

  std::vector<SchedulerEvent> enqueue(
    const embodied_agent_interfaces::msg::RobotCommand & command);

  std::vector<SchedulerEvent> complete(
    const std::string & command_id,
    bool success,
    std::uint8_t status,
    const std::string & message);

  /// 终止活动命令并清空等待队列，用于 Lifecycle deactivate/cleanup。
  std::vector<SchedulerEvent> clear_all(const std::string & reason);

  ActionSchedulerSnapshot snapshot() const;

private:
  using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

  bool is_priority(const RobotCommand & command) const;
  bool contains_id(const std::string & command_id) const;
  SchedulerEvent dispatch(RobotCommand command);
  SchedulerEvent result(
    const std::string & command_id,
    bool success,
    std::uint8_t status,
    const std::string & message) const;
  void clear_pending(
    std::vector<SchedulerEvent> & events,
    const std::string & reason);
  void dispatch_next(std::vector<SchedulerEvent> & events);

  std::size_t max_pending_;
  bool clear_queue_on_failure_;
  std::optional<RobotCommand> active_;
  std::deque<RobotCommand> pending_;
  bool cancel_requested_{false};
  std::uint64_t accepted_count_{0};
  std::uint64_t rejected_count_{0};
  std::uint64_t completed_count_{0};
  std::uint64_t cleared_count_{0};
};

}  // namespace embodied_agent_cpp
