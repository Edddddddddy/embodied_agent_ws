#pragma once

#include <memory>
#include <string>

#include <embodied_agent_interfaces/msg/robot_command.hpp>

#include "embodied_simulation/action_execution.hpp"

namespace embodied_simulation
{

enum class CommandTreeOutcome
{
  kRunning,
  kSucceeded,
  kRejected,
  kCanceled,
  kTimedOut,
  kBlocked,
  kFailed,
};

struct CommandTreeResult
{
  CommandTreeOutcome outcome{CommandTreeOutcome::kRunning};
  std::string stage{"idle"};
  std::string detail;
};

class CommandBehaviorTree
{
public:
  // 外部只需要 start/tick/cancel；XML 注册、blackboard 和节点 halt 细节隐藏在 Impl 中。
  // 每次 tick 都重新注入 safety 状态，使运行中的动作也能被新障碍立即打断。
  explicit CommandBehaviorTree(const std::string & xml_text);
  ~CommandBehaviorTree();

  CommandBehaviorTree(const CommandBehaviorTree &) = delete;
  CommandBehaviorTree & operator=(const CommandBehaviorTree &) = delete;

  void start(const embodied_agent_interfaces::msg::RobotCommand & command);
  CommandTreeResult tick(
    bool safety_blocked,
    ActionExecutionState execution_state,
    const std::string & detail);
  CommandTreeResult cancel(const std::string & detail = "canceled");

private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace embodied_simulation
