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
