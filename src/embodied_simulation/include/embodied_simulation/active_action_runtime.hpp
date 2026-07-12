#pragma once

#include <optional>
#include <string>

#include "embodied_simulation/action_execution.hpp"
#include "embodied_simulation/command_behavior_tree.hpp"

namespace embodied_simulation
{

struct ActiveActionInput
{
  double now_s{0.0};
  bool cancel_requested{false};
  bool safety_stopped{false};
  std::string safety_reason;
  std::optional<ActionExecutionUpdate> external_update;
  std::string external_detail;
};

struct ActiveActionDecision
{
  ActionExecutionState state{ActionExecutionState::kRunning};
  double progress{0.0};
  std::string detail;
  std::optional<CommandTreeResult> tree_result;

  bool terminal() const {return state != ActionExecutionState::kRunning;}
};

class ActiveActionRuntime
{
public:
  explicit ActiveActionRuntime(CommandBehaviorTree * behavior_tree = nullptr);

  void start(
    std::string action_name, double duration_s, double timeout_s,
    double started_at_s, bool uses_external_result);
  ActiveActionDecision update(const ActiveActionInput & input) const;
  std::optional<CommandTreeResult> cancel_tree(const std::string & reason);
  void reset();

  bool active() const {return execution_.has_value();}
  bool uses_external_result() const {return uses_external_result_;}
  const std::string & action_name() const {return action_name_;}

private:
  static ActionExecutionState tree_state(CommandTreeOutcome outcome);

  CommandBehaviorTree * behavior_tree_{nullptr};
  std::optional<ActionExecution> execution_;
  std::string action_name_;
  bool uses_external_result_{false};
};

}  // namespace embodied_simulation
