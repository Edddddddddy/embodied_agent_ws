#include "embodied_simulation/command_behavior_tree.hpp"

#include <cmath>
#include <memory>
#include <utility>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>

namespace embodied_simulation
{
namespace
{

constexpr char kCommandKey[] = "command";
constexpr char kSafetyBlockedKey[] = "safety_blocked";
constexpr char kExecutionStateKey[] = "execution_state";
constexpr char kOutcomeKey[] = "outcome";
constexpr char kStageKey[] = "stage";
constexpr char kDetailKey[] = "detail";

using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

bool valid_command(const RobotCommand & command)
{
  if (command.action_type == RobotCommand::STOP) {
    return true;
  }
  if (command.action_type == RobotCommand::SET_MODE) {
    return command.mode == "manual" ||
           command.mode == "obstacle_avoidance" ||
           command.mode == "wall_following";
  }
  if (command.action_type == RobotCommand::WAVE) {
    return command.count >= 1 && command.count <= 5;
  }
  if (command.action_type == RobotCommand::SET_LED) {
    return command.color == "off" ||
           command.color == "red" ||
           command.color == "green" ||
           command.color == "blue" ||
           command.color == "yellow" ||
           command.color == "white";
  }
  if (command.action_type == RobotCommand::MOVE) {
    return std::isfinite(command.linear_x) &&
           std::isfinite(command.angular_z) &&
           std::isfinite(command.duration_s) &&
           command.duration_s >= 0.0 && command.duration_s <= 10.0;
  }
  if (command.action_type == RobotCommand::TURN) {
    return std::isfinite(command.angular_z) &&
           std::isfinite(command.duration_s) &&
           command.duration_s >= 0.0 && command.duration_s <= 10.0;
  }
  return false;
}

class ValidateCommandNode : public BT::ConditionNode
{
public:
  ValidateCommandNode(const std::string & name, const BT::NodeConfig & config)
  : ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    const auto command = config().blackboard->get<RobotCommand>(kCommandKey);
    if (!valid_command(command)) {
      config().blackboard->set(kOutcomeKey, CommandTreeOutcome::kRejected);
      config().blackboard->set(kStageKey, std::string("validate"));
      config().blackboard->set(kDetailKey, std::string("invalid_command"));
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

class CheckSafetyNode : public BT::ConditionNode
{
public:
  CheckSafetyNode(const std::string & name, const BT::NodeConfig & config)
  : ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    if (config().blackboard->get<bool>(kSafetyBlockedKey)) {
      config().blackboard->set(kOutcomeKey, CommandTreeOutcome::kBlocked);
      config().blackboard->set(kStageKey, std::string("safety"));
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

class ExecuteCommandNode : public BT::StatefulActionNode
{
public:
  ExecuteCommandNode(const std::string & name, const BT::NodeConfig & config)
  : StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus onStart() override {return evaluate();}
  BT::NodeStatus onRunning() override {return evaluate();}
  void onHalted() override {}

private:
  BT::NodeStatus evaluate()
  {
    const auto state = config().blackboard->get<ActionExecutionState>(
      kExecutionStateKey);
    config().blackboard->set(kStageKey, std::string("execute"));
    if (state == ActionExecutionState::kRunning) {
      config().blackboard->set(kOutcomeKey, CommandTreeOutcome::kRunning);
      return BT::NodeStatus::RUNNING;
    }
    if (state == ActionExecutionState::kSucceeded) {
      return BT::NodeStatus::SUCCESS;
    }
    if (state == ActionExecutionState::kCanceled) {
      config().blackboard->set(kOutcomeKey, CommandTreeOutcome::kCanceled);
    } else if (state == ActionExecutionState::kTimedOut) {
      config().blackboard->set(kOutcomeKey, CommandTreeOutcome::kTimedOut);
    } else if (state == ActionExecutionState::kBlocked) {
      config().blackboard->set(kOutcomeKey, CommandTreeOutcome::kBlocked);
    } else {
      config().blackboard->set(kOutcomeKey, CommandTreeOutcome::kFailed);
    }
    return BT::NodeStatus::FAILURE;
  }
};

class ConfirmResultNode : public BT::ConditionNode
{
public:
  ConfirmResultNode(const std::string & name, const BT::NodeConfig & config)
  : ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {return {};}

  BT::NodeStatus tick() override
  {
    config().blackboard->set(kStageKey, std::string("confirm"));
    const auto state = config().blackboard->get<ActionExecutionState>(
      kExecutionStateKey);
    if (state != ActionExecutionState::kSucceeded) {
      config().blackboard->set(kOutcomeKey, CommandTreeOutcome::kFailed);
      return BT::NodeStatus::FAILURE;
    }
    config().blackboard->set(kOutcomeKey, CommandTreeOutcome::kSucceeded);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace

class CommandBehaviorTree::Impl
{
public:
  explicit Impl(std::string xml_text)
  : xml_text_(std::move(xml_text))
  {
    factory_.registerNodeType<ValidateCommandNode>("ValidateCommand");
    factory_.registerNodeType<CheckSafetyNode>("CheckSafety");
    factory_.registerNodeType<ExecuteCommandNode>("ExecuteCommand");
    factory_.registerNodeType<ConfirmResultNode>("ConfirmResult");
  }

  void start(const RobotCommand & command)
  {
    blackboard_ = BT::Blackboard::create();
    blackboard_->set(kCommandKey, command);
    blackboard_->set(kSafetyBlockedKey, false);
    blackboard_->set(kExecutionStateKey, ActionExecutionState::kRunning);
    blackboard_->set(kOutcomeKey, CommandTreeOutcome::kRunning);
    blackboard_->set(kStageKey, std::string("validate"));
    blackboard_->set(kDetailKey, std::string());
    tree_ = std::make_unique<BT::Tree>(
      factory_.createTreeFromText(xml_text_, blackboard_));
  }

  CommandTreeResult tick(
    bool safety_blocked,
    ActionExecutionState execution_state,
    const std::string & detail)
  {
    if (!tree_) {
      return {CommandTreeOutcome::kFailed, "idle", "tree_not_started"};
    }
    blackboard_->set(kSafetyBlockedKey, safety_blocked);
    blackboard_->set(kExecutionStateKey, execution_state);
    blackboard_->set(kDetailKey, detail);
    // 每个控制周期只 tick 一次，让“安全检查、执行状态、结果确认”保持可观测。
    // 这样比在一个回调里写死 if/else 更容易替换成复杂 BT XML。
    const auto status = tree_->tickOnce();
    auto outcome = blackboard_->get<CommandTreeOutcome>(kOutcomeKey);
    if (status == BT::NodeStatus::FAILURE && outcome == CommandTreeOutcome::kRunning) {
      outcome = CommandTreeOutcome::kFailed;
    }
    return {
      outcome,
      blackboard_->get<std::string>(kStageKey),
      blackboard_->get<std::string>(kDetailKey)};
  }

  CommandTreeResult cancel(const std::string & detail)
  {
    if (tree_) {
      tree_->haltTree();
    }
    if (blackboard_) {
      blackboard_->set(kOutcomeKey, CommandTreeOutcome::kCanceled);
      blackboard_->set(kStageKey, std::string("execute"));
      blackboard_->set(kDetailKey, detail);
    }
    return {CommandTreeOutcome::kCanceled, "execute", detail};
  }

private:
  std::string xml_text_;
  BT::BehaviorTreeFactory factory_;
  BT::Blackboard::Ptr blackboard_;
  std::unique_ptr<BT::Tree> tree_;
};

CommandBehaviorTree::CommandBehaviorTree(const std::string & xml_text)
: impl_(std::make_unique<Impl>(xml_text)) {}

CommandBehaviorTree::~CommandBehaviorTree() = default;

void CommandBehaviorTree::start(const RobotCommand & command)
{
  impl_->start(command);
}

CommandTreeResult CommandBehaviorTree::tick(
  bool safety_blocked,
  ActionExecutionState execution_state,
  const std::string & detail)
{
  return impl_->tick(safety_blocked, execution_state, detail);
}

CommandTreeResult CommandBehaviorTree::cancel(const std::string & detail)
{
  return impl_->cancel(detail);
}

}  // namespace embodied_simulation
