#include <gtest/gtest.h>

#include "embodied_simulation/command_behavior_tree.hpp"

namespace embodied_simulation
{
namespace
{

const char * kTreeXml = R"(
<root BTCPP_format="4" main_tree_to_execute="CommandExecution">
  <BehaviorTree ID="CommandExecution">
    <ReactiveSequence>
      <ValidateCommand/>
      <CheckSafety/>
      <ExecuteCommand/>
      <ConfirmResult/>
    </ReactiveSequence>
  </BehaviorTree>
</root>)";

embodied_agent_interfaces::msg::RobotCommand valid_move()
{
  embodied_agent_interfaces::msg::RobotCommand command;
  command.command_id = "bt-success";
  command.action_type = embodied_agent_interfaces::msg::RobotCommand::MOVE;
  command.linear_x = 0.15;
  command.duration_s = 1.0;
  return command;
}

TEST(CommandBehaviorTreeTest, ValidCommandRunsThenConfirmsSuccess)
{
  CommandBehaviorTree tree(kTreeXml);
  tree.start(valid_move());

  const auto running = tree.tick(
    false, ActionExecutionState::kRunning, "moving");
  EXPECT_EQ(running.outcome, CommandTreeOutcome::kRunning);
  EXPECT_EQ(running.stage, "execute");

  const auto succeeded = tree.tick(
    false, ActionExecutionState::kSucceeded, "succeeded");
  EXPECT_EQ(succeeded.outcome, CommandTreeOutcome::kSucceeded);
  EXPECT_EQ(succeeded.stage, "confirm");
}

TEST(CommandBehaviorTreeTest, InvalidCommandIsRejectedByValidation)
{
  CommandBehaviorTree tree(kTreeXml);
  auto command = valid_move();
  command.action_type = embodied_agent_interfaces::msg::RobotCommand::UNKNOWN;
  tree.start(command);

  const auto result = tree.tick(
    false, ActionExecutionState::kRunning, "pending");
  EXPECT_EQ(result.outcome, CommandTreeOutcome::kRejected);
  EXPECT_EQ(result.stage, "validate");
  EXPECT_EQ(result.detail, "invalid_command");
}

TEST(CommandBehaviorTreeTest, NewObstacleInterruptsRunningExecution)
{
  CommandBehaviorTree tree(kTreeXml);
  tree.start(valid_move());
  ASSERT_EQ(
    tree.tick(false, ActionExecutionState::kRunning, "moving").outcome,
    CommandTreeOutcome::kRunning);

  const auto result = tree.tick(
    true, ActionExecutionState::kBlocked, "lidar_emergency_stop");
  EXPECT_EQ(result.outcome, CommandTreeOutcome::kBlocked);
  EXPECT_EQ(result.stage, "safety");
  EXPECT_EQ(result.detail, "lidar_emergency_stop");
}

TEST(CommandBehaviorTreeTest, CancelAndTimeoutRemainDistinctTerminalResults)
{
  CommandBehaviorTree tree(kTreeXml);
  tree.start(valid_move());
  tree.tick(false, ActionExecutionState::kRunning, "moving");
  const auto canceled = tree.cancel("user_canceled");
  EXPECT_EQ(canceled.outcome, CommandTreeOutcome::kCanceled);
  EXPECT_EQ(canceled.detail, "user_canceled");

  tree.start(valid_move());
  const auto timed_out = tree.tick(
    false, ActionExecutionState::kTimedOut, "timed_out");
  EXPECT_EQ(timed_out.outcome, CommandTreeOutcome::kTimedOut);
  EXPECT_EQ(timed_out.stage, "execute");
}

TEST(CommandBehaviorTreeTest, ANewCommandRecoversAfterSafetyBlock)
{
  CommandBehaviorTree tree(kTreeXml);
  tree.start(valid_move());
  ASSERT_EQ(
    tree.tick(true, ActionExecutionState::kBlocked, "blocked").outcome,
    CommandTreeOutcome::kBlocked);

  tree.start(valid_move());
  EXPECT_EQ(
    tree.tick(false, ActionExecutionState::kSucceeded, "succeeded").outcome,
    CommandTreeOutcome::kSucceeded);
}

TEST(CommandBehaviorTreeTest, NavigationCommandsAreValidLongActions)
{
  CommandBehaviorTree tree(kTreeXml);
  embodied_agent_interfaces::msg::RobotCommand nav;
  nav.action_type = nav.NAVIGATE_TO;
  nav.target = "door";
  nav.duration_s = 3.0;

  tree.start(nav);
  auto result = tree.tick(false, ActionExecutionState::kRunning, "accepted");
  EXPECT_EQ(result.outcome, CommandTreeOutcome::kRunning);
  result = tree.tick(false, ActionExecutionState::kSucceeded, "succeeded");
  EXPECT_EQ(result.outcome, CommandTreeOutcome::kSucceeded);

  embodied_agent_interfaces::msg::RobotCommand patrol;
  patrol.action_type = patrol.FOLLOW_WAYPOINTS;
  patrol.waypoints = {"door", "desk", "home"};
  patrol.number_of_loops = 1;
  patrol.duration_s = 6.0;
  tree.start(patrol);
  result = tree.tick(false, ActionExecutionState::kRunning, "accepted");
  EXPECT_EQ(result.outcome, CommandTreeOutcome::kRunning);
}

}  // namespace
}  // namespace embodied_simulation
