#include "gtest/gtest.h"

#include "embodied_simulation/active_action_runtime.hpp"

namespace
{

using embodied_simulation::ActionExecutionState;
using embodied_simulation::ActionExecutionUpdate;
using embodied_simulation::ActiveActionInput;
using embodied_simulation::ActiveActionRuntime;

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
  command.command_id = "runtime-bt";
  command.action_type = command.MOVE;
  command.linear_x = 0.15;
  command.duration_s = 1.0;
  return command;
}

TEST(ActiveActionRuntimeTest, TimedActionProgressesAndFinishes)
{
  ActiveActionRuntime runtime;
  runtime.start("move", 2.0, 5.0, 10.0, false);

  const auto running = runtime.update({11.0, false, false, "moving", {}, ""});
  EXPECT_EQ(running.state, ActionExecutionState::kRunning);
  EXPECT_DOUBLE_EQ(running.progress, 0.5);
  EXPECT_EQ(running.detail, "moving");

  const auto finished = runtime.update({12.0, false, false, "moving", {}, ""});
  EXPECT_EQ(finished.state, ActionExecutionState::kSucceeded);
  EXPECT_EQ(finished.detail, "succeeded");
}

TEST(ActiveActionRuntimeTest, ExternalResultWinsWithoutLosingElapsedProgress)
{
  ActiveActionRuntime runtime;
  runtime.start("navigate_to", 11.0, 10.0, 0.0, true);
  ActiveActionInput input;
  input.now_s = 2.0;
  input.external_update = ActionExecutionUpdate{ActionExecutionState::kSucceeded, 0.1};
  input.external_detail = "nav2_goal_succeeded";

  const auto decision = runtime.update(input);

  EXPECT_EQ(decision.state, ActionExecutionState::kSucceeded);
  EXPECT_NEAR(decision.progress, 2.0 / 11.0, 1e-9);
  EXPECT_EQ(decision.detail, "nav2_goal_succeeded");
}

TEST(ActiveActionRuntimeTest, CancellationAndTimeoutHaveStableDetails)
{
  ActiveActionRuntime canceled;
  canceled.start("turn", 3.0, 5.0, 0.0, false);
  EXPECT_EQ(
    canceled.update({1.0, true, false, "turning", {}, ""}).detail,
    "canceled");

  ActiveActionRuntime timed_out;
  timed_out.start("move", 5.0, 1.0, 0.0, false);
  const auto decision = timed_out.update({1.0, false, false, "moving", {}, ""});
  EXPECT_EQ(decision.state, ActionExecutionState::kTimedOut);
  EXPECT_EQ(decision.detail, "timed_out");
}

TEST(ActiveActionRuntimeTest, ResetClearsAllPerGoalState)
{
  ActiveActionRuntime runtime;
  runtime.start("navigate_to", 10.0, 9.0, 1.0, true);
  runtime.reset();

  EXPECT_FALSE(runtime.active());
  EXPECT_FALSE(runtime.uses_external_result());
  EXPECT_TRUE(runtime.action_name().empty());
}

TEST(ActiveActionRuntimeTest, BehaviorTreeOutcomeIsMappedIntoActionDecision)
{
  embodied_simulation::CommandBehaviorTree tree(kTreeXml);
  tree.start(valid_move());
  ActiveActionRuntime runtime(&tree);
  runtime.start("move", 1.0, 3.0, 0.0, false);

  const auto running = runtime.update({0.5, false, false, "moving", {}, ""});
  ASSERT_TRUE(running.tree_result.has_value());
  EXPECT_EQ(running.state, ActionExecutionState::kRunning);
  EXPECT_EQ(running.detail, "execute:moving");

  const auto succeeded = runtime.update({1.0, false, false, "moving", {}, ""});
  ASSERT_TRUE(succeeded.tree_result.has_value());
  EXPECT_EQ(succeeded.state, ActionExecutionState::kSucceeded);
  EXPECT_EQ(succeeded.tree_result->stage, "confirm");
}

TEST(ActiveActionRuntimeTest, SafetyStopIsNormalizedByBehaviorTree)
{
  embodied_simulation::CommandBehaviorTree tree(kTreeXml);
  tree.start(valid_move());
  ActiveActionRuntime runtime(&tree);
  runtime.start("move", 2.0, 3.0, 0.0, false);

  const auto decision = runtime.update(
    {0.2, false, true, "lidar_emergency_stop", {}, ""});

  EXPECT_EQ(decision.state, ActionExecutionState::kBlocked);
  ASSERT_TRUE(decision.tree_result.has_value());
  EXPECT_EQ(decision.tree_result->stage, "safety");
  EXPECT_EQ(decision.detail, "lidar_emergency_stop");
}

}  // namespace
