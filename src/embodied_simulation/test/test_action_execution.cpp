#include "gtest/gtest.h"

#include "embodied_simulation/action_execution.hpp"

TEST(ActionExecutionTest, ReportsProgressThenSucceedsAtRequestedDuration)
{
  const embodied_simulation::ActionExecution execution(2.0, 5.0, 10.0);

  const auto halfway = execution.update(11.0);
  EXPECT_EQ(halfway.state, embodied_simulation::ActionExecutionState::kRunning);
  EXPECT_DOUBLE_EQ(halfway.progress, 0.5);

  const auto finished = execution.update(12.0);
  EXPECT_EQ(finished.state, embodied_simulation::ActionExecutionState::kSucceeded);
  EXPECT_DOUBLE_EQ(finished.progress, 1.0);
}

TEST(ActionExecutionTest, CancellationWinsAndPreservesObservedProgress)
{
  const embodied_simulation::ActionExecution execution(4.0, 8.0, 20.0);
  const auto canceled = execution.update(21.0, true, false);

  EXPECT_EQ(canceled.state, embodied_simulation::ActionExecutionState::kCanceled);
  EXPECT_DOUBLE_EQ(canceled.progress, 0.25);
}

TEST(ActionExecutionTest, SafetyBlockTerminatesBeforeRequestedDuration)
{
  const embodied_simulation::ActionExecution execution(3.0, 6.0, 5.0);
  const auto blocked = execution.update(5.6, false, true);

  EXPECT_EQ(blocked.state, embodied_simulation::ActionExecutionState::kBlocked);
  EXPECT_NEAR(blocked.progress, 0.2, 1e-9);
}

TEST(ActionExecutionTest, HardTimeoutCanBeShorterThanRequestedDuration)
{
  const embodied_simulation::ActionExecution execution(5.0, 1.0, 100.0);
  const auto timed_out = execution.update(101.0);

  EXPECT_EQ(timed_out.state, embodied_simulation::ActionExecutionState::kTimedOut);
  EXPECT_DOUBLE_EQ(timed_out.progress, 0.2);
}
