#include "gtest/gtest.h"

#include "embodied_agent_cpp/typed_action_client_contract.hpp"

using embodied_agent_cpp::ActionTerminalOutcome;
using embodied_agent_cpp::outcome_from_action_status;
using embodied_agent_cpp::outcome_matches_expectation;

TEST(TypedActionClientContractTest, PreservesServerBusinessOutcome)
{
  EXPECT_EQ(outcome_from_action_status(true, 1), ActionTerminalOutcome::kSucceeded);
  EXPECT_EQ(outcome_from_action_status(false, 3), ActionTerminalOutcome::kCanceled);
  EXPECT_EQ(outcome_from_action_status(false, 4), ActionTerminalOutcome::kTimedOut);
  EXPECT_EQ(outcome_from_action_status(false, 5), ActionTerminalOutcome::kBlocked);
}

TEST(TypedActionClientContractTest, DoesNotTreatInconsistentSuccessAsSucceeded)
{
  EXPECT_EQ(outcome_from_action_status(false, 1), ActionTerminalOutcome::kFailed);
  EXPECT_EQ(outcome_from_action_status(true, 0), ActionTerminalOutcome::kFailed);
}

TEST(TypedActionClientContractTest, MatchesExactMachineReadableExpectation)
{
  EXPECT_TRUE(outcome_matches_expectation(ActionTerminalOutcome::kTimedOut, "timed_out"));
  EXPECT_FALSE(outcome_matches_expectation(ActionTerminalOutcome::kTimedOut, "succeeded"));
}

TEST(TypedActionClientContractTest, KeepsAvailabilityAndWaitTimeoutDistinct)
{
  EXPECT_STREQ(
    embodied_agent_cpp::outcome_name(ActionTerminalOutcome::kServerUnavailable),
    "server_unavailable");
  EXPECT_STREQ(
    embodied_agent_cpp::outcome_name(ActionTerminalOutcome::kClientTimedOut),
    "client_timed_out");
}
