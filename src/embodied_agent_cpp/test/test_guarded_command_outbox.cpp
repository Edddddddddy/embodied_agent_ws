#include <gtest/gtest.h>

#include <embodied_agent_interfaces/msg/robot_command.hpp>

#include "embodied_agent_cpp/guarded_command_outbox.hpp"

namespace
{

using embodied_agent_cpp::GuardedCommandOutbox;
using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

RobotCommand command(const std::string & id)
{
  RobotCommand output;
  output.command_id = id;
  output.action_type = RobotCommand::MOVE;
  return output;
}

RobotCommand priority_stop(const std::string & id)
{
  RobotCommand output;
  output.command_id = id;
  output.action_type = RobotCommand::STOP;
  output.priority = true;
  return output;
}

TEST(GuardedCommandOutboxTest, HoldsCommandsUntilDownstreamIsMatched)
{
  GuardedCommandOutbox outbox(3);
  ASSERT_TRUE(outbox.enqueue(command("one"), 1.0, 7U));
  ASSERT_TRUE(outbox.enqueue(command("two"), 1.1, 7U));

  EXPECT_TRUE(outbox.drain(false, 1.2, 3.0, 7U).ready.empty());
  const auto matched = outbox.drain(true, 1.3, 3.0, 7U);
  ASSERT_EQ(matched.ready.size(), 2U);
  EXPECT_EQ(matched.ready[0].command_id, "one");
  EXPECT_EQ(matched.ready[1].command_id, "two");
  EXPECT_EQ(outbox.size(), 0U);
}

TEST(GuardedCommandOutboxTest, CapacityAndTtlPreventLateUnsafeExecution)
{
  GuardedCommandOutbox outbox(1);
  ASSERT_TRUE(outbox.enqueue(command("old"), 1.0, 1U));
  EXPECT_FALSE(outbox.enqueue(command("overflow"), 1.1, 1U));

  const auto expired = outbox.drain(true, 4.0, 3.0, 1U);
  EXPECT_TRUE(expired.ready.empty());
  EXPECT_EQ(expired.expired_command_ids, std::vector<std::string>({"old"}));
}

TEST(GuardedCommandOutboxTest, AuthorityGenerationInvalidatesOnlyOrdinaryCommands)
{
  GuardedCommandOutbox outbox(3);
  ASSERT_TRUE(outbox.enqueue(command("old-motion"), 1.0, 4U));
  ASSERT_TRUE(outbox.enqueue(priority_stop("stop"), 1.1, 4U));

  const auto hold = outbox.drain(false, 1.2, 3.0, std::nullopt);
  EXPECT_EQ(
    hold.invalidated_command_ids,
    std::vector<std::string>({"old-motion"}));
  EXPECT_EQ(outbox.size(), 1U);

  const auto scheduler_ready = outbox.drain(
    true, 1.3, 3.0, std::nullopt);
  ASSERT_EQ(scheduler_ready.ready.size(), 1U);
  EXPECT_EQ(scheduler_ready.ready.front().command_id, "stop");
}

TEST(GuardedCommandOutboxTest, LeaseDiscontinuityRejectsOldGenerationAfterReauthorization)
{
  GuardedCommandOutbox outbox(2);
  ASSERT_TRUE(outbox.enqueue(command("before-gap"), 1.0, 2U));

  const auto reauthorized = outbox.drain(true, 1.1, 3.0, 3U);
  EXPECT_TRUE(reauthorized.ready.empty());
  EXPECT_EQ(
    reauthorized.invalidated_command_ids,
    std::vector<std::string>({"before-gap"}));
}

TEST(GuardedCommandOutboxTest, PriorityStopPreemptsAFullOrdinaryBuffer)
{
  GuardedCommandOutbox outbox(1);
  ASSERT_TRUE(outbox.enqueue(command("queued-motion"), 1.0, 8U));
  ASSERT_TRUE(outbox.enqueue(priority_stop("urgent-stop"), 1.1, 8U));

  const auto drain = outbox.drain(true, 1.2, 3.0, 8U);
  ASSERT_EQ(drain.ready.size(), 1U);
  EXPECT_EQ(drain.ready.front().command_id, "urgent-stop");
  EXPECT_EQ(drain.ready.front().action_type, RobotCommand::STOP);
  EXPECT_EQ(
    drain.invalidated_command_ids,
    std::vector<std::string>({"queued-motion"}));
}

TEST(GuardedCommandOutboxTest, NewPriorityStopReplacesOldExactIdentity)
{
  GuardedCommandOutbox outbox(1);
  ASSERT_TRUE(outbox.enqueue(priority_stop("old-stop"), 1.0, 8U));
  ASSERT_TRUE(outbox.enqueue(priority_stop("quiescence-stop"), 1.1, 9U));

  const auto drain = outbox.drain(true, 1.2, 3.0, std::nullopt);
  ASSERT_EQ(drain.ready.size(), 1U);
  EXPECT_EQ(drain.ready.front().command_id, "quiescence-stop");
  EXPECT_EQ(
    drain.superseded_stop_command_ids,
    std::vector<std::string>({"old-stop"}));
}

TEST(GuardedCommandOutboxTest, RetryOfSamePriorityStopRefreshesWithoutFalseAudit)
{
  GuardedCommandOutbox outbox(1);
  ASSERT_TRUE(outbox.enqueue(priority_stop("same-stop"), 1.0, 8U));
  ASSERT_TRUE(outbox.enqueue(priority_stop("same-stop"), 2.9, 9U));

  const auto drain = outbox.drain(true, 3.1, 3.0, std::nullopt);
  ASSERT_EQ(drain.ready.size(), 1U);
  EXPECT_EQ(drain.ready.front().command_id, "same-stop");
  EXPECT_TRUE(drain.superseded_stop_command_ids.empty());
  EXPECT_TRUE(drain.expired_command_ids.empty());
}

}  // namespace
