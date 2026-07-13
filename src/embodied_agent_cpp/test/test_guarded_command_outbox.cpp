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

TEST(GuardedCommandOutboxTest, HoldsCommandsUntilDownstreamIsMatched)
{
  GuardedCommandOutbox outbox(3);
  ASSERT_TRUE(outbox.enqueue(command("one"), 1.0));
  ASSERT_TRUE(outbox.enqueue(command("two"), 1.1));

  EXPECT_TRUE(outbox.drain(false, 1.2, 3.0).ready.empty());
  const auto matched = outbox.drain(true, 1.3, 3.0);
  ASSERT_EQ(matched.ready.size(), 2U);
  EXPECT_EQ(matched.ready[0].command_id, "one");
  EXPECT_EQ(matched.ready[1].command_id, "two");
  EXPECT_EQ(outbox.size(), 0U);
}

TEST(GuardedCommandOutboxTest, CapacityAndTtlPreventLateUnsafeExecution)
{
  GuardedCommandOutbox outbox(1);
  ASSERT_TRUE(outbox.enqueue(command("old"), 1.0));
  EXPECT_FALSE(outbox.enqueue(command("overflow"), 1.1));

  const auto expired = outbox.drain(true, 4.0, 3.0);
  EXPECT_TRUE(expired.ready.empty());
  EXPECT_EQ(expired.expired_command_ids, std::vector<std::string>({"old"}));
}

}  // namespace
