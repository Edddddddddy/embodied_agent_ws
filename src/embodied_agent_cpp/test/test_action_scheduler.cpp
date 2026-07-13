#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include <embodied_agent_interfaces/msg/robot_command.hpp>
#include <embodied_agent_interfaces/msg/robot_command_result.hpp>

#include "embodied_agent_cpp/action_scheduler.hpp"

namespace
{

using embodied_agent_cpp::ActionScheduler;
using embodied_agent_cpp::SchedulerEvent;
using embodied_agent_cpp::SchedulerEventKind;
using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;
using RobotCommandResult = embodied_agent_interfaces::msg::RobotCommandResult;

RobotCommand command(const std::string & id, const std::uint8_t action_type)
{
  RobotCommand output;
  output.command_id = id;
  output.source = "scheduler_test";
  output.action_type = action_type;
  return output;
}

RobotCommand priority_command(const std::string & id, const std::uint8_t action_type)
{
  auto output = command(id, action_type);
  output.priority = true;
  return output;
}

std::vector<std::string> ids(
  const std::vector<SchedulerEvent> & events,
  const SchedulerEventKind kind)
{
  std::vector<std::string> output;
  for (const auto & event : events) {
    if (event.kind == kind) {
      output.push_back(event.command_id);
    }
  }
  return output;
}

TEST(ActionSchedulerTest, DispatchesOneGoalAndKeepsFollowingCommandsFifo)
{
  ActionScheduler scheduler(4);

  EXPECT_EQ(
    ids(scheduler.enqueue(command("move-1", RobotCommand::MOVE)), SchedulerEventKind::kDispatch),
    std::vector<std::string>({"move-1"}));
  EXPECT_TRUE(scheduler.enqueue(command("turn-2", RobotCommand::TURN)).empty());
  EXPECT_TRUE(scheduler.enqueue(command("move-3", RobotCommand::MOVE)).empty());

  const auto first_done = scheduler.complete(
    "move-1", true, RobotCommandResult::STATUS_SUCCEEDED, "succeeded");
  EXPECT_EQ(
    ids(first_done, SchedulerEventKind::kDispatch),
    std::vector<std::string>({"turn-2"}));
  const auto second_done = scheduler.complete(
    "turn-2", true, RobotCommandResult::STATUS_SUCCEEDED, "succeeded");
  EXPECT_EQ(
    ids(second_done, SchedulerEventKind::kDispatch),
    std::vector<std::string>({"move-3"}));
}

TEST(ActionSchedulerTest, PriorityStopClearsPendingAndCancelsActiveBeforeDispatch)
{
  ActionScheduler scheduler(4);
  scheduler.enqueue(command("move-1", RobotCommand::MOVE));
  scheduler.enqueue(command("turn-2", RobotCommand::TURN));

  const auto priority = scheduler.enqueue(priority_command("stop-3", RobotCommand::STOP));
  EXPECT_EQ(
    ids(priority, SchedulerEventKind::kResult),
    std::vector<std::string>({"turn-2"}));
  EXPECT_EQ(
    ids(priority, SchedulerEventKind::kCancelActive),
    std::vector<std::string>({"move-1"}));
  EXPECT_TRUE(ids(priority, SchedulerEventKind::kDispatch).empty());
  EXPECT_TRUE(scheduler.snapshot().cancel_requested);

  const auto canceled = scheduler.complete(
    "move-1", false, RobotCommandResult::STATUS_CANCELED, "canceled");
  EXPECT_EQ(
    ids(canceled, SchedulerEventKind::kDispatch),
    std::vector<std::string>({"stop-3"}));
}

TEST(ActionSchedulerTest, PlannedStopRemainsInFifoInsteadOfCancelingSequence)
{
  ActionScheduler scheduler(4);
  scheduler.enqueue(command("move-1", RobotCommand::MOVE));

  const auto queued = scheduler.enqueue(command("planned-stop", RobotCommand::STOP));
  EXPECT_TRUE(queued.empty());
  EXPECT_FALSE(scheduler.snapshot().cancel_requested);

  const auto completed = scheduler.complete(
    "move-1", true, RobotCommandResult::STATUS_SUCCEEDED, "succeeded");
  EXPECT_EQ(
    ids(completed, SchedulerEventKind::kDispatch),
    std::vector<std::string>({"planned-stop"}));
}

TEST(ActionSchedulerTest, QueueFullAndDuplicateIdsProduceExplicitRejections)
{
  ActionScheduler scheduler(1);
  scheduler.enqueue(command("move-1", RobotCommand::MOVE));
  scheduler.enqueue(command("turn-2", RobotCommand::TURN));

  const auto full = scheduler.enqueue(command("move-3", RobotCommand::MOVE));
  ASSERT_EQ(full.size(), 1U);
  EXPECT_EQ(full.front().kind, SchedulerEventKind::kResult);
  EXPECT_EQ(full.front().message, "scheduler_queue_full");
  EXPECT_EQ(full.front().status, RobotCommandResult::STATUS_REJECTED);

  const auto duplicate = scheduler.enqueue(command("turn-2", RobotCommand::TURN));
  ASSERT_EQ(duplicate.size(), 1U);
  EXPECT_EQ(duplicate.front().kind, SchedulerEventKind::kRejectedInput);
  EXPECT_EQ(duplicate.front().message, "duplicate_command_id");
  EXPECT_EQ(scheduler.snapshot().rejected_count, 2U);
}

TEST(ActionSchedulerTest, ActionFailureClearsQueuedCommandsAndStopsDispatch)
{
  ActionScheduler scheduler(4);
  scheduler.enqueue(command("move-1", RobotCommand::MOVE));
  scheduler.enqueue(command("turn-2", RobotCommand::TURN));
  scheduler.enqueue(command("move-3", RobotCommand::MOVE));

  const auto failed = scheduler.complete(
    "move-1", false, RobotCommandResult::STATUS_BLOCKED, "safety_blocked");
  EXPECT_EQ(
    ids(failed, SchedulerEventKind::kResult),
    std::vector<std::string>({"move-1", "turn-2", "move-3"}));
  EXPECT_TRUE(ids(failed, SchedulerEventKind::kDispatch).empty());
  EXPECT_EQ(scheduler.snapshot().cleared_count, 2U);
  EXPECT_EQ(scheduler.snapshot().state, "idle");
}

TEST(ActionSchedulerTest, StaleResultCannotAdvanceCurrentGoal)
{
  ActionScheduler scheduler(4);
  scheduler.enqueue(command("move-1", RobotCommand::MOVE));

  EXPECT_TRUE(scheduler.complete(
      "old-command", true, RobotCommandResult::STATUS_SUCCEEDED, "late").empty());
  EXPECT_EQ(scheduler.snapshot().active_command_id, "move-1");
  EXPECT_EQ(scheduler.snapshot().completed_count, 0U);
}

TEST(ActionSchedulerTest, MissingCommandIdIsRejectedWithoutOccupyingExecutor)
{
  ActionScheduler scheduler;
  const auto rejected = scheduler.enqueue(command("", RobotCommand::MOVE));

  ASSERT_EQ(rejected.size(), 1U);
  EXPECT_EQ(rejected.front().kind, SchedulerEventKind::kRejectedInput);
  EXPECT_EQ(rejected.front().message, "missing_command_id");
  EXPECT_EQ(scheduler.snapshot().state, "idle");
}

}  // namespace
