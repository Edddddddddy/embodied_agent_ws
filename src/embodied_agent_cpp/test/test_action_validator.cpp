#include <limits>
#include <string>

#include "gtest/gtest.h"

#include "embodied_agent_cpp/action_validator.hpp"

namespace
{
using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

RobotCommand candidate(std::uint8_t type)
{
  RobotCommand command;
  command.action_type = type;
  return command;
}
}  // namespace

TEST(ActionValidatorTest, ClampsMoveAndPreservesTypedIdentity)
{
  embodied_agent_cpp::ActionValidator validator;
  auto command = candidate(RobotCommand::MOVE);
  command.command_id = "agent-action-7";
  command.source = "offline";
  command.linear_x = 9.0;
  command.angular_z = -9.0;
  command.duration_s = 20.0;

  const auto result = validator.validate(command, "guard-1", "agent");

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_EQ(result.command.command_id, "agent-action-7");
  EXPECT_EQ(result.command.source, "offline");
  EXPECT_DOUBLE_EQ(result.command.linear_x, 0.5);
  EXPECT_DOUBLE_EQ(result.command.angular_z, -1.5);
  EXPECT_DOUBLE_EQ(result.command.duration_s, 10.0);
}

TEST(ActionValidatorTest, SuppliesIdentityWhenCandidateOmitsIt)
{
  embodied_agent_cpp::ActionValidator validator;
  const auto result = validator.validate(
    candidate(RobotCommand::STOP), "guard-42", "agent");

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_EQ(result.command.command_id, "guard-42");
  EXPECT_EQ(result.command.source, "agent");
}

TEST(ActionValidatorTest, NormalizesArcCandidateIntoCurvedMove)
{
  embodied_agent_cpp::ActionValidator validator;
  auto arc = candidate(RobotCommand::ARC);
  arc.linear_x = 0.12;
  arc.angular_z = 0.45;
  arc.duration_s = 6.0;

  const auto result = validator.validate(arc, "arc-1", "agent");

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_EQ(result.command.action_type, RobotCommand::MOVE);
  EXPECT_DOUBLE_EQ(result.command.linear_x, 0.12);
  EXPECT_DOUBLE_EQ(result.command.angular_z, 0.45);
  EXPECT_DOUBLE_EQ(result.command.duration_s, 6.0);
}

TEST(ActionValidatorTest, RejectsUnknownAndNonFiniteMotion)
{
  embodied_agent_cpp::ActionValidator validator;
  EXPECT_FALSE(validator.validate(
      candidate(RobotCommand::UNKNOWN), "guard-1", "agent").valid);

  auto move = candidate(RobotCommand::MOVE);
  move.linear_x = std::numeric_limits<double>::quiet_NaN();
  move.duration_s = 1.0;
  EXPECT_FALSE(validator.validate(move, "guard-2", "agent").valid);
}

TEST(ActionValidatorTest, RejectsAmbiguousFieldsForStopAndTurn)
{
  embodied_agent_cpp::ActionValidator validator;
  auto stop = candidate(RobotCommand::STOP);
  stop.duration_s = 1.0;
  EXPECT_FALSE(validator.validate(stop, "stop-1", "agent").valid);

  auto turn = candidate(RobotCommand::TURN);
  turn.linear_x = 0.2;
  turn.angular_z = 0.6;
  turn.duration_s = 1.0;
  EXPECT_FALSE(validator.validate(turn, "turn-1", "agent").valid);
}

TEST(ActionValidatorTest, ValidatesAccessoryAndModePayloads)
{
  embodied_agent_cpp::ActionValidator validator;

  auto wave = candidate(RobotCommand::WAVE);
  wave.count = 9;
  const auto normalized_wave = validator.validate(wave, "wave-1", "agent");
  ASSERT_TRUE(normalized_wave.valid) << normalized_wave.error;
  EXPECT_EQ(normalized_wave.command.count, 5);

  auto led = candidate(RobotCommand::SET_LED);
  led.color = "purple";
  const auto invalid_led = validator.validate(led, "led-1", "agent");
  EXPECT_FALSE(invalid_led.valid);
  EXPECT_EQ(invalid_led.error, "unsupported LED color");

  auto mode = candidate(RobotCommand::SET_MODE);
  mode.mode = "obstacle_avoidance";
  EXPECT_TRUE(validator.validate(mode, "mode-1", "agent").valid);
  mode.mode = "unsafe_racing";
  EXPECT_FALSE(validator.validate(mode, "mode-2", "agent").valid);
}

TEST(ActionValidatorTest, ValidatesNavigationAndNormalizesRuntimeFields)
{
  embodied_agent_cpp::ActionValidator validator;

  auto target = candidate(RobotCommand::NAVIGATE_TO);
  target.target = "door";
  const auto normalized_target = validator.validate(target, "nav-1", "agent");
  ASSERT_TRUE(normalized_target.valid) << normalized_target.error;
  EXPECT_DOUBLE_EQ(normalized_target.command.duration_s, 3.0);

  auto patrol = candidate(RobotCommand::FOLLOW_WAYPOINTS);
  patrol.waypoints = {"door", "desk", "home"};
  patrol.number_of_loops = 9;
  const auto normalized_patrol = validator.validate(patrol, "patrol-1", "agent");
  ASSERT_TRUE(normalized_patrol.valid) << normalized_patrol.error;
  EXPECT_EQ(normalized_patrol.command.number_of_loops, 3U);
  EXPECT_DOUBLE_EQ(normalized_patrol.command.duration_s, 10.0);

  target.target = "server_room";
  EXPECT_FALSE(validator.validate(target, "nav-2", "agent").valid);
  target.target = "unreachable_zone";
  EXPECT_TRUE(validator.validate(target, "nav-3", "agent").valid);
}

TEST(ActionValidatorTest, CancelNavigationRejectsUnrelatedPayload)
{
  embodied_agent_cpp::ActionValidator validator;
  auto cancel = candidate(RobotCommand::CANCEL_NAVIGATION);
  EXPECT_TRUE(validator.validate(cancel, "cancel-1", "agent").valid);
  cancel.target = "door";
  EXPECT_FALSE(validator.validate(cancel, "cancel-2", "agent").valid);
}

TEST(ActionValidatorTest, PriorityIsRestrictedToControlCommands)
{
  embodied_agent_cpp::ActionValidator validator;

  auto move = candidate(RobotCommand::MOVE);
  move.linear_x = 0.2;
  move.duration_s = 1.0;
  move.priority = true;
  const auto invalid = validator.validate(move, "move-priority", "agent");
  EXPECT_FALSE(invalid.valid);
  EXPECT_EQ(invalid.error, "priority is only valid for stop or cancel_navigation");

  auto stop = candidate(RobotCommand::STOP);
  stop.priority = true;
  const auto valid = validator.validate(stop, "stop-priority", "agent");
  ASSERT_TRUE(valid.valid) << valid.error;
  EXPECT_TRUE(valid.command.priority);
}
