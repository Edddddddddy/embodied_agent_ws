#include <string>

#include "gtest/gtest.h"

#include "embodied_agent_cpp/robot_command_adapter.hpp"

TEST(RobotCommandAdapterTest, ConvertsAndClampsMoveIntoTypedCommand)
{
  embodied_agent_cpp::RobotCommandAdapter adapter;
  const auto result = adapter.convert(
    R"({"name":"move","arguments":{"linear_x":9.0,"duration_s":20.0}})",
    "command-42", "online_agent");

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_EQ(
    result.typed_command.action_type,
    embodied_agent_interfaces::msg::RobotCommand::MOVE);
  EXPECT_EQ(result.typed_command.command_id, "command-42");
  EXPECT_EQ(result.typed_command.source, "online_agent");
  EXPECT_DOUBLE_EQ(result.typed_command.linear_x, 0.5);
  EXPECT_DOUBLE_EQ(result.typed_command.duration_s, 10.0);
  EXPECT_DOUBLE_EQ(result.legacy_command["arguments"]["linear_x"], 0.5);
}

TEST(RobotCommandAdapterTest, UsesRequestIdWhenProvidedByAgent)
{
  embodied_agent_cpp::RobotCommandAdapter adapter;
  const auto result = adapter.convert(
    R"({"name":"move","request_id":"agent-action-7","arguments":{"linear_x":0.2,"duration_s":1.0}})",
    "guard-1", "online_agent");

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_EQ(result.typed_command.command_id, "agent-action-7");
}

TEST(RobotCommandAdapterTest, ConvertsArcIntoTypedCurvedMove)
{
  embodied_agent_cpp::RobotCommandAdapter adapter;
  const auto result = adapter.convert(
    R"({"name":"arc","arguments":{"linear_x":0.12,"angular_z":0.45,"duration_s":6.0}})",
    "arc-1", "online_agent");

  ASSERT_TRUE(result.valid) << result.error;
  EXPECT_EQ(
    result.typed_command.action_type,
    embodied_agent_interfaces::msg::RobotCommand::MOVE);
  EXPECT_DOUBLE_EQ(result.typed_command.linear_x, 0.12);
  EXPECT_DOUBLE_EQ(result.typed_command.angular_z, 0.45);
  EXPECT_DOUBLE_EQ(result.typed_command.duration_s, 6.0);
  EXPECT_EQ(result.legacy_command["name"], "move");
}

TEST(RobotCommandAdapterTest, ConvertsTurnAndStopWithoutAmbiguousFields)
{
  embodied_agent_cpp::RobotCommandAdapter adapter;
  const auto turn = adapter.convert(
    R"({"name":"turn","arguments":{"angular_z":-0.8,"duration_s":1.5}})",
    "turn-1", "offline_agent");
  ASSERT_TRUE(turn.valid) << turn.error;
  EXPECT_EQ(
    turn.typed_command.action_type,
    embodied_agent_interfaces::msg::RobotCommand::TURN);
  EXPECT_DOUBLE_EQ(turn.typed_command.angular_z, -0.8);
  EXPECT_DOUBLE_EQ(turn.typed_command.duration_s, 1.5);
  EXPECT_DOUBLE_EQ(turn.typed_command.linear_x, 0.0);

  const auto stop = adapter.convert(
    R"({"name":"stop","arguments":{}})", "stop-1", "online_agent");
  ASSERT_TRUE(stop.valid) << stop.error;
  EXPECT_EQ(
    stop.typed_command.action_type,
    embodied_agent_interfaces::msg::RobotCommand::STOP);
  EXPECT_DOUBLE_EQ(stop.typed_command.duration_s, 0.0);
}

TEST(RobotCommandAdapterTest, ConvertsWaveLedAndControlMode)
{
  embodied_agent_cpp::RobotCommandAdapter adapter;

  const auto wave = adapter.convert(
    R"({"name":"wave","arguments":{"count":3}})", "wave-1", "agent");
  ASSERT_TRUE(wave.valid) << wave.error;
  EXPECT_EQ(wave.typed_command.action_type,
    embodied_agent_interfaces::msg::RobotCommand::WAVE);
  EXPECT_EQ(wave.typed_command.count, 3);

  const auto led = adapter.convert(
    R"({"name":"set_led","arguments":{"color":"blue"}})",
    "led-1", "agent");
  ASSERT_TRUE(led.valid) << led.error;
  EXPECT_EQ(led.typed_command.action_type,
    embodied_agent_interfaces::msg::RobotCommand::SET_LED);
  EXPECT_EQ(led.typed_command.color, "blue");

  const auto mode = adapter.convert(
    R"({"name":"set_mode","arguments":{"mode":"wall_following"}})",
    "mode-1", "agent");
  ASSERT_TRUE(mode.valid) << mode.error;
  EXPECT_EQ(mode.typed_command.action_type,
    embodied_agent_interfaces::msg::RobotCommand::SET_MODE);
  EXPECT_EQ(mode.typed_command.mode, "wall_following");
}

TEST(RobotCommandAdapterTest, RejectsInvalidInputWithoutProducingACommand)
{
  embodied_agent_cpp::RobotCommandAdapter adapter;
  const auto result = adapter.convert(
    R"({"name":"move","arguments":{"linear_x":0.2}})",
    "invalid-1", "agent");

  EXPECT_FALSE(result.valid);
  EXPECT_FALSE(result.error.empty());
  EXPECT_TRUE(result.legacy_command.is_null());
  EXPECT_TRUE(result.typed_command.command_id.empty());
  EXPECT_EQ(
    result.typed_command.action_type,
    embodied_agent_interfaces::msg::RobotCommand::UNKNOWN);
}

TEST(RobotCommandAdapterTest, ConvertsNavigationCommands)
{
  embodied_agent_cpp::RobotCommandAdapter adapter;
  const auto target = adapter.convert(
    R"({"name":"navigate_to","arguments":{"target":"door"}})",
    "nav-1", "agent");
  ASSERT_TRUE(target.valid) << target.error;
  EXPECT_EQ(
    target.typed_command.action_type,
    embodied_agent_interfaces::msg::RobotCommand::NAVIGATE_TO);
  EXPECT_EQ(target.typed_command.target, "door");
  EXPECT_DOUBLE_EQ(target.typed_command.duration_s, 3.0);

  const auto patrol = adapter.convert(
    R"({"name":"follow_waypoints","arguments":{"waypoints":["door","desk","home"],"number_of_loops":2}})",
    "patrol-1", "agent");
  ASSERT_TRUE(patrol.valid) << patrol.error;
  EXPECT_EQ(
    patrol.typed_command.action_type,
    embodied_agent_interfaces::msg::RobotCommand::FOLLOW_WAYPOINTS);
  ASSERT_EQ(patrol.typed_command.waypoints.size(), 3U);
  EXPECT_EQ(patrol.typed_command.waypoints[1], "desk");
  EXPECT_EQ(patrol.typed_command.number_of_loops, 2U);
  EXPECT_DOUBLE_EQ(patrol.typed_command.duration_s, 10.0);

  const auto cancel = adapter.convert(
    R"({"name":"cancel_navigation","arguments":{}})",
    "cancel-1", "agent");
  ASSERT_TRUE(cancel.valid) << cancel.error;
  EXPECT_EQ(
    cancel.typed_command.action_type,
    embodied_agent_interfaces::msg::RobotCommand::CANCEL_NAVIGATION);
}
