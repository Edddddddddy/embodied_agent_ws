#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>

#include <embodied_agent_interfaces/msg/robot_command.hpp>

#include "embodied_simulation/robot_command_policy.hpp"

namespace embodied_simulation
{
namespace
{

using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

RobotCommand command(std::uint8_t action_type)
{
  RobotCommand value;
  value.action_type = action_type;
  return value;
}

TEST(RobotCommandPolicyTest, AcceptsImmediateControlCommands)
{
  EXPECT_TRUE(is_executable_robot_command(command(RobotCommand::STOP)));
  EXPECT_TRUE(
    is_executable_robot_command(command(RobotCommand::CANCEL_NAVIGATION)));
  EXPECT_FALSE(is_executable_robot_command(command(RobotCommand::UNKNOWN)));
  // ARC 是候选层语义；ActionGuard 会先规范化为 MOVE，执行层不应再收到 ARC。
  EXPECT_FALSE(is_executable_robot_command(command(RobotCommand::ARC)));
}

TEST(RobotCommandPolicyTest, ValidatesModeWaveAndLedDomains)
{
  for (const std::string mode : {
      "manual", "obstacle_avoidance", "wall_following"})
  {
    auto value = command(RobotCommand::SET_MODE);
    value.mode = mode;
    EXPECT_TRUE(is_executable_robot_command(value));
  }
  auto mode = command(RobotCommand::SET_MODE);
  mode.mode = "autonomous";
  EXPECT_FALSE(is_executable_robot_command(mode));

  for (const auto count : {1, 5}) {
    auto value = command(RobotCommand::WAVE);
    value.count = count;
    EXPECT_TRUE(is_executable_robot_command(value));
  }
  for (const auto count : {0, 6}) {
    auto value = command(RobotCommand::WAVE);
    value.count = count;
    EXPECT_FALSE(is_executable_robot_command(value));
  }

  for (const std::string color : {
      "off", "red", "green", "blue", "yellow", "white"})
  {
    auto value = command(RobotCommand::SET_LED);
    value.color = color;
    EXPECT_TRUE(is_executable_robot_command(value));
  }
  auto led = command(RobotCommand::SET_LED);
  led.color = "purple";
  EXPECT_FALSE(is_executable_robot_command(led));
}

TEST(RobotCommandPolicyTest, RejectsNonFiniteOrOutOfRangeTimedMotion)
{
  auto move = command(RobotCommand::MOVE);
  move.linear_x = 0.2;
  move.angular_z = 0.4;
  move.duration_s = 0.0;
  EXPECT_TRUE(is_executable_robot_command(move));
  move.duration_s = 10.0;
  EXPECT_TRUE(is_executable_robot_command(move));
  move.duration_s = -0.01;
  EXPECT_FALSE(is_executable_robot_command(move));
  move.duration_s = 10.01;
  EXPECT_FALSE(is_executable_robot_command(move));
  move.duration_s = 1.0;
  move.linear_x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(is_executable_robot_command(move));
  move.linear_x = 0.2;
  move.angular_z = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(is_executable_robot_command(move));

  auto turn = command(RobotCommand::TURN);
  turn.angular_z = 0.8;
  turn.duration_s = 1.0;
  EXPECT_TRUE(is_executable_robot_command(turn));
  turn.duration_s = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(is_executable_robot_command(turn));
}

TEST(RobotCommandPolicyTest, ValidatesNavigationTargetsAndWaypointLoops)
{
  auto navigate = command(RobotCommand::NAVIGATE_TO);
  navigate.target = "meeting_room";
  navigate.duration_s = 10.0;
  EXPECT_TRUE(is_executable_robot_command(navigate));
  navigate.target.clear();
  EXPECT_FALSE(is_executable_robot_command(navigate));
  navigate.target = "meeting_room";
  navigate.duration_s = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(is_executable_robot_command(navigate));

  auto patrol = command(RobotCommand::FOLLOW_WAYPOINTS);
  patrol.waypoints = {"door", "desk"};
  patrol.number_of_loops = 1;
  patrol.duration_s = 0.0;
  EXPECT_TRUE(is_executable_robot_command(patrol));
  patrol.number_of_loops = 3;
  patrol.duration_s = 10.0;
  EXPECT_TRUE(is_executable_robot_command(patrol));
  patrol.number_of_loops = 0;
  EXPECT_FALSE(is_executable_robot_command(patrol));
  patrol.number_of_loops = 4;
  EXPECT_FALSE(is_executable_robot_command(patrol));
  patrol.number_of_loops = 1;
  patrol.waypoints.clear();
  EXPECT_FALSE(is_executable_robot_command(patrol));
}

}  // namespace
}  // namespace embodied_simulation
