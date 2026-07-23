#include "embodied_simulation/robot_command_policy.hpp"

#include <cmath>

namespace embodied_simulation
{
namespace
{

using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

bool valid_duration(double duration_s)
{
  return std::isfinite(duration_s) && duration_s >= 0.0 && duration_s <= 10.0;
}

}  // namespace

bool is_executable_robot_command(const RobotCommand & command)
{
  switch (command.action_type) {
    case RobotCommand::STOP:
    case RobotCommand::CANCEL_NAVIGATION:
      return true;

    case RobotCommand::SET_MODE:
      return command.mode == "manual" ||
             command.mode == "obstacle_avoidance" ||
             command.mode == "wall_following";

    case RobotCommand::WAVE:
      return command.count >= 1 && command.count <= 5;

    case RobotCommand::SET_LED:
      return command.color == "off" ||
             command.color == "red" ||
             command.color == "green" ||
             command.color == "blue" ||
             command.color == "yellow" ||
             command.color == "white";

    case RobotCommand::MOVE:
      return std::isfinite(command.linear_x) &&
             std::isfinite(command.angular_z) &&
             valid_duration(command.duration_s);

    case RobotCommand::TURN:
      return std::isfinite(command.angular_z) &&
             valid_duration(command.duration_s);

    case RobotCommand::NAVIGATE_TO:
      return !command.target.empty() && valid_duration(command.duration_s);

    case RobotCommand::FOLLOW_WAYPOINTS:
      return !command.waypoints.empty() &&
             command.number_of_loops >= 1 &&
             command.number_of_loops <= 3 &&
             valid_duration(command.duration_s);

    default:
      // UNKNOWN 与候选层 ARC 都不能越过 ActionGuard 直接进入执行器。
      return false;
  }
}

}  // namespace embodied_simulation
