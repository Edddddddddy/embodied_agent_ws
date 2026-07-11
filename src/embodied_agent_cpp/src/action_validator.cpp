#include "embodied_agent_cpp/action_validator.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>

namespace embodied_agent_cpp
{
namespace
{
using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

const std::set<std::string> & supported_navigation_places()
{
  static const std::set<std::string> places{
    "home", "door", "desk", "living_room", "kitchen", "charging_station",
    "waypoint_a", "waypoint_b", "waypoint_c", "unreachable_zone"};
  return places;
}

bool supported_place(const std::string & value)
{
  return supported_navigation_places().count(value) > 0;
}

bool no_motion_payload(const RobotCommand & command)
{
  return command.linear_x == 0.0 && command.angular_z == 0.0 &&
         command.duration_s == 0.0;
}

bool no_accessory_payload(const RobotCommand & command)
{
  return command.count == 0 && command.color.empty() && command.mode.empty();
}

bool no_navigation_payload(const RobotCommand & command)
{
  return command.target.empty() && command.waypoints.empty() &&
         command.number_of_loops == 0;
}

bool no_non_motion_payload(const RobotCommand & command)
{
  return no_accessory_payload(command) && no_navigation_payload(command);
}

bool no_payload(const RobotCommand & command)
{
  return no_motion_payload(command) && no_accessory_payload(command) &&
         no_navigation_payload(command);
}

std::string validate_navigation_places(const RobotCommand & command)
{
  if (command.waypoints.empty()) {
    return "waypoints must not be empty";
  }
  if (command.waypoints.size() > 8U) {
    return "too many waypoints";
  }
  for (const auto & waypoint : command.waypoints) {
    if (!supported_place(waypoint)) {
      return "unsupported navigation waypoint";
    }
  }
  return {};
}
}  // namespace

ValidationResult ActionValidator::validate(
  const RobotCommand & candidate,
  const std::string & fallback_command_id,
  const std::string & fallback_source) const
{
  ValidationResult result;
  result.command = candidate;
  result.command.command_id = candidate.command_id.empty() ?
    fallback_command_id : candidate.command_id;
  result.command.source = candidate.source.empty() ?
    fallback_source : candidate.source;
  if (result.command.command_id.empty()) {
    result.error = "command_id must not be empty";
    return result;
  }

  switch (candidate.action_type) {
    case RobotCommand::STOP:
    case RobotCommand::CANCEL_NAVIGATION:
      if (!no_payload(candidate)) {
        result.error = "control command contains unrelated payload";
        return result;
      }
      break;

    case RobotCommand::MOVE:
    case RobotCommand::ARC:
      if (!no_non_motion_payload(candidate) ||
        !std::isfinite(candidate.linear_x) ||
        !std::isfinite(candidate.angular_z) ||
        !std::isfinite(candidate.duration_s))
      {
        result.error = "invalid move payload";
        return result;
      }
      result.command.linear_x = clamp(candidate.linear_x, -0.5, 0.5);
      result.command.angular_z = clamp(candidate.angular_z, -1.5, 1.5);
      result.command.duration_s = clamp(candidate.duration_s, 0.0, 10.0);
      result.command.action_type = RobotCommand::MOVE;
      break;

    case RobotCommand::TURN:
      if (candidate.linear_x != 0.0 || !no_non_motion_payload(candidate) ||
        !std::isfinite(candidate.angular_z) ||
        !std::isfinite(candidate.duration_s))
      {
        result.error = "invalid turn payload";
        return result;
      }
      result.command.angular_z = clamp(candidate.angular_z, -1.5, 1.5);
      result.command.duration_s = clamp(candidate.duration_s, 0.0, 10.0);
      break;

    case RobotCommand::WAVE:
      if (!no_motion_payload(candidate) || !candidate.color.empty() ||
        !candidate.mode.empty() || !no_navigation_payload(candidate) ||
        candidate.count == 0)
      {
        result.error = "invalid wave payload";
        return result;
      }
      result.command.count = static_cast<std::int32_t>(
        std::clamp(candidate.count, 1, 5));
      break;

    case RobotCommand::SET_LED: {
        if (!no_motion_payload(candidate) || candidate.count != 0 ||
          !candidate.mode.empty() || !no_navigation_payload(candidate))
        {
          result.error = "invalid LED payload";
          return result;
        }
        static const std::set<std::string> colors{
          "off", "red", "green", "blue", "yellow", "white"};
        if (colors.count(candidate.color) == 0) {
          result.error = "unsupported LED color";
          return result;
        }
        break;
      }

    case RobotCommand::SET_MODE: {
        if (!no_motion_payload(candidate) || candidate.count != 0 ||
          !candidate.color.empty() || !no_navigation_payload(candidate))
        {
          result.error = "invalid mode payload";
          return result;
        }
        static const std::set<std::string> modes{
          "manual", "obstacle_avoidance", "wall_following"};
        if (modes.count(candidate.mode) == 0) {
          result.error = "unsupported control mode";
          return result;
        }
        break;
      }

    case RobotCommand::NAVIGATE_TO:
      if (!no_motion_payload(candidate) || !no_accessory_payload(candidate) ||
        !candidate.waypoints.empty() || candidate.number_of_loops != 0 ||
        !supported_place(candidate.target))
      {
        result.error = supported_place(candidate.target) ?
          "invalid navigation payload" : "unsupported navigation target";
        return result;
      }
      result.command.duration_s = 3.0;
      break;

    case RobotCommand::FOLLOW_WAYPOINTS: {
        if (!no_motion_payload(candidate) || !no_accessory_payload(candidate) ||
          !candidate.target.empty())
        {
          result.error = "invalid waypoint payload";
          return result;
        }
        result.error = validate_navigation_places(candidate);
        if (!result.error.empty()) {
          return result;
        }
        result.command.number_of_loops = std::clamp(
          candidate.number_of_loops == 0U ? 1U : candidate.number_of_loops,
          1U, 3U);
        const auto visits = static_cast<double>(
          result.command.waypoints.size() * result.command.number_of_loops);
        result.command.duration_s = std::clamp(visits * 2.0, 2.0, 10.0);
        break;
      }

    default:
      result.error = "unsupported action type";
      return result;
  }

  result.valid = true;
  return result;
}

double ActionValidator::clamp(double value, double lower, double upper)
{
  return std::clamp(value, lower, upper);
}

}  // namespace embodied_agent_cpp
