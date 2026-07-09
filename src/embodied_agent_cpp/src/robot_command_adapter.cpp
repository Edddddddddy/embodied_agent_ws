#include "embodied_agent_cpp/robot_command_adapter.hpp"

#include <algorithm>
#include <cstdint>
#include <utility>

#include "embodied_agent_cpp/action_validator.hpp"

namespace embodied_agent_cpp
{

RobotCommandConversion RobotCommandAdapter::convert(
  const std::string & serialized_command,
  const std::string & command_id,
  const std::string & source) const
{
  RobotCommandConversion conversion;
  const auto validation = ActionValidator().validate(serialized_command);
  if (!validation.valid) {
    conversion.error = validation.error;
    return conversion;
  }

  conversion.typed_command.command_id = command_id;
  if (validation.command.contains("request_id")) {
    conversion.typed_command.command_id =
      validation.command.at("request_id").get<std::string>();
  }
  conversion.typed_command.source = source;

  const auto & name = validation.command.at("name").get_ref<const std::string &>();
  const auto & arguments = validation.command.at("arguments");
  if (name == "move") {
    conversion.typed_command.action_type =
      embodied_agent_interfaces::msg::RobotCommand::MOVE;
    conversion.typed_command.linear_x = arguments.at("linear_x").get<double>();
    if (arguments.contains("angular_z")) {
      conversion.typed_command.angular_z = arguments.at("angular_z").get<double>();
    }
    conversion.typed_command.duration_s = arguments.at("duration_s").get<double>();
  } else if (name == "turn") {
    conversion.typed_command.action_type =
      embodied_agent_interfaces::msg::RobotCommand::TURN;
    conversion.typed_command.angular_z = arguments.at("angular_z").get<double>();
    conversion.typed_command.duration_s = arguments.at("duration_s").get<double>();
  } else if (name == "stop") {
    conversion.typed_command.action_type =
      embodied_agent_interfaces::msg::RobotCommand::STOP;
  } else if (name == "wave") {
    conversion.typed_command.action_type =
      embodied_agent_interfaces::msg::RobotCommand::WAVE;
    conversion.typed_command.count = arguments.at("count").get<int>();
  } else if (name == "set_led") {
    conversion.typed_command.action_type =
      embodied_agent_interfaces::msg::RobotCommand::SET_LED;
    conversion.typed_command.color = arguments.at("color").get<std::string>();
  } else if (name == "set_mode") {
    conversion.typed_command.action_type =
      embodied_agent_interfaces::msg::RobotCommand::SET_MODE;
    conversion.typed_command.mode = arguments.at("mode").get<std::string>();
  } else if (name == "navigate_to") {
    conversion.typed_command.action_type =
      embodied_agent_interfaces::msg::RobotCommand::NAVIGATE_TO;
    conversion.typed_command.target = arguments.at("target").get<std::string>();
    // 第一版导航执行器用估算时长模拟 Nav2 Action 的运行窗口；后续真实 Nav2 bridge
    // 会用 NavigateToPose 的 result 决定终态。
    conversion.typed_command.duration_s = 3.0;
  } else if (name == "follow_waypoints") {
    conversion.typed_command.action_type =
      embodied_agent_interfaces::msg::RobotCommand::FOLLOW_WAYPOINTS;
    for (const auto & waypoint : arguments.at("waypoints")) {
      conversion.typed_command.waypoints.push_back(waypoint.get<std::string>());
    }
    conversion.typed_command.number_of_loops =
      static_cast<std::uint32_t>(arguments.at("number_of_loops").get<int>());
    const auto waypoint_count = static_cast<double>(
      conversion.typed_command.waypoints.size() *
      std::max<std::uint32_t>(1U, conversion.typed_command.number_of_loops));
    conversion.typed_command.duration_s = std::min(10.0, std::max(2.0, waypoint_count * 2.0));
  } else if (name == "cancel_navigation") {
    conversion.typed_command.action_type =
      embodied_agent_interfaces::msg::RobotCommand::CANCEL_NAVIGATION;
  } else {
    conversion.error = "typed adapter does not support action: " + name;
    return conversion;
  }

  conversion.valid = true;
  return conversion;
}

}  // namespace embodied_agent_cpp
