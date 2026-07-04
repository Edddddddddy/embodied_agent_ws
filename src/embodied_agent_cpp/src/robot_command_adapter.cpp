#include "embodied_agent_cpp/robot_command_adapter.hpp"

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

  conversion.legacy_command = validation.command;
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
  } else {
    conversion.error = "typed adapter does not support action: " + name;
    return conversion;
  }

  conversion.valid = true;
  return conversion;
}

}  // namespace embodied_agent_cpp
