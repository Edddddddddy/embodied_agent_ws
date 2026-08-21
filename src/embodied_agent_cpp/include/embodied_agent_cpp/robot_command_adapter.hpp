#pragma once

#include <string>

#include <nlohmann/json.hpp>

#include "embodied_agent_interfaces/msg/robot_command.hpp"

namespace embodied_agent_cpp
{

struct RobotCommandConversion
{
  bool valid{false};
  nlohmann::json legacy_command;
  embodied_agent_interfaces::msg::RobotCommand typed_command;
  std::string error;
};

class RobotCommandAdapter
{
public:
  RobotCommandConversion convert(
    const std::string & serialized_command,
    const std::string & command_id,
    const std::string & source) const;
};

}  // namespace embodied_agent_cpp
