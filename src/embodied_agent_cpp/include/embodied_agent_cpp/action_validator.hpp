#pragma once

#include <string>

#include "embodied_agent_interfaces/msg/robot_command.hpp"

namespace embodied_agent_cpp
{

struct ValidationResult
{
  bool valid{false};
  embodied_agent_interfaces::msg::RobotCommand command;
  std::string error;
};

class ActionValidator
{
public:
  // Agent 候选消息仍是不可信输入：这里统一完成动作白名单、字段约束和数值限幅。
  // 强类型消息消除了 JSON schema 解析，但不能替代业务层安全校验。
  ValidationResult validate(
    const embodied_agent_interfaces::msg::RobotCommand & candidate,
    const std::string & fallback_command_id,
    const std::string & fallback_source) const;

private:
  static double clamp(double value, double lower, double upper);
};

}  // namespace embodied_agent_cpp
