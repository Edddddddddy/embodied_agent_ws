#pragma once

#include <string>

#include <nlohmann/json.hpp>

namespace embodied_agent_cpp
{

struct ValidationResult
{
  bool valid{false};
  nlohmann::json command;
  std::string error;
};

class ActionValidator
{
public:
  // LLM 输出始终是不可信输入：这里统一完成 JSON schema、白名单和数值限幅。
  // 调用方只根据 valid/error 决策，不能绕过该 seam 直接控制机器人。
  ValidationResult validate(const std::string & serialized_command) const;

private:
  static bool require_exact_keys(
    const nlohmann::json & arguments,
    std::initializer_list<const char *> keys,
    std::string & error);
  static bool require_number(
    const nlohmann::json & arguments,
    const char * key,
    std::string & error);
  static double clamp(double value, double lower, double upper);
};

}  // namespace embodied_agent_cpp
