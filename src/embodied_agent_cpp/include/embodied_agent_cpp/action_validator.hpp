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

