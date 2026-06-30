#include "embodied_agent_cpp/action_validator.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace embodied_agent_cpp
{

ValidationResult ActionValidator::validate(const std::string & serialized_command) const
{
  ValidationResult result;
  nlohmann::json command;
  try {
    command = nlohmann::json::parse(serialized_command);
  } catch (const nlohmann::json::exception & error) {
    result.error = std::string("invalid JSON: ") + error.what();
    return result;
  }

  if (!command.is_object() || !command.contains("name") || !command["name"].is_string()) {
    result.error = "action requires a string name";
    return result;
  }
  if (!command.contains("arguments")) {
    command["arguments"] = nlohmann::json::object();
  }
  if (!command["arguments"].is_object()) {
    result.error = "action arguments must be an object";
    return result;
  }

  const std::string name = command["name"];
  auto & arguments = command["arguments"];
  if (name == "stop") {
    if (!require_exact_keys(arguments, {}, result.error)) {
      return result;
    }
  } else if (name == "move") {
    if (!require_exact_keys(arguments, {"linear_x", "duration_s"}, result.error) ||
      !require_number(arguments, "linear_x", result.error) ||
      !require_number(arguments, "duration_s", result.error))
    {
      return result;
    }
    arguments["linear_x"] = clamp(arguments["linear_x"].get<double>(), -0.5, 0.5);
    arguments["duration_s"] = clamp(arguments["duration_s"].get<double>(), 0.0, 10.0);
  } else if (name == "turn") {
    if (!require_exact_keys(arguments, {"angular_z", "duration_s"}, result.error) ||
      !require_number(arguments, "angular_z", result.error) ||
      !require_number(arguments, "duration_s", result.error))
    {
      return result;
    }
    arguments["angular_z"] = clamp(arguments["angular_z"].get<double>(), -1.5, 1.5);
    arguments["duration_s"] = clamp(arguments["duration_s"].get<double>(), 0.0, 10.0);
  } else if (name == "wave") {
    if (!require_exact_keys(arguments, {"count"}, result.error) ||
      !require_number(arguments, "count", result.error))
    {
      return result;
    }
    arguments["count"] = static_cast<int>(
      std::lround(clamp(arguments["count"].get<double>(), 1.0, 5.0)));
  } else if (name == "set_led") {
    if (!require_exact_keys(arguments, {"color"}, result.error) ||
      !arguments["color"].is_string())
    {
      if (result.error.empty()) {
        result.error = "color must be a string";
      }
      return result;
    }
  } else {
    result.error = "unsupported action: " + name;
    return result;
  }

  result.valid = true;
  result.command = std::move(command);
  return result;
}

bool ActionValidator::require_exact_keys(
  const nlohmann::json & arguments,
  std::initializer_list<const char *> keys,
  std::string & error)
{
  const std::set<std::string> expected(keys.begin(), keys.end());
  std::set<std::string> actual;
  for (const auto & item : arguments.items()) {
    actual.insert(item.key());
  }
  if (actual != expected) {
    error = "arguments do not match the action schema";
    return false;
  }
  return true;
}

bool ActionValidator::require_number(
  const nlohmann::json & arguments,
  const char * key,
  std::string & error)
{
  if (!arguments.contains(key) || !arguments[key].is_number()) {
    error = std::string(key) + " must be numeric";
    return false;
  }
  return true;
}

double ActionValidator::clamp(double value, double lower, double upper)
{
  return std::clamp(value, lower, upper);
}

}  // namespace embodied_agent_cpp
