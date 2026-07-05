#include "embodied_agent_cpp/action_validator.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <string>

namespace embodied_agent_cpp
{

namespace
{
const std::set<std::string> & supported_navigation_places()
{
  static const std::set<std::string> places{
    "home", "door", "desk", "living_room", "kitchen", "charging_station",
    "waypoint_a", "waypoint_b", "waypoint_c"};
  return places;
}

bool is_supported_navigation_place(const nlohmann::json & value)
{
  return value.is_string() &&
         supported_navigation_places().count(value.get<std::string>()) > 0;
}
}  // namespace

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
  if (command.contains("request_id") && !command["request_id"].is_string()) {
    result.error = "request_id must be a string";
    return result;
  }

  const std::string name = command["name"];
  auto & arguments = command["arguments"];
  if (name == "stop") {
    if (!require_exact_keys(arguments, {}, result.error)) {
      return result;
    }
  } else if (name == "move" || name == "arc") {
    const bool curved_move = arguments.contains("angular_z") || name == "arc";
    if (!require_exact_keys(
        arguments,
        curved_move ? std::initializer_list<const char *>{"linear_x", "angular_z", "duration_s"} :
        std::initializer_list<const char *>{"linear_x", "duration_s"},
        result.error) ||
      !require_number(arguments, "linear_x", result.error) ||
      (curved_move && !require_number(arguments, "angular_z", result.error)) ||
      !require_number(arguments, "duration_s", result.error))
    {
      return result;
    }
    command["name"] = "move";
    arguments["linear_x"] = clamp(arguments["linear_x"].get<double>(), -0.5, 0.5);
    if (curved_move) {
      arguments["angular_z"] = clamp(arguments["angular_z"].get<double>(), -1.5, 1.5);
    }
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
    static const std::set<std::string> supported_colors{
      "off", "red", "green", "blue", "yellow", "white"};
    if (supported_colors.count(arguments["color"].get<std::string>()) == 0) {
      result.error = "unsupported LED color";
      return result;
    }
  } else if (name == "set_mode") {
    if (!require_exact_keys(arguments, {"mode"}, result.error) ||
      !arguments["mode"].is_string())
    {
      if (result.error.empty()) {
        result.error = "mode must be a string";
      }
      return result;
    }
    static const std::set<std::string> supported_modes{
      "manual", "obstacle_avoidance", "wall_following"};
    if (supported_modes.count(arguments["mode"].get<std::string>()) == 0) {
      result.error = "unsupported control mode";
      return result;
    }
  } else if (name == "navigate_to") {
    if (!require_exact_keys(arguments, {"target"}, result.error)) {
      return result;
    }
    if (!is_supported_navigation_place(arguments["target"])) {
      result.error = "unsupported navigation target";
      return result;
    }
  } else if (name == "follow_waypoints" || name == "patrol") {
    command["name"] = "follow_waypoints";
    const bool has_loops = arguments.contains("number_of_loops");
    if (!require_exact_keys(
        arguments,
        has_loops ?
        std::initializer_list<const char *>{"waypoints", "number_of_loops"} :
        std::initializer_list<const char *>{"waypoints"},
        result.error))
    {
      return result;
    }
    if (!arguments["waypoints"].is_array() || arguments["waypoints"].empty()) {
      result.error = "waypoints must be a non-empty array";
      return result;
    }
    if (arguments["waypoints"].size() > 8) {
      result.error = "too many waypoints";
      return result;
    }
    for (const auto & waypoint : arguments["waypoints"]) {
      if (!is_supported_navigation_place(waypoint)) {
        result.error = "unsupported navigation waypoint";
        return result;
      }
    }
    if (has_loops) {
      if (!arguments["number_of_loops"].is_number_integer()) {
        result.error = "number_of_loops must be an integer";
        return result;
      }
      const int loops = arguments["number_of_loops"].get<int>();
      arguments["number_of_loops"] = static_cast<int>(clamp(loops, 1, 3));
    } else {
      arguments["number_of_loops"] = 1;
    }
  } else if (name == "cancel_navigation") {
    if (!require_exact_keys(arguments, {}, result.error)) {
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
