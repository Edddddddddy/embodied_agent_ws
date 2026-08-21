#include "embodied_simulation/node_configuration.hpp"

#include <cmath>

namespace embodied_simulation
{

ConfigurationValidation validate_node_configuration(
  const ControllerConfig & controller,
  double control_rate_hz,
  double action_timeout_s,
  const std::string & executor_plugin)
{
  if (!std::isfinite(control_rate_hz) ||
    control_rate_hz <= 0.0 || control_rate_hz > 1000.0)
  {
    return {false, "control_rate_hz must be in (0, 1000]"};
  }
  if (!std::isfinite(action_timeout_s) || action_timeout_s <= 0.0) {
    return {false, "action_timeout_s must be positive"};
  }
  if (executor_plugin.empty()) {
    return {false, "executor_plugin must not be empty"};
  }
  if (controller.max_linear_speed <= 0.0 ||
    controller.max_angular_speed <= 0.0 ||
    controller.linear_acceleration <= 0.0 ||
    controller.angular_acceleration <= 0.0)
  {
    return {false, "speed and acceleration limits must be positive"};
  }
  if (controller.emergency_distance <= 0.0 ||
    controller.obstacle_distance <= controller.emergency_distance)
  {
    return {false, "obstacle_distance must exceed emergency_distance"};
  }
  if (controller.scan_timeout <= 0.0) {
    return {false, "scan_timeout must be positive"};
  }
  return {true, ""};
}

}  // namespace embodied_simulation
