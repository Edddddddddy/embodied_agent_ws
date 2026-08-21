#pragma once

#include <string>

#include "embodied_simulation/simulation_controller.hpp"

namespace embodied_simulation
{

struct ConfigurationValidation
{
  bool valid{false};
  std::string error;
};

ConfigurationValidation validate_node_configuration(
  const ControllerConfig & controller,
  double control_rate_hz,
  double action_timeout_s,
  const std::string & executor_plugin);

}  // namespace embodied_simulation
