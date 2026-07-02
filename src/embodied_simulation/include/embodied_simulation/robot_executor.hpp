#pragma once

#include <string>
#include <vector>

#include <embodied_agent_interfaces/msg/robot_command.hpp>

#include "embodied_simulation/simulation_controller.hpp"

namespace embodied_simulation
{

class RobotExecutor
{
public:
  virtual ~RobotExecutor() = default;

  virtual void configure(const ControllerConfig & config) = 0;
  virtual bool execute(
    const embodied_agent_interfaces::msg::RobotCommand & command,
    double now_s) = 0;
  virtual void stop() = 0;
  virtual void update_scan(
    const std::vector<float> & ranges,
    double angle_min,
    double angle_increment,
    double range_min,
    double range_max,
    double now_s) = 0;
  virtual ControllerOutput step(double now_s) = 0;
  virtual std::string mode_name() const = 0;
  virtual std::string backend_name() const = 0;
};

}  // namespace embodied_simulation
