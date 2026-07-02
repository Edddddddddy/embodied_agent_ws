#include "embodied_simulation/robot_executor.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace embodied_simulation
{

using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

class GazeboRobotExecutor : public RobotExecutor
{
public:
  void configure(const ControllerConfig & config) override
  {
    controller_ = std::make_unique<SimulationController>(config);
  }

  bool execute(const RobotCommand & command, double now_s) override
  {
    if (!controller_) {
      return false;
    }
    if (command.action_type == RobotCommand::MOVE) {
      controller_->set_manual_command(
        command.linear_x, command.angular_z, command.duration_s, now_s);
      return true;
    }
    if (command.action_type == RobotCommand::TURN) {
      controller_->set_manual_command(
        0.0, command.angular_z, command.duration_s, now_s);
      return true;
    }
    if (command.action_type == RobotCommand::STOP) {
      controller_->stop();
      return true;
    }
    if (command.action_type == RobotCommand::SET_MODE) {
      return controller_->set_mode(command.mode);
    }
    if (command.action_type == RobotCommand::WAVE ||
      command.action_type == RobotCommand::SET_LED)
    {
      controller_->stop();
      return true;
    }
    return false;
  }

  void stop() override
  {
    if (controller_) {
      controller_->stop();
    }
  }

  void update_scan(
    const std::vector<float> & ranges,
    double angle_min,
    double angle_increment,
    double range_min,
    double range_max,
    double now_s) override
  {
    if (controller_) {
      controller_->update_scan(
        ranges, angle_min, angle_increment, range_min, range_max, now_s);
    }
  }

  ControllerOutput step(double now_s) override
  {
    return controller_ ? controller_->step(now_s) : ControllerOutput{};
  }

  std::string mode_name() const override
  {
    return controller_ ? SimulationController::mode_name(controller_->mode()) : "manual";
  }

  std::string backend_name() const override {return "simulation";}

private:
  std::unique_ptr<SimulationController> controller_;
};

class MockRobotExecutor : public RobotExecutor
{
public:
  void configure(const ControllerConfig & config) override
  {
    config_ = config;
    stop();
  }

  bool execute(const RobotCommand & command, double now_s) override
  {
    if (command.action_type == RobotCommand::MOVE) {
      mode_ = ControlMode::kManual;
      velocity_.linear_x = std::clamp(
        command.linear_x, -config_.max_linear_speed, config_.max_linear_speed);
      velocity_.angular_z = std::clamp(
        command.angular_z, -config_.max_angular_speed, config_.max_angular_speed);
      active_until_s_ = now_s + std::clamp(command.duration_s, 0.0, 10.0);
      return true;
    }
    if (command.action_type == RobotCommand::TURN) {
      mode_ = ControlMode::kManual;
      velocity_.linear_x = 0.0;
      velocity_.angular_z = std::clamp(
        command.angular_z, -config_.max_angular_speed, config_.max_angular_speed);
      active_until_s_ = now_s + std::clamp(command.duration_s, 0.0, 10.0);
      return true;
    }
    if (command.action_type == RobotCommand::STOP) {
      stop();
      return true;
    }
    if (command.action_type == RobotCommand::SET_MODE) {
      if (command.mode == "manual") {
        mode_ = ControlMode::kManual;
      } else if (command.mode == "obstacle_avoidance") {
        mode_ = ControlMode::kObstacleAvoidance;
      } else if (command.mode == "wall_following") {
        mode_ = ControlMode::kWallFollowing;
      } else {
        return false;
      }
      velocity_ = {};
      return true;
    }
    if (command.action_type == RobotCommand::WAVE ||
      command.action_type == RobotCommand::SET_LED)
    {
      velocity_ = {};
      active_until_s_ = now_s;
      return true;
    }
    return false;
  }

  void stop() override
  {
    mode_ = ControlMode::kManual;
    velocity_ = {};
    active_until_s_ = 0.0;
  }

  void update_scan(
    const std::vector<float> &,
    double, double, double, double, double) override {}

  ControllerOutput step(double now_s) override
  {
    if (now_s > active_until_s_) {
      velocity_ = {};
    }
    ControllerOutput output;
    output.velocity = velocity_;
    output.mode = mode_;
    output.sensor_stale = false;
    output.front_distance = std::numeric_limits<double>::infinity();
    output.right_distance = std::numeric_limits<double>::infinity();
    output.reason = (velocity_.linear_x != 0.0 || velocity_.angular_z != 0.0) ?
      "mock_execution" : "mock_idle";
    return output;
  }

  std::string mode_name() const override
  {
    return SimulationController::mode_name(mode_);
  }

  std::string backend_name() const override {return "mock";}

private:
  ControllerConfig config_;
  ControlMode mode_{ControlMode::kManual};
  VelocityCommand velocity_;
  double active_until_s_{0.0};
};

}  // namespace embodied_simulation

PLUGINLIB_EXPORT_CLASS(
  embodied_simulation::GazeboRobotExecutor,
  embodied_simulation::RobotExecutor)
PLUGINLIB_EXPORT_CLASS(
  embodied_simulation::MockRobotExecutor,
  embodied_simulation::RobotExecutor)
