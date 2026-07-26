#include "embodied_simulation/robot_executor.hpp"

#include <algorithm>
#include <limits>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace embodied_simulation
{

using RobotCommand = embodied_agent_interfaces::msg::RobotCommand;

class MockRobotExecutor : public RobotExecutor
{
public:
  void configure(const ControllerConfig & config) override
  {
    config_ = config;
    request_stop(StopClock::now());
  }

  bool execute(const RobotCommand & command, double now_s) override
  {
    if (command.action_type == RobotCommand::MOVE) {
      return start_motion(command.linear_x, command.angular_z, command.duration_s, now_s);
    }
    if (command.action_type == RobotCommand::TURN) {
      return start_motion(0.0, command.angular_z, command.duration_s, now_s);
    }
    if (command.action_type == RobotCommand::STOP ||
      command.action_type == RobotCommand::CANCEL_NAVIGATION)
    {
      request_stop(StopClock::now());
      return true;
    }
    if (command.action_type == RobotCommand::NAVIGATE_TO) {
      return start_motion(0.14, 0.0, command.duration_s, now_s);
    }
    if (command.action_type == RobotCommand::FOLLOW_WAYPOINTS) {
      return start_motion(0.12, 0.25, command.duration_s, now_s);
    }
    if (command.action_type == RobotCommand::SET_MODE) {
      return set_mode(command.mode);
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

  void request_stop(StopTimePoint) override
  {
    mode_ = ControlMode::kManual;
    velocity_ = {};
    active_until_s_ = 0.0;
  }

  StopExecutionUpdate poll_stop(StopTimePoint) override
  {
    return {StopExecutionState::kQuiesced, "mock:velocity_zero"};
  }

  bool is_quiesced() const override {return true;}

  void update_scan(
    const std::vector<float> &, double, double, double, double, double) override {}

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
  bool start_motion(double linear_x, double angular_z, double duration_s, double now_s)
  {
    mode_ = ControlMode::kManual;
    velocity_.linear_x = std::clamp(
      linear_x, -config_.max_linear_speed, config_.max_linear_speed);
    velocity_.angular_z = std::clamp(
      angular_z, -config_.max_angular_speed, config_.max_angular_speed);
    active_until_s_ = now_s + std::clamp(duration_s, 0.0, 10.0);
    return true;
  }

  bool set_mode(const std::string & mode)
  {
    if (mode == "manual") {
      mode_ = ControlMode::kManual;
    } else if (mode == "obstacle_avoidance") {
      mode_ = ControlMode::kObstacleAvoidance;
    } else if (mode == "wall_following") {
      mode_ = ControlMode::kWallFollowing;
    } else {
      return false;
    }
    velocity_ = {};
    return true;
  }

  ControllerConfig config_;
  ControlMode mode_{ControlMode::kManual};
  VelocityCommand velocity_;
  double active_until_s_{0.0};
};

}  // namespace embodied_simulation

PLUGINLIB_EXPORT_CLASS(
  embodied_simulation::MockRobotExecutor,
  embodied_simulation::RobotExecutor)
