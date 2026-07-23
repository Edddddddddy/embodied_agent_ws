#include "embodied_simulation/robot_executor.hpp"

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
      // MOVE 同时支持 linear_x 与 angular_z，因此“绕圈/画圆”无需新增接口字段。
      controller_->set_manual_command(
        command.linear_x, command.angular_z, command.duration_s, now_s);
      return true;
    }
    if (command.action_type == RobotCommand::TURN) {
      controller_->set_manual_command(0.0, command.angular_z, command.duration_s, now_s);
      return true;
    }
    if (command.action_type == RobotCommand::STOP) {
      request_stop(StopClock::now());
      return true;
    }
    if (command.action_type == RobotCommand::NAVIGATE_TO) {
      // 非 Nav2 profile 下保留可观测的替代运动，便于无地图环境验收语音导航链路。
      controller_->set_manual_command(0.14, 0.0, command.duration_s, now_s);
      return true;
    }
    if (command.action_type == RobotCommand::FOLLOW_WAYPOINTS) {
      controller_->set_manual_command(0.12, 0.25, command.duration_s, now_s);
      return true;
    }
    if (command.action_type == RobotCommand::CANCEL_NAVIGATION) {
      request_stop(StopClock::now());
      return true;
    }
    if (command.action_type == RobotCommand::SET_MODE) {
      return controller_->set_mode(command.mode);
    }
    if (command.action_type == RobotCommand::WAVE ||
      command.action_type == RobotCommand::SET_LED)
    {
      // 仿真没有机械臂/灯带；附件动作只产生 ACK，并确保底盘停止。
      controller_->stop();
      return true;
    }
    return false;
  }

  void request_stop(StopTimePoint) override
  {
    if (controller_) {
      controller_->stop();
    }
  }

  StopExecutionUpdate poll_stop(StopTimePoint) override
  {
    return {StopExecutionState::kQuiesced, "gazebo:velocity_zero"};
  }

  bool is_quiesced() const override {return true;}

  void update_scan(
    const std::vector<float> & ranges, double angle_min, double angle_increment,
    double range_min, double range_max, double now_s) override
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

}  // namespace embodied_simulation

PLUGINLIB_EXPORT_CLASS(
  embodied_simulation::GazeboRobotExecutor,
  embodied_simulation::RobotExecutor)
