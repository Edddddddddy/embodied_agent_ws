#pragma once

#include <optional>
#include <string>
#include <vector>

#include <embodied_agent_interfaces/msg/robot_command.hpp>

#include "embodied_simulation/action_execution.hpp"
#include "embodied_simulation/simulation_controller.hpp"

namespace embodied_simulation
{

class RobotExecutor
{
public:
  virtual ~RobotExecutor() = default;

  // 插件契约：execute/step 不得长时间阻塞 ROS executor；stop 必须幂等并立即归零。
  // Gazebo 与 mock 两个 adapter 共同证明这个 pluginlib seam 是真实可替换点。
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
  // Nav2 这类外部控制器会自己发布 /cmd_vel；此时 simulation_control 只负责发 action goal，
  // 不能再周期性发布零速度，否则会和 Nav2 controller 抢控制权。
  virtual bool publishes_cmd_vel() const {return true;}
  // 普通仿真动作由 duration_s 判定完成；Nav2 这类外部 action server 则应把真正的
  // result 回传到这里，避免“goal 刚发送就按固定时间取消”的假完成。
  virtual std::optional<ActionExecutionUpdate> external_action_update() const
  {
    return std::nullopt;
  }
  // 外部 action server（如 Nav2）可以把 server unavailable、goal rejected、
  // planner/controller aborted 等原因放在这里，SimulationControl 再透传到 feedback/result。
  virtual std::string external_action_detail() const {return "";}
  // 名称会进入 ACK 与 diagnostics，必须稳定，不能包含一次运行的随机信息。
  virtual std::string mode_name() const = 0;
  virtual std::string backend_name() const = 0;
};

}  // namespace embodied_simulation
