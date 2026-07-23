#pragma once

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include <embodied_agent_interfaces/msg/robot_command.hpp>

#include "embodied_simulation/action_execution.hpp"
#include "embodied_simulation/simulation_controller.hpp"

namespace embodied_simulation
{

using StopClock = std::chrono::steady_clock;
using StopTimePoint = StopClock::time_point;

enum class StopExecutionState
{
  kStopping,
  kQuiesced,
  kFailed,
  kTimedOut,
};

struct StopExecutionUpdate
{
  StopExecutionState state{StopExecutionState::kQuiesced};
  std::string detail{"executor_quiesced"};

  bool terminal() const {return state != StopExecutionState::kStopping;}
  bool succeeded() const {return state == StopExecutionState::kQuiesced;}
};

inline ActionExecutionUpdate stop_as_action_update(const StopExecutionUpdate & update)
{
  switch (update.state) {
    case StopExecutionState::kStopping:
      return {ActionExecutionState::kRunning, 0.0};
    case StopExecutionState::kQuiesced:
      return {ActionExecutionState::kSucceeded, 1.0};
    case StopExecutionState::kTimedOut:
      return {ActionExecutionState::kTimedOut, 1.0};
    case StopExecutionState::kFailed:
      return {ActionExecutionState::kBlocked, 1.0};
  }
  return {ActionExecutionState::kBlocked, 1.0};
}

class RobotExecutor
{
public:
  virtual ~RobotExecutor() = default;

  // 插件契约：execute/step/request_stop 均不得长时间阻塞 ROS executor。
  // 对本地速度执行器，request_stop 可同步归零；对 Nav2 等外部 Action Server，
  // request_stop 只发起取消，调用方必须持续 poll_stop，直到收到 terminal result。
  // Gazebo 与 mock 两个 adapter 共同证明这个 pluginlib seam 是真实可替换点。
  virtual void configure(const ControllerConfig & config) = 0;
  virtual bool execute(
    const embodied_agent_interfaces::msg::RobotCommand & command,
    double now_s) = 0;
  virtual void request_stop(StopTimePoint requested_at) = 0;
  virtual StopExecutionUpdate poll_stop(StopTimePoint now) = 0;
  virtual bool is_quiesced() const = 0;
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

inline bool should_publish_executor_cmd_vel(
  bool plugin_publishes_cmd_vel,
  bool action_was_active,
  bool action_stopped_this_tick,
  const std::string & executor_mode)
{
  // /cmd_vel 没有天然的多发布者仲裁。手动 executor 空闲时持续发零会与
  // Nav2 controller 争用底盘；只有持有 Action、刚完成需归零，或处于持续
  // 自主模式时，它才拥有速度写权限。
  return plugin_publishes_cmd_vel &&
         (action_was_active || action_stopped_this_tick ||
         executor_mode != "manual");
}

}  // namespace embodied_simulation
