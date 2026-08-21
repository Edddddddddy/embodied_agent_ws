#pragma once

#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace embodied_simulation
{

// 诊断线程只读取这个不可变快照，不直接访问正在执行动作的 RobotExecutor。
// 这既固定了 diagnostics 的字段契约，也隔离了 MultiThreadedExecutor 下的数据竞争风险。
struct ExecutorDiagnosticSnapshot
{
  std::string node_name;
  std::string lifecycle_state;
  std::string executor_plugin;
  std::string executor_backend;
  std::string control_mode;
  std::string reason;
  bool action_active{false};
  bool sensor_stale{false};
  bool safety_stopped{false};
};

diagnostic_msgs::msg::DiagnosticStatus make_executor_diagnostic(
  const ExecutorDiagnosticSnapshot & snapshot);

}  // namespace embodied_simulation
