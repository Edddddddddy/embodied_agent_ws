#include "embodied_simulation/executor_diagnostics.hpp"

#include <string>

#include <diagnostic_msgs/msg/key_value.hpp>

namespace embodied_simulation
{
namespace
{

diagnostic_msgs::msg::KeyValue value(
  const std::string & key, const std::string & content)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = content;
  return item;
}

std::string boolean(bool enabled)
{
  return enabled ? "true" : "false";
}

}  // namespace

diagnostic_msgs::msg::DiagnosticStatus make_executor_diagnostic(
  const ExecutorDiagnosticSnapshot & snapshot)
{
  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = snapshot.node_name;
  status.hardware_id = snapshot.executor_backend;

  // 安全停车必须压过传感器超时，运维端看到 ERROR 时应优先处理真实运动风险。
  if (snapshot.safety_stopped) {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    status.message = snapshot.reason;
  } else if (
    snapshot.sensor_stale && snapshot.executor_backend == "simulation")
  {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    status.message = snapshot.reason;
  } else {
    status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    status.message = "ready";
  }

  status.values = {
    value("lifecycle_state", snapshot.lifecycle_state),
    value("executor_plugin", snapshot.executor_plugin),
    value("executor_backend", snapshot.executor_backend),
    value("control_mode", snapshot.control_mode),
    value("active_action", boolean(snapshot.action_active)),
    value("sensor_stale", boolean(snapshot.sensor_stale)),
    value("safety_stopped", boolean(snapshot.safety_stopped)),
    value("reason", snapshot.reason),
  };
  return status;
}

}  // namespace embodied_simulation
