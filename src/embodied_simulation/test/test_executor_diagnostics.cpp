#include <map>
#include <string>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <gtest/gtest.h>

#include "embodied_simulation/executor_diagnostics.hpp"

namespace embodied_simulation
{
namespace
{

std::map<std::string, std::string> values_of(
  const diagnostic_msgs::msg::DiagnosticStatus & status)
{
  std::map<std::string, std::string> values;
  for (const auto & item : status.values) {
    values[item.key] = item.value;
  }
  return values;
}

ExecutorDiagnosticSnapshot ready_snapshot()
{
  return {
    "/robot1/simulation_control", "active",
    "embodied_simulation/MockRobotExecutor", "mock", "manual", "mock_idle",
    false, false, false};
}

}  // namespace

TEST(ExecutorDiagnosticsTest, ReportsStableFieldsForReadyExecutor)
{
  const auto status = make_executor_diagnostic(ready_snapshot());
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::OK);
  EXPECT_EQ(status.name, "/robot1/simulation_control");
  EXPECT_EQ(status.hardware_id, "mock");
  EXPECT_EQ(status.message, "ready");
  const auto values = values_of(status);
  EXPECT_EQ(values.at("lifecycle_state"), "active");
  EXPECT_EQ(values.at("executor_plugin"), "embodied_simulation/MockRobotExecutor");
  EXPECT_EQ(values.at("active_action"), "false");
}

TEST(ExecutorDiagnosticsTest, PrioritizesSafetyStopOverStaleSensor)
{
  auto snapshot = ready_snapshot();
  snapshot.executor_backend = "simulation";
  snapshot.sensor_stale = true;
  snapshot.safety_stopped = true;
  snapshot.reason = "emergency_obstacle";
  const auto status = make_executor_diagnostic(snapshot);
  EXPECT_EQ(status.level, diagnostic_msgs::msg::DiagnosticStatus::ERROR);
  EXPECT_EQ(status.message, "emergency_obstacle");
}

TEST(ExecutorDiagnosticsTest, WarnsOnlySimulationBackendAboutStaleScan)
{
  auto simulation = ready_snapshot();
  simulation.executor_backend = "simulation";
  simulation.sensor_stale = true;
  simulation.reason = "scan_timeout";
  EXPECT_EQ(
    make_executor_diagnostic(simulation).level,
    diagnostic_msgs::msg::DiagnosticStatus::WARN);

  auto mock = simulation;
  mock.executor_backend = "mock";
  EXPECT_EQ(
    make_executor_diagnostic(mock).level,
    diagnostic_msgs::msg::DiagnosticStatus::OK);
}

}  // namespace embodied_simulation
