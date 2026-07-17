#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/lifecycle_utils.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((180 + $$ % 20))}"

LOG_FILE="$(mktemp)"
setsid ros2 launch embodied_simulation simulation_control.launch.py \
  use_composition:=true use_typed_actions:=true \
  executor_plugin:=embodied_simulation/MockRobotExecutor \
  >"$LOG_FILE" 2>&1 &
CONTROL_PID=$!
setsid ros2 run embodied_agent_cpp action_guard >>"$LOG_FILE" 2>&1 &
GUARD_PID=$!
cleanup() {
  kill -TERM -- "-$CONTROL_PID" "-$GUARD_PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$CONTROL_PID" "-$GUARD_PID" 2>/dev/null || true
  wait "$CONTROL_PID" "$GUARD_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

activate_lifecycle_node action_guard
COMPOSED=false
for _ in $(seq 1 80); do
  if ros2 node list 2>/dev/null | grep -qx /simulation_container && \
    ros2 component list 2>/dev/null | grep -q /simulation_control; then
    COMPOSED=true
    break
  fi
  sleep 0.1
done
if [[ "$COMPOSED" != true ]]; then
  echo "FAIL: SimulationControl was not loaded in /simulation_container" >&2
  cat "$LOG_FILE" >&2
  exit 1
fi
if ! timeout 20 bash "$WORKSPACE/tools/acceptance/run_probe.sh" "$WORKSPACE/tests/integration/control/test_mock_executor_pipeline.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: component_container_mt -> Lifecycle/Action/BT -> mock executor"
