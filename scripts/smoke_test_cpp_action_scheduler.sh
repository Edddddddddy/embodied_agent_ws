#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/lifecycle_utils.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((180 + $$ % 40))}"

LOG_FILE="$(mktemp)"
setsid ros2 launch embodied_simulation simulation_control.launch.py \
  use_typed_actions:=true \
  executor_plugin:=embodied_simulation/MockRobotExecutor >"$LOG_FILE" 2>&1 &
CONTROL_PID=$!
cleanup() {
  kill -TERM -- "-$CONTROL_PID" 2>/dev/null || true
  kill -KILL -- "-$CONTROL_PID" 2>/dev/null || true
  wait "$CONTROL_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

if ! activate_lifecycle_node simulation_control; then
  cat "$LOG_FILE" >&2
  exit 1
fi
if ! timeout 35 bash "$WORKSPACE/tools/acceptance/run_probe.sh" "$WORKSPACE/tools/acceptance/probes/control/cpp_action_scheduler.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: C++ scheduler FIFO -> priority cancel -> diagnostics"
