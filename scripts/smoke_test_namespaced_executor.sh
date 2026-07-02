#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((160 + $$ % 20))}"

LOG_FILE="$(mktemp)"
setsid ros2 launch embodied_simulation simulation_control.launch.py \
  namespace:=robot1 use_composition:=true use_typed_actions:=true \
  executor_plugin:=embodied_simulation/MockRobotExecutor \
  >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

if ! timeout 30 python3 "$WORKSPACE/scripts/test_namespaced_executor.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
