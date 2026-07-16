#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((160 + $$ % 40))}"

LOG_FILE="$(mktemp)"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    kill -KILL -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

setsid ros2 run embodied_agent_cpp action_guard >>"$LOG_FILE" 2>&1 &
PIDS+=("$!")
setsid ros2 run embodied_simulation simulation_control_node \
  --ros-args -p action_timeout_s:=3.0 >>"$LOG_FILE" 2>&1 &
PIDS+=("$!")

if ! timeout 25 bash "$WORKSPACE/tests/integration/run_probe.sh" "$WORKSPACE/tests/integration/control/test_lifecycle_pipeline.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
