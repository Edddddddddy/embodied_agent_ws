#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"

LOG_FILE="$(mktemp)"
ros2 launch embodied_simulation simulation_control.launch.py \
  use_typed_actions:=false >"$LOG_FILE" 2>&1 &
CONTROL_PID=$!
ros2 run embodied_agent_cpp action_guard >>"$LOG_FILE" 2>&1 &
GUARD_PID=$!
cleanup() {
  kill "$CONTROL_PID" "$GUARD_PID" 2>/dev/null || true
  wait "$CONTROL_PID" "$GUARD_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

if ! python "$WORKSPACE/scripts/test_simulation_pipeline.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: action guard -> lidar safety/mode controller -> cmd_vel"
