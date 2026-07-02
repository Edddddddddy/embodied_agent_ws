#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((120 + $$ % 80))}"

LOG_FILE="$(mktemp)"
setsid ros2 launch embodied_simulation simulation_control.launch.py \
  use_typed_actions:=true >"$LOG_FILE" 2>&1 &
CONTROL_PID=$!
setsid ros2 run embodied_agent_cpp action_guard >>"$LOG_FILE" 2>&1 &
GUARD_PID=$!
cleanup() {
  kill -TERM -- "-$CONTROL_PID" "-$GUARD_PID" 2>/dev/null || true
  kill -KILL -- "-$CONTROL_PID" "-$GUARD_PID" 2>/dev/null || true
  wait "$CONTROL_PID" "$GUARD_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

if ! timeout 30 python "$WORKSPACE/scripts/test_simulation_pipeline.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: JSON candidate -> typed command -> ROS Action -> simulation controller"
