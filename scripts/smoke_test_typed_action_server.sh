#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((120 + $$ % 80))}"

LOG_FILE="$(mktemp)"
setsid ros2 run embodied_simulation simulation_control_node --ros-args \
  -p legacy_command_enabled:=false -p action_timeout_s:=1.0 \
  >"$LOG_FILE" 2>&1 &
SERVER_PID=$!
cleanup() {
  kill -TERM -- "-$SERVER_PID" 2>/dev/null || true
  kill -KILL -- "-$SERVER_PID" 2>/dev/null || true
  wait "$SERVER_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

if ! timeout 20 python "$WORKSPACE/scripts/test_typed_action_server.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
