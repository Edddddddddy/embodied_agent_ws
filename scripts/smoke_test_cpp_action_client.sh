#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((170 + $$ % 80))}"

LOG_FILE="$(mktemp)"
CLIENT_LOG="$(mktemp)"

setsid ros2 launch embodied_simulation simulation_control.launch.py \
  use_typed_actions:=false \
  executor_plugin:=embodied_simulation/MockRobotExecutor \
  action_timeout_s:=2.0 \
  >"$LOG_FILE" 2>&1 &
SERVER_PID=$!

cleanup() {
  kill -TERM -- "-$SERVER_PID" 2>/dev/null || true
  kill -KILL -- "-$SERVER_PID" 2>/dev/null || true
  wait "$SERVER_PID" 2>/dev/null || true
  rm -f "$LOG_FILE" "$CLIENT_LOG"
}
trap cleanup EXIT

for _ in $(seq 1 80); do
  if ros2 action list 2>/dev/null | grep -qx "/robot/execute_command"; then
    break
  fi
  sleep 0.1
done

if ! ros2 action list 2>/dev/null | grep -qx "/robot/execute_command"; then
  echo "typed action server did not appear" >&2
  cat "$LOG_FILE" >&2
  exit 1
fi

if ! timeout 20 ros2 run embodied_agent_cpp typed_action_demo_client move 0.10 0.20 \
  >"$CLIENT_LOG" 2>&1; then
  echo "typed_action_demo_client failed" >&2
  cat "$CLIENT_LOG" >&2
  cat "$LOG_FILE" >&2
  exit 1
fi

grep -q "success=true" "$CLIENT_LOG" || {
  echo "typed_action_demo_client did not report success" >&2
  cat "$CLIENT_LOG" >&2
  cat "$LOG_FILE" >&2
  exit 1
}

echo "PASS: C++ typed Action demo client sends command and receives result"
