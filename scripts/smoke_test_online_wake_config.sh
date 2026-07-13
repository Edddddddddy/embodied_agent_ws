#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + $$ % 100))}"

LOG_FILE="$(mktemp)"
setsid ros2 launch embodied_online_agent online_agent.launch.py \
  mode:=mock microphone_enabled:=false capture_enabled:=false \
  speaker_enabled:=false hardware_enabled:=false wake_word_enabled:=false \
  >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

if ! python3 "$WORKSPACE/scripts/activate_lifecycle_node.py" online_agent \
    --target-state active --wait-only --timeout 15; then
  echo "FAIL: online Agent did not reach the active lifecycle state" >&2
  cat "$LOG_FILE" >&2
  exit 1
fi
# 直接使用有界 rclpy 参数 client，不让 ros2cli daemon 的 discovery 时序污染测试结论。
if ! VALUE="$(python3 "$WORKSPACE/scripts/ros_parameter_check.py" \
    /online_agent wake_word_enabled --expected false --timeout 15)"; then
  echo "FAIL: online wake_word_enabled launch override was not applied" >&2
  cat "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: online wake_word_enabled=false reached the Agent node ($VALUE)"
