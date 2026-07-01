#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + $$ % 100))}"

LOG_FILE="$(mktemp)"
FEEDBACK_FILE="$(mktemp)"
setsid ros2 launch embodied_online_agent online_agent.launch.py \
  mode:=mock microphone_enabled:=false capture_enabled:=false \
  speaker_enabled:=false hardware_enabled:=false wake_word_enabled:=true \
  >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  rm -f "$LOG_FILE" "$FEEDBACK_FILE"
}
trap cleanup EXIT

for _ in $(seq 1 50); do
  ros2 node list 2>/dev/null | grep -qx /online_agent && break
  sleep 0.1
done
timeout 8 ros2 topic echo --once /agent/recognition_feedback \
  std_msgs/msg/String >"$FEEDBACK_FILE" &
ECHO_PID=$!
sleep 1
ros2 topic pub --once /agent/text_input std_msgs/msg/String \
  "{data: '完全没听清'}" >/dev/null
wait "$ECHO_PID" || {
  echo "FAIL: no retry feedback" >&2
  cat "$LOG_FILE" >&2
  exit 1
}
grep -q 'wake_word_not_detected' "$FEEDBACK_FILE"
grep -q 'retry' "$FEEDBACK_FILE"
echo "PASS: failed recognition emitted retry feedback and Agent stayed alive"
