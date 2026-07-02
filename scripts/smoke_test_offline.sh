#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((40 + $$ % 20))}"
LOG="$(mktemp)"
setsid ros2 launch embodied_offline_agent offline_agent.launch.py mode:=mock >"$LOG" 2>&1 &
PID=$!
cleanup() {
  kill -TERM -- "-$PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$PID" 2>/dev/null || true
  wait "$PID" 2>/dev/null || true
  rm -f "$LOG"
}
trap cleanup EXIT
sleep 3
ACTION_LOG="$(mktemp)"
timeout 10 ros2 topic echo --once /robot/action_command std_msgs/msg/String >"$ACTION_LOG" &
ECHO_PID=$!
sleep 1
ros2 topic pub --once /agent/text_input std_msgs/msg/String "{data: '小智，向前走一秒'}" >/dev/null
wait "$ECHO_PID" 2>/dev/null || { cat "$LOG"; rm -f "$ACTION_LOG"; exit 1; }
grep -q '"name":"move"' "$ACTION_LOG"
rm -f "$ACTION_LOG"
echo "PASS: offline mock ROS/action pipeline"
