#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((60 + $$ % 20))}"
LAUNCH_LOG="$(mktemp)"
ACK_LOG="$(mktemp)"
setsid ros2 launch embodied_agent_cpp hardware_control.launch.py backend:=mock >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
ECHO_PID=""
cleanup() {
  [[ -z "$ECHO_PID" ]] || kill "$ECHO_PID" 2>/dev/null || true
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  rm -f "$LAUNCH_LOG" "$ACK_LOG"
}
trap cleanup EXIT
sleep 3
timeout 8 ros2 topic echo /robot/action_ack std_msgs/msg/String >"$ACK_LOG" &
ECHO_PID=$!
sleep 1
ros2 topic pub --once /agent/action_candidate std_msgs/msg/String \
  "{data: '{\"name\":\"move\",\"arguments\":{\"linear_x\":0.1,\"duration_s\":0.1}}'}" >/dev/null
sleep 1
grep -q '"action":"move"' "$ACK_LOG"
grep -q '"source":"duration_elapsed"' "$ACK_LOG"
ros2 topic pub --once /robot/emergency_stop std_msgs/msg/Empty "{}" >/dev/null
sleep 1
grep -q '"source":"emergency_stop"' "$ACK_LOG"
echo "PASS: action guard -> hardware controller -> mock transport -> watchdog stop"
