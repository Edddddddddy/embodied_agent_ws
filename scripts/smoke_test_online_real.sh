#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
if [[ -z "${DASHSCOPE_API_KEY:-}" ]]; then
  echo "DASHSCOPE_API_KEY is required" >&2
  exit 2
fi
LAUNCH_LOG="$(mktemp)"
ACK_LOG="$(mktemp)"
METRICS_LOG="$(mktemp)"
ros2 launch embodied_online_agent online_agent.launch.py \
  mode:=online microphone_enabled:=false speaker_enabled:=false hardware_backend:=mock \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill "$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  rm -f "$LAUNCH_LOG" "$ACK_LOG" "$METRICS_LOG"
}
trap cleanup EXIT
sleep 4
ros2 topic pub --once /agent/clear_memory std_msgs/msg/Empty "{}" >/dev/null
timeout 45 ros2 topic echo --once /robot/action_ack embodied_agent_interfaces/msg/RobotActionAck >"$ACK_LOG" &
ACK_PID=$!
timeout 45 ros2 topic echo --once /agent/metrics std_msgs/msg/String >"$METRICS_LOG" &
METRICS_PID=$!
sleep 1
timeout 10 ros2 topic pub --once /agent/text_input std_msgs/msg/String \
  "{data: '小智，向前走一秒'}" >/dev/null
wait "$ACK_PID" || { cat "$LAUNCH_LOG"; exit 1; }
wait "$METRICS_PID" || { cat "$LAUNCH_LOG"; exit 1; }
grep -q '^action: move' "$ACK_LOG"
grep -q 'llm_first_token_ms' "$METRICS_LOG"
echo "PASS: live online LLM -> TTS -> action guard -> hardware mock"
cat "$METRICS_LOG"
