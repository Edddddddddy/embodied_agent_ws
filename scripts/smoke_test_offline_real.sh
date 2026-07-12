#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
SERVER_LOG="$(mktemp)"
LAUNCH_LOG="$(mktemp)"
ACK_LOG="$(mktemp)"
METRICS_LOG="$(mktemp)"
bash "$WORKSPACE/scripts/start_llama_server.sh" >"$SERVER_LOG" 2>&1 &
SERVER_PID=$!
LAUNCH_PID=""
cleanup() {
  [[ -z "$LAUNCH_PID" ]] || kill "$LAUNCH_PID" 2>/dev/null || true
  kill "$SERVER_PID" 2>/dev/null || true
  [[ -z "$LAUNCH_PID" ]] || wait "$LAUNCH_PID" 2>/dev/null || true
  wait "$SERVER_PID" 2>/dev/null || true
  rm -f "$SERVER_LOG" "$LAUNCH_LOG" "$ACK_LOG" "$METRICS_LOG"
}
trap cleanup EXIT
for _ in $(seq 1 30); do
  if curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1; then break; fi
  sleep 1
done
curl -fsS http://127.0.0.1:8080/health >/dev/null || { cat "$SERVER_LOG"; exit 1; }

ros2 launch embodied_offline_agent offline_agent.launch.py \
  mode:=offline microphone_enabled:=false speaker_enabled:=false hardware_backend:=mock \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
sleep 5
for _ in $(seq 1 20); do
  if ros2 topic info /agent/text_input 2>/dev/null | grep -Eq 'Subscription count: [1-9]'; then
    break
  fi
  sleep 0.5
done
ros2 topic info /agent/text_input | grep -Eq 'Subscription count: [1-9]' || {
  cat "$LAUNCH_LOG"; exit 1;
}
ros2 topic pub --once /agent/clear_memory std_msgs/msg/Empty "{}" >/dev/null
timeout 60 ros2 topic echo --once /robot/action_ack embodied_agent_interfaces/msg/RobotActionAck >"$ACK_LOG" &
ACK_PID=$!
timeout 60 ros2 topic echo --once /offline_agent/metrics std_msgs/msg/String >"$METRICS_LOG" &
METRICS_PID=$!
sleep 1
timeout 10 ros2 topic pub -r 2 --times 3 /agent/text_input std_msgs/msg/String \
  "{data: '小智，向前走一秒'}" >/dev/null
wait "$ACK_PID" || { cat "$LAUNCH_LOG"; cat "$SERVER_LOG"; exit 1; }
wait "$METRICS_PID" || { cat "$LAUNCH_LOG"; cat "$SERVER_LOG"; exit 1; }
grep -q '^action: move' "$ACK_LOG"
grep -q 'end_to_first_audio_ms' "$METRICS_LOG"
echo "PASS: real offline llama.cpp -> Sherpa-TTS -> action -> hardware mock"
cat "$METRICS_LOG"
