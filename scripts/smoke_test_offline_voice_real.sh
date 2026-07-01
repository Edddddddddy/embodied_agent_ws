#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
SERVER_LOG="$(mktemp)"
LAUNCH_LOG="$(mktemp)"
bash "$WORKSPACE/scripts/start_llama_server.sh" >"$SERVER_LOG" 2>&1 &
SERVER_PID=$!
LAUNCH_PID=""
cleanup() {
  [[ -z "$LAUNCH_PID" ]] || kill "$LAUNCH_PID" 2>/dev/null || true
  kill "$SERVER_PID" 2>/dev/null || true
  [[ -z "$LAUNCH_PID" ]] || wait "$LAUNCH_PID" 2>/dev/null || true
  wait "$SERVER_PID" 2>/dev/null || true
  rm -f "$SERVER_LOG" "$LAUNCH_LOG"
}
trap cleanup EXIT
for _ in $(seq 1 30); do
  if curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1; then break; fi
  sleep 1
done
curl -fsS http://127.0.0.1:8080/health >/dev/null || { cat "$SERVER_LOG"; exit 1; }
ros2 launch embodied_offline_agent offline_agent.launch.py \
  mode:=offline microphone_enabled:=true capture_enabled:=false \
  wake_word_enabled:=false speaker_enabled:=false hardware_backend:=mock \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
sleep 6
ros2 topic pub --once /agent/clear_memory std_msgs/msg/Empty "{}" >/dev/null
python "$WORKSPACE/scripts/test_offline_voice_e2e.py" || {
  cat "$LAUNCH_LOG"; cat "$SERVER_LOG"; exit 1;
}
echo "PASS: real ZipFormer ASR -> llama.cpp -> Sherpa-TTS -> hardware mock"
