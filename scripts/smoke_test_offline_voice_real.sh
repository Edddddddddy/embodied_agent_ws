#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
REPORT="${OFFLINE_VOICE_E2E_REPORT:-$WORKSPACE/logs/offline_voice_e2e_report.json}"
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
kill -0 "$SERVER_PID" 2>/dev/null || {
  echo "FAIL: newly started llama-server exited before Agent launch" >&2
  cat "$SERVER_LOG" >&2
  exit 1
}
ros2 launch embodied_offline_agent offline_agent.launch.py \
  mode:=offline microphone_enabled:=true capture_enabled:=false \
  wake_word_enabled:=false speaker_enabled:=false hardware_backend:=mock \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
for _ in $(seq 1 120); do
  if grep -q "offline agent ready" "$LAUNCH_LOG"; then
    break
  fi
  sleep 0.5
done
grep -q "offline agent ready" "$LAUNCH_LOG" || {
  echo "FAIL: offline Agent did not finish runtime warmup within 60s" >&2
  cat "$LAUNCH_LOG" >&2
  cat "$SERVER_LOG" >&2
  exit 1
}
ros2 topic pub --once /agent/clear_memory std_msgs/msg/Empty "{}" >/dev/null
python "$WORKSPACE/tests/integration/test_offline_voice_e2e.py" --output "$REPORT" || {
  cat "$LAUNCH_LOG"; cat "$SERVER_LOG"; exit 1;
}
echo "PASS: real ZipFormer ASR -> llama.cpp -> Sherpa-TTS -> hardware mock; report=$REPORT"
