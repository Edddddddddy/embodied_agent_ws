#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
REPORT="${OFFLINE_VOICE_E2E_REPORT:-$WORKSPACE/logs/offline_voice_e2e_report.json}"
BASE_URL="${LLAMA_BASE_URL:-http://${LLAMA_HOST:-127.0.0.1}:${LLAMA_PORT:-8080}}"
SERVER_LOG="$(mktemp)"
LAUNCH_LOG="$(mktemp)"
MEMORY_DIR="$(mktemp -d)"
SERVER_PID=""
if ! curl -fsS "$BASE_URL/health" >/dev/null 2>&1; then
  bash "$WORKSPACE/scripts/start_llama_server.sh" >"$SERVER_LOG" 2>&1 &
  SERVER_PID=$!
fi
LAUNCH_PID=""
cleanup() {
  [[ -z "$LAUNCH_PID" ]] || kill "$LAUNCH_PID" 2>/dev/null || true
  [[ -z "$SERVER_PID" ]] || kill "$SERVER_PID" 2>/dev/null || true
  [[ -z "$LAUNCH_PID" ]] || wait "$LAUNCH_PID" 2>/dev/null || true
  [[ -z "$SERVER_PID" ]] || wait "$SERVER_PID" 2>/dev/null || true
  rm -f "$SERVER_LOG" "$LAUNCH_LOG"
  rm -rf "$MEMORY_DIR"
}
trap cleanup EXIT
for _ in $(seq 1 30); do
  if curl -fsS "$BASE_URL/health" >/dev/null 2>&1; then break; fi
  sleep 1
done
curl -fsS "$BASE_URL/health" >/dev/null || { cat "$SERVER_LOG"; exit 1; }
if [[ -n "$SERVER_PID" ]] && ! kill -0 "$SERVER_PID" 2>/dev/null; then
  echo "FAIL: newly started llama-server exited before Agent launch" >&2
  cat "$SERVER_LOG" >&2
  exit 1
fi
ros2 launch embodied_offline_agent offline_agent.launch.py \
  mode:=offline microphone_enabled:=true capture_enabled:=false \
  wake_word_enabled:=false speaker_enabled:=false hardware_backend:=mock \
  memory_path:="$MEMORY_DIR/conversation.json" \
  user_memory_dir:="$MEMORY_DIR/users" \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
if ! python3 "$WORKSPACE/scripts/activate_lifecycle_node.py" offline_agent \
    --target-state active --wait-only --timeout 60; then
  echo "FAIL: offline Agent did not finish runtime warmup within 60s" >&2
  cat "$LAUNCH_LOG" >&2
  cat "$SERVER_LOG" >&2
  exit 1
fi
python "$WORKSPACE/tests/integration/test_offline_voice_e2e.py" --output "$REPORT" || {
  cat "$LAUNCH_LOG"; cat "$SERVER_LOG"; exit 1;
}
echo "PASS: real ZipFormer ASR -> llama.cpp -> Sherpa-TTS -> hardware mock; report=$REPORT"
