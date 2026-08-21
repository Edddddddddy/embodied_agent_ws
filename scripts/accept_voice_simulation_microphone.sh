#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODE="${1:-offline}"
TIMEOUT="${VOICE_ACCEPT_TIMEOUT:-60}"
WAKE_WORD_ENABLED="${WAKE_WORD_ENABLED:-false}"
SPEAKER_ENABLED="${SPEAKER_ENABLED:-false}"
GUI_ENABLED="${GUI_ENABLED:-true}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + $$ % 100))}"

if [[ "$MODE" != "offline" && "$MODE" != "online" ]]; then
  echo "Usage: $0 {offline|online}" >&2
  exit 2
fi
if ! pactl list short sources 2>/dev/null | grep -q .; then
  echo "FAIL: WSL 中没有可用麦克风 source；请先检查 WSLg 音频权限。" >&2
  exit 1
fi

SERVER_PID=""
LAUNCH_PID=""
SERVER_LOG="$(mktemp)"
LAUNCH_LOG="$(mktemp)"
cleanup() {
  [[ -z "$LAUNCH_PID" ]] || kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  if [[ -n "$LAUNCH_PID" ]]; then
    for _ in $(seq 1 20); do
      kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  if [[ -n "$SERVER_PID" ]]; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
  rm -f "$SERVER_LOG" "$LAUNCH_LOG"
}
trap cleanup EXIT INT TERM

if [[ "$MODE" == "offline" ]] && \
  ! curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1
then
  bash "$WORKSPACE/scripts/start_llama_server.sh" >"$SERVER_LOG" 2>&1 &
  SERVER_PID=$!
  for _ in $(seq 1 30); do
    if curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1; then break; fi
    sleep 1
  done
  curl -fsS http://127.0.0.1:8080/health >/dev/null || {
    cat "$SERVER_LOG" >&2
    exit 1
  }
fi

setsid ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  gui:="$GUI_ENABLED" rviz:=false launch_agent:=true agent_type:="$MODE" \
  provider_mode:="$MODE" microphone_enabled:=true capture_enabled:=true \
  speaker_enabled:="$SPEAKER_ENABLED" wake_word_enabled:="$WAKE_WORD_ENABLED" \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

PROBE_ARGS=(--timeout "$TIMEOUT")
if [[ "$WAKE_WORD_ENABLED" == "true" ]]; then
  PROBE_ARGS+=(--wake-word)
fi
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID，验收模式=$MODE"
if ! python "$WORKSPACE/scripts/accept_voice_simulation_microphone.py" \
  "${PROBE_ARGS[@]}"
then
  echo "--- launch log ---" >&2
  tail -120 "$LAUNCH_LOG" >&2
  echo "--- model log ---" >&2
  tail -60 "$SERVER_LOG" >&2
  exit 1
fi
