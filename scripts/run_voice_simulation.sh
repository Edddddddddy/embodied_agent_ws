#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODE="${1:-offline}"
if [[ $# -gt 0 ]]; then shift; fi
source "$WORKSPACE/scripts/activate.sh"

MICROPHONE_ENABLED="${MICROPHONE_ENABLED:-true}"
SPEAKER_ENABLED="${SPEAKER_ENABLED:-true}"
COMMON_ARGS=(
  gui:=true
  rviz:=false
  microphone_enabled:="$MICROPHONE_ENABLED"
  speaker_enabled:="$SPEAKER_ENABLED"
)

case "$MODE" in
  online)
    exec ros2 launch embodied_simulation voice_turtlebot3.launch.py \
      agent_type:=online provider_mode:=online \
      "${COMMON_ARGS[@]}" "$@"
    ;;
  offline)
    SERVER_PID=""
    cleanup() {
      if [[ -n "$SERVER_PID" ]]; then
        kill "$SERVER_PID" 2>/dev/null || true
        wait "$SERVER_PID" 2>/dev/null || true
      fi
    }
    trap cleanup EXIT INT TERM
    if ! curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1; then
      bash "$WORKSPACE/scripts/start_llama_server.sh" &
      SERVER_PID=$!
      for _ in $(seq 1 30); do
        if curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1; then break; fi
        sleep 1
      done
    fi
    curl -fsS http://127.0.0.1:8080/health >/dev/null || {
      echo "llama-server did not become ready" >&2
      exit 1
    }
    ros2 launch embodied_simulation voice_turtlebot3.launch.py \
      agent_type:=offline provider_mode:=offline \
      "${COMMON_ARGS[@]}" "$@"
    ;;
  *)
    echo "Usage: $0 {offline|online} [additional launch arguments]" >&2
    exit 2
    ;;
esac
