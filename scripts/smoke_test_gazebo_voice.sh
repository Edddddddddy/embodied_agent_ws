#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + $$ % 100))}"
USE_TYPED_ACTIONS="${USE_TYPED_ACTIONS:-false}"

SERVER_LOG="$(mktemp)"
LAUNCH_LOG="$(mktemp)"
bash "$WORKSPACE/scripts/start_llama_server.sh" >"$SERVER_LOG" 2>&1 &
SERVER_PID=$!
LAUNCH_PID=""
cleanup() {
  [[ -z "$LAUNCH_PID" ]] || kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  if [[ -n "$LAUNCH_PID" ]]; then
    for _ in $(seq 1 20); do
      kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
      sleep 0.1
    done
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  fi
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
curl -fsS http://127.0.0.1:8080/health >/dev/null || {
  cat "$SERVER_LOG" >&2
  exit 1
}

setsid ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  gui:=false rviz:=false launch_agent:=true agent_type:=offline \
  provider_mode:=offline microphone_enabled:=true capture_enabled:=false \
  speaker_enabled:=false wake_word_enabled:=false \
  use_typed_actions:="$USE_TYPED_ACTIONS" >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

if ! REQUIRE_TYPED_ACTION_RESULT="$USE_TYPED_ACTIONS" \
  bash "$WORKSPACE/tools/acceptance/run_probe.sh" "$WORKSPACE/tests/integration/voice/test_gazebo_voice.py"; then
  cat "$LAUNCH_LOG" >&2
  cat "$SERVER_LOG" >&2
  exit 1
fi
echo "PASS: speech -> ZipFormer -> llama.cpp -> action guard -> Gazebo TurtleBot3 (typed=$USE_TYPED_ACTIONS)"
