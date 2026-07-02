#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + $$ % 100))}"

if [[ -z "${DASHSCOPE_API_KEY:-}" ]]; then
  echo "FAIL: DASHSCOPE_API_KEY is required" >&2
  exit 1
fi

LAUNCH_LOG="$(mktemp)"
setsid ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  gui:=false rviz:=false launch_agent:=true agent_type:=online \
  provider_mode:=online microphone_enabled:=true capture_enabled:=false \
  speaker_enabled:=false wake_word_enabled:=false use_typed_actions:=true \
  >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  rm -f "$LAUNCH_LOG"
}
trap cleanup EXIT

if ! REQUIRE_TYPED_ACTION_RESULT=true \
  python "$WORKSPACE/scripts/test_gazebo_voice.py"; then
  cat "$LAUNCH_LOG" >&2
  exit 1
fi
echo "PASS: speech -> online ASR/LLM -> action guard -> Gazebo TurtleBot3"
