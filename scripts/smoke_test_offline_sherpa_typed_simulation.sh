#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/lifecycle_utils.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((200 + $$ % 30))}"

SERVER_LOG="$(mktemp)"
CONTROL_LOG="$(mktemp)"
AGENT_LOG="$(mktemp)"
SERVER_PID=""
CONTROL_PID=""
AGENT_PID=""

cleanup() {
  set +e
  [[ -z "$AGENT_PID" ]] || kill -TERM -- "-$AGENT_PID" 2>/dev/null || true
  [[ -z "$CONTROL_PID" ]] || kill -TERM -- "-$CONTROL_PID" 2>/dev/null || true
  [[ -z "$SERVER_PID" ]] || kill "$SERVER_PID" 2>/dev/null || true
  sleep 0.3
  [[ -z "$AGENT_PID" ]] || kill -KILL -- "-$AGENT_PID" 2>/dev/null || true
  [[ -z "$CONTROL_PID" ]] || kill -KILL -- "-$CONTROL_PID" 2>/dev/null || true
  [[ -z "$AGENT_PID" ]] || wait "$AGENT_PID" 2>/dev/null || true
  [[ -z "$CONTROL_PID" ]] || wait "$CONTROL_PID" 2>/dev/null || true
  [[ -z "$SERVER_PID" ]] || wait "$SERVER_PID" 2>/dev/null || true
  rm -f "$SERVER_LOG" "$CONTROL_LOG" "$AGENT_LOG"
  return 0
}
trap cleanup EXIT

bash "$WORKSPACE/scripts/start_llama_server.sh" >"$SERVER_LOG" 2>&1 &
SERVER_PID=$!
for _ in $(seq 1 45); do
  if curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1; then
    break
  fi
  sleep 1
done
curl -fsS http://127.0.0.1:8080/health >/dev/null || {
  cat "$SERVER_LOG" >&2
  exit 1
}

setsid ros2 launch embodied_simulation simulation_control.launch.py \
  use_typed_actions:=true \
  use_behavior_tree:=true \
  executor_plugin:=embodied_simulation/MockRobotExecutor \
  >"$CONTROL_LOG" 2>&1 &
CONTROL_PID=$!

setsid ros2 launch embodied_offline_agent offline_agent.launch.py \
  mode:=offline \
  microphone_enabled:=true \
  capture_enabled:=false \
  speaker_enabled:=false \
  wake_word_enabled:=false \
  hardware_enabled:=false \
  lifecycle_autostart:=true \
  >"$AGENT_LOG" 2>&1 &
AGENT_PID=$!

if ! wait_for_topic_subscribers /audio/clean_pcm 1 180; then
  cat "$AGENT_LOG" >&2
  exit 1
fi
if ! wait_for_topic_subscribers /agent/action_candidate 1 180; then
  cat "$AGENT_LOG" "$CONTROL_LOG" >&2
  exit 1
fi
if ! wait_for_topic_subscribers /robot/action_command_typed 1 180; then
  cat "$AGENT_LOG" "$CONTROL_LOG" >&2
  exit 1
fi

if ! timeout 120 python3 "$WORKSPACE/tests/integration/test_offline_sherpa_typed_simulation.py"; then
  cat "$AGENT_LOG" >&2
  cat "$CONTROL_LOG" >&2
  cat "$SERVER_LOG" >&2
  exit 1
fi

echo "PASS: Sherpa-ONNX ASR -> llama.cpp/Sherpa-TTS -> ActionGuard -> typed Action -> simulation cmd_vel"
exit 0
