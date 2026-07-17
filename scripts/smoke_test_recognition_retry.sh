#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + $$ % 100))}"

LOG_FILE="$(mktemp)"
setsid ros2 launch embodied_online_agent online_agent.launch.py \
  mode:=mock microphone_enabled:=false capture_enabled:=false \
  speaker_enabled:=false hardware_enabled:=false wake_word_enabled:=true \
  >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

if ! timeout 20 bash "$WORKSPACE/tools/acceptance/run_probe.sh" \
  "$WORKSPACE/tools/acceptance/probes/voice/recognition_retry.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
