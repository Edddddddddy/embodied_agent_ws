#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((180 + $$ % 40))}"

LOG_FILE="$(mktemp)"
setsid ros2 run embodied_voice_frontend keyword_wake --ros-args \
  -p mode:=mock_text \
  -p provider_name:=mock_kws \
  >"$LOG_FILE" 2>&1 &
KWS_PID=$!

cleanup() {
  kill -TERM -- "-$KWS_PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$KWS_PID" 2>/dev/null || true
  wait "$KWS_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

if ! timeout 15 bash "$WORKSPACE/tools/acceptance/run_probe.sh" "$WORKSPACE/tests/integration/voice/test_keyword_wake_sidecar.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi

echo "PASS: keyword wake sidecar publishes external wake_event_input"
