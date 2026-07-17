#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
AGENT_KIND="${1:-online}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 10))}"

if [[ "$AGENT_KIND" != "online" && "$AGENT_KIND" != "offline" ]]; then
  echo "Usage: $0 {online|offline}" >&2
  exit 2
fi

USER_MEMORY_DIR="$(mktemp -d)"
LOG_FILE="$(mktemp)"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  sleep 0.2
  for pid in "${PIDS[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  rm -rf "$USER_MEMORY_DIR" "$LOG_FILE"
}
trap cleanup EXIT

if [[ "$AGENT_KIND" == "online" ]]; then
  setsid ros2 run embodied_online_agent online_agent --ros-args \
    -p mode:=mock \
    -p microphone_enabled:=false \
    -p wake_word_enabled:=false \
    -p continuous_control_enabled:=false \
    -p user_memory_dir:="$USER_MEMORY_DIR" \
    >"$LOG_FILE" 2>&1 &
else
  setsid ros2 run embodied_offline_agent offline_agent --ros-args \
    -p mode:=mock \
    -p microphone_enabled:=false \
    -p wake_word_enabled:=false \
    -p continuous_control_enabled:=false \
    -p user_memory_dir:="$USER_MEMORY_DIR" \
    >"$LOG_FILE" 2>&1 &
fi
PIDS+=("$!")

if ! timeout 25 env USER_MEMORY_DIR="$USER_MEMORY_DIR" \
  bash "$WORKSPACE/tools/acceptance/run_probe.sh" "$WORKSPACE/tests/integration/voice/test_speaker_memory_mock.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi

echo "PASS: speaker-memory mock ($AGENT_KIND)"
