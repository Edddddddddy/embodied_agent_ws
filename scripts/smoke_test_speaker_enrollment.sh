#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((225 + $$ % 5))}"

SPEAKER_ENROLL_DIR="$(mktemp -d)"
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
  rm -rf "$SPEAKER_ENROLL_DIR" "$LOG_FILE"
}
trap cleanup EXIT

setsid ros2 run embodied_online_agent speaker_identity --ros-args \
  -p mode:=mock \
  -p publish_on_start:=false \
  -p min_audio_rms:=0.0 \
  -p enroll_dir:="$SPEAKER_ENROLL_DIR" \
  >"$LOG_FILE" 2>&1 &
PIDS+=("$!")

if ! timeout 25 env SPEAKER_ENROLL_DIR="$SPEAKER_ENROLL_DIR" \
  python3 "$WORKSPACE/tests/integration/test_speaker_enrollment.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi

echo "PASS: speaker enrollment samples"
