#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((160 + $$ % 40))}"

AUDIO_LOG="$(mktemp)"
VAD_LOG="$(mktemp)"

setsid ros2 run embodied_agent_cpp audio_frontend --ros-args \
  -p capture_enabled:=false \
  -p speaker_enabled:=false \
  -p vad_provider:=silero \
  -p endpoint_events_enabled:=false \
  >"$AUDIO_LOG" 2>&1 &
AUDIO_PID=$!

setsid ros2 run embodied_online_agent silero_vad --ros-args \
  -p enabled:=false \
  -p sample_rate:=16000 \
  -p frame_ms:=32 \
  -p threshold:=0.5 \
  >"$VAD_LOG" 2>&1 &
VAD_PID=$!

cleanup() {
  kill -TERM -- "-$AUDIO_PID" "-$VAD_PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$AUDIO_PID" "-$VAD_PID" 2>/dev/null || true
  wait "$AUDIO_PID" "$VAD_PID" 2>/dev/null || true
  rm -f "$AUDIO_LOG" "$VAD_LOG"
}
trap cleanup EXIT

for _ in $(seq 1 60); do
  if ros2 param get /audio_frontend endpoint_events_enabled >/dev/null 2>&1 && \
     ros2 param get /silero_vad enabled >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

ros2 param get /audio_frontend vad_provider | grep -q "silero" || {
  cat "$AUDIO_LOG" >&2
  exit 1
}
ros2 param get /audio_frontend endpoint_events_enabled | grep -q "False" || {
  cat "$AUDIO_LOG" >&2
  exit 1
}
ros2 param get /silero_vad enabled | grep -q "False" || {
  cat "$VAD_LOG" >&2
  exit 1
}
ros2 topic list | grep -qx "/audio/vad_event" || {
  cat "$VAD_LOG" >&2
  exit 1
}

echo "PASS: optional Silero VAD sidecar launch seam is available without model dependencies"
