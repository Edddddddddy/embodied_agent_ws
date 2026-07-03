#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((120 + $$ % 40))}"

LOG_FILE="$(mktemp)"
setsid ros2 run embodied_agent_cpp audio_frontend --ros-args \
  -p capture_enabled:=false \
  -p speaker_enabled:=false \
  -p vad_provider:=energy \
  -p audio_enhancer:=nlms \
  -p aec_enabled:=true \
  -p noise_suppression_enabled:=false \
  -p auto_gain_enabled:=false \
  -p speech_end_silence_s:=0.4 \
  -p min_utterance_ms:=100.0 \
  -p max_utterance_s:=12.0 \
  >"$LOG_FILE" 2>&1 &
PID=$!

cleanup() {
  kill -TERM -- "-$PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$PID" 2>/dev/null || true
  wait "$PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

for _ in $(seq 1 40); do
  if ros2 topic list 2>/dev/null | grep -qx "/audio/speech_started" && \
     ros2 topic list 2>/dev/null | grep -qx "/audio/speech_ended"; then
    break
  fi
  sleep 0.1
done

ros2 topic list | grep -qx "/audio/speech_started" || { cat "$LOG_FILE" >&2; exit 1; }
ros2 topic list | grep -qx "/audio/speech_ended" || { cat "$LOG_FILE" >&2; exit 1; }
ros2 param get /audio_frontend vad_provider | grep -q "energy" || {
  cat "$LOG_FILE" >&2
  exit 1
}
ros2 param get /audio_frontend audio_enhancer | grep -q "nlms" || {
  cat "$LOG_FILE" >&2
  exit 1
}
ros2 param get /audio_frontend aec_enabled | grep -q "True" || {
  cat "$LOG_FILE" >&2
  exit 1
}

echo "PASS: audio endpoint topics, VAD seam, and enhancer seam are available"
