#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((165 + $$ % 35))}"

# 这个 smoke 是显式 WebRTC runtime 验收：与 dependency-free 的 Silero seam 测试不同，
# 这里要求 `webrtcvad` 真正安装成功。如果缺依赖，preflight 会先给出可执行的 setup 建议。
python3 "$WORKSPACE/scripts/voice_provider_preflight.py" \
  --mode offline \
  --vad-provider webrtc \
  --kws-provider none

AUDIO_LOG="$(mktemp)"
VAD_LOG="$(mktemp)"

setsid ros2 run embodied_agent_cpp audio_frontend --ros-args \
  -p capture_enabled:=false \
  -p speaker_enabled:=false \
  -p vad_provider:=webrtc \
  -p endpoint_events_enabled:=false \
  >"$AUDIO_LOG" 2>&1 &
AUDIO_PID=$!

setsid ros2 run embodied_online_agent webrtc_vad --ros-args \
  -p enabled:=true \
  -p sample_rate:=16000 \
  -p frame_ms:=20 \
  -p aggressiveness:=2 \
  -p speech_end_silence_s:=0.7 \
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

for _ in $(seq 1 80); do
  if ros2 param get /audio_frontend endpoint_events_enabled >/dev/null 2>&1 && \
     ros2 param get /webrtc_vad enabled >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done

ros2 param get /audio_frontend vad_provider | grep -q "webrtc" || {
  cat "$AUDIO_LOG" >&2
  exit 1
}
ros2 param get /audio_frontend endpoint_events_enabled | grep -q "False" || {
  cat "$AUDIO_LOG" >&2
  exit 1
}
ros2 param get /webrtc_vad enabled | grep -q "True" || {
  cat "$VAD_LOG" >&2
  exit 1
}
ros2 topic list | grep -qx "/audio/vad_event" || {
  cat "$VAD_LOG" >&2
  exit 1
}

echo "PASS: WebRTC VAD sidecar runtime is installed and owns speech endpoint events"
