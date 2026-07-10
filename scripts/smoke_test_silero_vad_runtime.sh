#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODEL="${SILERO_VAD_MODEL_PATH:-$WORKSPACE/models/silero_vad/silero_vad.onnx}"
WAV="${SILERO_VAD_TEST_WAV:-$WORKSPACE/models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/test_wavs/0.wav}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((200 + $$ % 20))}"

python3 "$WORKSPACE/scripts/voice_provider_preflight.py" \
  --mode offline \
  --vad-provider silero \
  --kws-provider none \
  --silero-model-path "$MODEL" \
  --silero-use-onnx true

VAD_LOG="$(mktemp)"
setsid ros2 run embodied_online_agent silero_vad --ros-args \
  -p enabled:=true \
  -p use_onnx:=true \
  -p model_path:="$MODEL" \
  -p frame_ms:=32 \
  -p speech_start_ms:=96.0 \
  -p threshold:=0.5 \
  -p speech_end_threshold:=0.35 \
  -p speech_end_silence_s:=0.7 \
  >"$VAD_LOG" 2>&1 &
VAD_PID=$!

cleanup() {
  kill -TERM -- "-$VAD_PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$VAD_PID" 2>/dev/null || true
  wait "$VAD_PID" 2>/dev/null || true
  rm -f "$VAD_LOG"
}
trap cleanup EXIT

for _ in $(seq 1 80); do
  if ros2 param get /silero_vad enabled >/dev/null 2>&1; then
    break
  fi
  sleep 0.1
done
if ! ros2 param get /silero_vad enabled >/dev/null 2>&1; then
  cat "$VAD_LOG" >&2
  exit 1
fi

python3 "$WORKSPACE/scripts/silero_ros_runtime_probe.py" --wav "$WAV"
echo "PASS: real Silero ONNX inference owns paired ROS 2 speech endpoint events"
