#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
# Fast DDS 的默认端口公式要求 domain id 不超过 232；留出余量避免随机 PID
# 把验收落到非法 domain。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((200 + $$ % 20))}"

MODEL="${SHERPA_SPEAKER_MODEL:-$WORKSPACE/models/speaker_id/3dspeaker_speech_eres2net_base_200k_sv_zh-cn_16k-common.onnx}"
WAV="${SPEAKER_TEST_WAV:-$WORKSPACE/models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/test_wavs/0.wav}"
SPEAKER_ID="${SPEAKER_TEST_ID:-runtime_probe_user}"
SPEAKER_FILE="$(mktemp)"
LOG_FILE="$(mktemp)"
NODE_PID=""

cleanup() {
  if [[ -n "$NODE_PID" ]]; then
    kill -TERM -- "-$NODE_PID" 2>/dev/null || true
    kill -KILL -- "-$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
  fi
  rm -f "$SPEAKER_FILE" "$LOG_FILE"
}
trap cleanup EXIT

test -s "$MODEL" || { echo "missing speaker model; run setup_sherpa_speaker_runtime.sh" >&2; exit 2; }
test -s "$WAV" || { echo "missing speaker test WAV: $WAV" >&2; exit 2; }
# 同一用户写入三段注册记录，覆盖真实 enrollment 的多样本聚合路径。
printf '%s %s\n%s %s\n%s %s\n' \
  "$SPEAKER_ID" "$WAV" \
  "$SPEAKER_ID" "$WAV" \
  "$SPEAKER_ID" "$WAV" >"$SPEAKER_FILE"

setsid ros2 run embodied_voice_frontend speaker_identity --ros-args \
  -p mode:=sherpa \
  -p publish_on_start:=false \
  -p sherpa_model:="$MODEL" \
  -p sherpa_speaker_file:="$SPEAKER_FILE" \
  -p sherpa_threshold:=0.6 \
  -p sherpa_min_margin:=0.05 \
  >"$LOG_FILE" 2>&1 &
NODE_PID=$!

if ! timeout 45 env SPEAKER_TEST_WAV="$WAV" SPEAKER_TEST_ID="$SPEAKER_ID" \
  bash "$WORKSPACE/tests/integration/run_probe.sh" "$WORKSPACE/tests/integration/voice/test_sherpa_speaker_identity_ros.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi

echo "PASS: real PCM -> sherpa speaker embedding -> ROS speaker identity"
