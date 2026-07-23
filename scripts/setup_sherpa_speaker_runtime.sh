#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODEL_DIR="${SHERPA_SPEAKER_MODEL_DIR:-$WORKSPACE/models/speaker_id}"
MODEL_NAME="3dspeaker_speech_eres2net_base_200k_sv_zh-cn_16k-common.onnx"
MODEL_PATH="$MODEL_DIR/$MODEL_NAME"
MODEL_URL="https://github.com/k2-fsa/sherpa-onnx/releases/download/speaker-recongition-models/$MODEL_NAME"
EXPECTED_SHA256="e2d2048292e055f7b61cdec3db010503f35369b245bf0b3bbad021c9a91e4053"

mkdir -p "$MODEL_DIR"
if [[ ! -s "$MODEL_PATH" ]]; then
  echo "Downloading official sherpa-onnx Chinese speaker embedding model..."
  curl --fail --location --retry 3 --output "$MODEL_PATH.part" "$MODEL_URL"
  mv "$MODEL_PATH.part" "$MODEL_PATH"
fi

MODEL_SIZE="$(stat -c %s "$MODEL_PATH")"
if (( MODEL_SIZE < 1000000 )); then
  echo "speaker model is unexpectedly small: $MODEL_SIZE bytes" >&2
  exit 1
fi

ACTUAL_SHA256="$(sha256sum "$MODEL_PATH" | cut -d ' ' -f1)"
if [[ "$ACTUAL_SHA256" != "$EXPECTED_SHA256" ]]; then
  echo "speaker model SHA256 mismatch: $ACTUAL_SHA256" >&2
  exit 1
fi
printf '%s  %s\n' "$ACTUAL_SHA256" "$MODEL_PATH" >"$MODEL_PATH.sha256"
echo "READY: $MODEL_PATH ($MODEL_SIZE bytes)"
echo "SHA256: $ACTUAL_SHA256"
