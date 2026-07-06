#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODEL_DIR="${MODEL_DIR:-$WORKSPACE/models}"
ASR_DIR="$MODEL_DIR/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16"
SHERPA_ONNX_VERSION="${SHERPA_ONNX_VERSION:-1.13.3}"
HF_ENDPOINT="${HF_ENDPOINT:-https://huggingface.co}"
ASR_REPO="$HF_ENDPOINT/csukuangfj/k2fsa-zipformer-bilingual-zh-en-t/resolve/main"

mkdir -p "$ASR_DIR"
source "$WORKSPACE/.venv/bin/activate"

python -m pip install "sherpa-onnx==$SHERPA_ONNX_VERSION" numpy

download_file() {
  local url="$1"
  local output="$2"
  local minimum_bytes="$3"
  if [[ -f "$output" ]] && (( $(stat -c %s "$output") >= minimum_bytes )); then
    echo "READY: $output"
    return
  fi
  mkdir -p "$(dirname "$output")"
  if command -v aria2c >/dev/null 2>&1; then
    aria2c --allow-overwrite=true --auto-file-renaming=false --continue=true \
      --max-connection-per-server=8 --split=8 --min-split-size=1M \
      --dir="$(dirname "$output")" --out="$(basename "$output")" "$url"
  else
    curl -L --fail --retry 3 --output "$output" "$url"
  fi
  if (( $(stat -c %s "$output") < minimum_bytes )); then
    echo "Downloaded file is unexpectedly small: $output" >&2
    exit 1
  fi
  echo "READY: $output"
}

download_file "$ASR_REPO/exp/32/encoder-epoch-99-avg-1.int8.onnx?download=true" "$ASR_DIR/encoder-epoch-99-avg-1.int8.onnx" 40000000
download_file "$ASR_REPO/exp/32/decoder-epoch-99-avg-1.int8.onnx?download=true" "$ASR_DIR/decoder-epoch-99-avg-1.int8.onnx" 3000000
download_file "$ASR_REPO/exp/32/joiner-epoch-99-avg-1.int8.onnx?download=true" "$ASR_DIR/joiner-epoch-99-avg-1.int8.onnx" 3000000
download_file "$ASR_REPO/data/lang_char_bpe/tokens.txt?download=true" "$ASR_DIR/tokens.txt" 50000
download_file "$ASR_REPO/test_wavs/0.wav?download=true" "$ASR_DIR/test_wavs/0.wav" 100000

python3 "$WORKSPACE/scripts/sherpa_asr_smoke.py" --model-dir "$ASR_DIR" --wav "$ASR_DIR/test_wavs/0.wav" --preflight-only

cat <<EOF

Sherpa-ONNX ASR runtime is ready.

Next checks:
  source scripts/activate.sh
  bash scripts/acceptance_test.sh sherpa-asr-preflight
  bash scripts/acceptance_test.sh sherpa-asr-smoke

This ASR-only setup intentionally does not download llama.cpp, Qwen GGUF, or Sherpa-TTS.
Use scripts/setup_offline_runtime.sh when you need the full offline Agent stack.
EOF
