#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODEL_DIR="$WORKSPACE/models"
THIRD_PARTY="$WORKSPACE/third_party"
SHERPA_ONNX_VERSION="${SHERPA_ONNX_VERSION:-1.13.3}"
LLAMA_CPP_REPO="${LLAMA_CPP_REPO:-https://github.com/ggml-org/llama.cpp.git}"
LLAMA_CPP_REF="${LLAMA_CPP_REF:-0eca4d490e591d4e93058d07540cf47278a72577}"
mkdir -p "$MODEL_DIR" "$THIRD_PARTY"
touch "$THIRD_PARTY/COLCON_IGNORE"

sudo apt-get update
sudo apt-get install -y aria2 build-essential cmake curl git ninja-build

source "$WORKSPACE/.venv/bin/activate"
python -m pip install "sherpa-onnx==$SHERPA_ONNX_VERSION"

download_file() {
  local url="$1" output="$2" minimum_bytes="$3"
  if [[ -f "$output" ]] && (( $(stat -c %s "$output") >= minimum_bytes )); then
    return
  fi
  mkdir -p "$(dirname "$output")"
  aria2c --allow-overwrite=true --auto-file-renaming=false --continue=true \
    --max-connection-per-server=8 --split=8 --min-split-size=1M \
    --dir="$(dirname "$output")" --out="$(basename "$output")" "$url"
  if (( $(stat -c %s "$output") < minimum_bytes )); then
    echo "Downloaded file is unexpectedly small: $output" >&2
    exit 1
  fi
}

HF_ENDPOINT="${HF_ENDPOINT:-https://huggingface.co}"
ASR_REPO="$HF_ENDPOINT/csukuangfj/k2fsa-zipformer-bilingual-zh-en-t/resolve/main"
ASR_DIR="$MODEL_DIR/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16"
download_file "$ASR_REPO/exp/32/encoder-epoch-99-avg-1.int8.onnx?download=true" "$ASR_DIR/encoder-epoch-99-avg-1.int8.onnx" 40000000
download_file "$ASR_REPO/exp/32/decoder-epoch-99-avg-1.int8.onnx?download=true" "$ASR_DIR/decoder-epoch-99-avg-1.int8.onnx" 3000000
download_file "$ASR_REPO/exp/32/joiner-epoch-99-avg-1.int8.onnx?download=true" "$ASR_DIR/joiner-epoch-99-avg-1.int8.onnx" 3000000
download_file "$ASR_REPO/data/lang_char_bpe/tokens.txt?download=true" "$ASR_DIR/tokens.txt" 50000
download_file "$ASR_REPO/test_wavs/0.wav?download=true" "$ASR_DIR/test_wavs/0.wav" 100000

TTS_REPO="$HF_ENDPOINT/csukuangfj/vits-melo-tts-zh_en/resolve/main"
TTS_DIR="$MODEL_DIR/vits-melo-tts-zh_en"
download_file "$TTS_REPO/model.onnx?download=true" "$TTS_DIR/model.onnx" 160000000
download_file "$TTS_REPO/model.int8.onnx?download=true" "$TTS_DIR/model.int8.onnx" 50000000
download_file "$TTS_REPO/lexicon.txt?download=true" "$TTS_DIR/lexicon.txt" 6000000
download_file "$TTS_REPO/tokens.txt?download=true" "$TTS_DIR/tokens.txt" 500
download_file "$TTS_REPO/date.fst?download=true" "$TTS_DIR/date.fst" 50000
download_file "$TTS_REPO/number.fst?download=true" "$TTS_DIR/number.fst" 50000
download_file "$TTS_REPO/phone.fst?download=true" "$TTS_DIR/phone.fst" 80000
rm -f "$MODEL_DIR/zipformer.tar.bz2" "$MODEL_DIR/vits.tar.bz2"

if [[ ! -d "$THIRD_PARTY/llama.cpp/.git" ]]; then
  git init "$THIRD_PARTY/llama.cpp"
  git -C "$THIRD_PARTY/llama.cpp" remote add origin "$LLAMA_CPP_REPO"
  git -C "$THIRD_PARTY/llama.cpp" fetch --depth 1 origin "$LLAMA_CPP_REF"
  git -C "$THIRD_PARTY/llama.cpp" checkout --detach FETCH_HEAD
else
  CURRENT_LLAMA_REF="$(git -C "$THIRD_PARTY/llama.cpp" rev-parse HEAD)"
  if [[ "$CURRENT_LLAMA_REF" != "$LLAMA_CPP_REF" ]]; then
    echo "WARN: llama.cpp ref is $CURRENT_LLAMA_REF, expected $LLAMA_CPP_REF" >&2
  fi
fi
cmake -S "$THIRD_PARTY/llama.cpp" -B "$THIRD_PARTY/llama.cpp/build" \
  -G Ninja -DCMAKE_BUILD_TYPE=Release -DGGML_NATIVE=ON -DLLAMA_CURL=OFF
cmake --build "$THIRD_PARTY/llama.cpp/build" --target llama-server llama-cli llama-bench llama-quantize -j "$(nproc)"

QWEN="$MODEL_DIR/Qwen3-0.6B-Q8_0.gguf"
download_file \
  "$HF_ENDPOINT/Qwen/Qwen3-0.6B-GGUF/resolve/main/Qwen3-0.6B-Q8_0.gguf?download=true" \
  "$QWEN" 600000000

if [[ "${SETUP_SUMMER_TTS:-true}" == "true" ]]; then
  bash "$WORKSPACE/scripts/setup_summer_tts_runtime.sh"
fi

echo "Offline runtime ready under $WORKSPACE"
