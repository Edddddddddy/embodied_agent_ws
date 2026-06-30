#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODEL_DIR="$WORKSPACE/models"
THIRD_PARTY="$WORKSPACE/third_party"
mkdir -p "$MODEL_DIR" "$THIRD_PARTY"

sudo apt-get update
sudo apt-get install -y build-essential bzip2 cmake curl git ninja-build

source "$WORKSPACE/.venv/bin/activate"
python -m pip install "sherpa-onnx==1.13.3"

download_extract() {
  local url="$1" archive="$2" marker="$3"
  if [[ -e "$marker" ]]; then return; fi
  curl -fL -C - --retry 3 "$url" -o "$archive"
  tar -xjf "$archive" -C "$MODEL_DIR"
  rm -f "$archive"
}

download_extract \
  "https://github.com/k2-fsa/sherpa-onnx/releases/download/asr-models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16.tar.bz2" \
  "$MODEL_DIR/zipformer.tar.bz2" \
  "$MODEL_DIR/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16/tokens.txt"
download_extract \
  "https://github.com/k2-fsa/sherpa-onnx/releases/download/tts-models/vits-melo-tts-zh_en.tar.bz2" \
  "$MODEL_DIR/vits.tar.bz2" \
  "$MODEL_DIR/vits-melo-tts-zh_en/model.onnx"

if [[ ! -d "$THIRD_PARTY/llama.cpp/.git" ]]; then
  git clone --depth 1 https://github.com/ggml-org/llama.cpp.git "$THIRD_PARTY/llama.cpp"
fi
cmake -S "$THIRD_PARTY/llama.cpp" -B "$THIRD_PARTY/llama.cpp/build" \
  -G Ninja -DCMAKE_BUILD_TYPE=Release -DGGML_NATIVE=ON -DLLAMA_CURL=OFF
cmake --build "$THIRD_PARTY/llama.cpp/build" --target llama-server llama-cli llama-bench llama-quantize -j "$(nproc)"

QWEN="$MODEL_DIR/Qwen3-0.6B-Q8_0.gguf"
if [[ ! -f "$QWEN" ]]; then
  curl -fL --retry 3 \
    "https://huggingface.co/Qwen/Qwen3-0.6B-GGUF/resolve/main/Qwen3-0.6B-Q8_0.gguf?download=true" \
    -o "$QWEN"
fi

echo "Offline runtime ready under $WORKSPACE"
