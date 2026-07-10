#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
BIN="${LLAMA_SERVER:-$WORKSPACE/third_party/llama.cpp/build/bin/llama-server}"
MODEL="${LLAMA_MODEL:-$WORKSPACE/models/Qwen3-0.6B-Q8_0.gguf}"
HOST="${LLAMA_HOST:-127.0.0.1}"
PORT="${LLAMA_PORT:-8080}"
CONTEXT="${LLAMA_CONTEXT:-2048}"
THREADS="${LLAMA_THREADS:-8}"
PARALLEL="${LLAMA_PARALLEL:-1}"
GPU_LAYERS="${LLAMA_N_GPU_LAYERS:-0}"

if [[ ! -x "$BIN" ]]; then
  echo "MISSING llama-server binary: $BIN" >&2
  echo "Run: bash scripts/setup_offline_runtime.sh" >&2
  exit 1
fi
if [[ ! -s "$MODEL" ]]; then
  echo "MISSING GGUF model: $MODEL" >&2
  echo "Expected example: models/Qwen3-0.6B-Q8_0.gguf" >&2
  exit 1
fi

ARGS=(
  -m "$MODEL"
  --host "$HOST"
  --port "$PORT"
  -c "$CONTEXT"
  -t "$THREADS"
  --parallel "$PARALLEL"
  --jinja
)

if [[ "$GPU_LAYERS" != "0" ]]; then
  ARGS+=(--n-gpu-layers "$GPU_LAYERS")
fi

# LLAMA_EXTRA_ARGS 用于现场调参，例如 "--parallel 1 --cont-batching"。
# 这里有意放在最后，让高级用户可以覆盖 llama.cpp 的可选运行参数。
if [[ -n "${LLAMA_EXTRA_ARGS:-}" ]]; then
  # shellcheck disable=SC2206
  EXTRA_ARGS=($LLAMA_EXTRA_ARGS)
  ARGS+=("${EXTRA_ARGS[@]}")
fi

echo "Starting llama-server: model=$MODEL host=$HOST port=$PORT ctx=$CONTEXT threads=$THREADS parallel=$PARALLEL gpu_layers=$GPU_LAYERS" >&2
exec "$BIN" "${ARGS[@]}"
