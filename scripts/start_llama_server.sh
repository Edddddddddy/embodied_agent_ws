#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
BIN="${LLAMA_SERVER:-$WORKSPACE/third_party/llama.cpp/build/bin/llama-server}"
MODEL="${LLAMA_MODEL:-$WORKSPACE/models/Qwen3-0.6B-Q8_0.gguf}"
exec "$BIN" -m "$MODEL" --host 127.0.0.1 --port "${LLAMA_PORT:-8080}" \
  -c "${LLAMA_CONTEXT:-2048}" -t "${LLAMA_THREADS:-8}" --jinja
