#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
python "$WORKSPACE/scripts/benchmark_offline.py" "$@"
"$WORKSPACE/third_party/llama.cpp/build/bin/llama-bench" \
  -m "$WORKSPACE/models/Qwen3-0.6B-Q8_0.gguf" -p 64 -n 128 -t "${LLAMA_THREADS:-8}"
