#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 2 ]]; then
  echo "Usage: $0 input-f16.gguf output-q8_0.gguf" >&2
  exit 2
fi
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
"$WORKSPACE/third_party/llama.cpp/build/bin/llama-quantize" "$1" "$2" Q8_0
ls -lh "$1" "$2"
