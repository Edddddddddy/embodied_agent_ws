#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
INPUT="${1:-}"
OUTPUT="${2:-$WORKSPACE/models/Qwen3-0.6B-robot-Q8_0.gguf}"
QUANTIZE="${LLAMA_QUANTIZE:-$WORKSPACE/third_party/llama.cpp/build/bin/llama-quantize}"

if [[ -z "$INPUT" ]]; then
  echo "Usage: $0 INPUT_F16_GGUF [OUTPUT_Q8_GGUF]" >&2
  exit 2
fi
test -s "$INPUT" || { echo "missing F16 GGUF: $INPUT" >&2; exit 2; }
test -x "$QUANTIZE" || { echo "missing llama-quantize: $QUANTIZE" >&2; exit 2; }

mkdir -p "$(dirname "$OUTPUT")"
"$QUANTIZE" "$INPUT" "$OUTPUT" Q8_0
test -s "$OUTPUT"

INPUT_SIZE="$(stat -c %s "$INPUT")"
OUTPUT_SIZE="$(stat -c %s "$OUTPUT")"
python3 - "$INPUT_SIZE" "$OUTPUT_SIZE" <<'PY'
import json
import sys

source = int(sys.argv[1])
quantized = int(sys.argv[2])
print(json.dumps({
    "source_f16_bytes": source,
    "q8_0_bytes": quantized,
    "q8_to_f16_ratio": round(quantized / source, 4),
    "reduction_percent": round((1.0 - quantized / source) * 100.0, 2),
}, indent=2))
PY
