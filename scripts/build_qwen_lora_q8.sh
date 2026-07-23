#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
LORA_VENV="${LORA_VENV:-$WORKSPACE/.venv-lora}"
LLAMA_CPP="${LLAMA_CPP_ROOT:-$WORKSPACE/third_party/llama.cpp}"
MERGED_DIR="${LORA_MERGED_DIR:-$WORKSPACE/outputs/qwen3-0.6b-robot-merged}"
F16_GGUF="${LORA_F16_GGUF:-$WORKSPACE/outputs/qwen3-0.6b-robot-f16.gguf}"
Q8_GGUF="${LORA_Q8_GGUF:-$WORKSPACE/models/Qwen3-0.6B-robot-Q8_0.gguf}"
MODE="${1:---dry-run}"
cd "$WORKSPACE"

commands=(
  "$LORA_VENV/bin/python $WORKSPACE/scripts/build_robot_lora_dataset.py"
  "$LORA_VENV/bin/llamafactory-cli train $WORKSPACE/training/qwen3_0_6b_lora.yaml"
  "$LORA_VENV/bin/llamafactory-cli export $WORKSPACE/training/qwen3_0_6b_lora_merge.yaml"
  "$LORA_VENV/bin/python $LLAMA_CPP/convert_hf_to_gguf.py $MERGED_DIR --outfile $F16_GGUF --outtype f16"
  "$WORKSPACE/scripts/quantize_qwen_q8.sh $F16_GGUF $Q8_GGUF"
  "bash $WORKSPACE/tools/evaluation/evaluate_lora_q8_comparison.sh"
  "$LORA_VENV/bin/python $WORKSPACE/tools/evaluation/audit_lora_q8_pipeline.py --f16 $F16_GGUF --q8 $Q8_GGUF --strict-reproduced"
)

if [[ "$MODE" == "--dry-run" ]]; then
  printf '[dry-run] %s\n' "${commands[@]}"
  source "$WORKSPACE/scripts/activate.sh"
  python3 "$WORKSPACE/tools/evaluation/audit_lora_q8_pipeline.py" --f16 "$F16_GGUF" --q8 "$Q8_GGUF"
  exit 0
fi
if [[ "$MODE" != "--execute" ]]; then
  echo "Usage: $0 [--dry-run|--execute]" >&2
  exit 2
fi

test -x "$LORA_VENV/bin/llamafactory-cli" || {
  echo "missing LLaMA-Factory runtime; run scripts/setup_lora_toolchain.sh" >&2
  exit 2
}
test -f "$LLAMA_CPP/convert_hf_to_gguf.py"
test -x "$LLAMA_CPP/build/bin/llama-quantize"
mkdir -p "$WORKSPACE/outputs" "$WORKSPACE/models"

for command in "${commands[@]}"; do
  echo "+ $command"
  bash -lc "$command"
done
