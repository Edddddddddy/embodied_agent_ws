#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
BASELINE_MODEL="${LORA_BASELINE_Q8:-$WORKSPACE/models/Qwen3-0.6B-Q8_0.gguf}"
TUNED_MODEL="${LORA_TUNED_Q8:-$WORKSPACE/models/Qwen3-0.6B-robot-Q8_0.gguf}"
BASELINE_PORT="${LORA_BASELINE_PORT:-18180}"
TUNED_PORT="${LORA_TUNED_PORT:-18181}"
DATASET="${LORA_EVAL_DATASET:-training/robot_instruction_eval.jsonl}"
SERVER_PID=""

cd "$WORKSPACE"
source "$WORKSPACE/scripts/activate.sh"
mkdir -p "$WORKSPACE/logs"

cleanup() {
  if [[ -n "$SERVER_PID" ]] && kill -0 "$SERVER_PID" 2>/dev/null; then
    kill "$SERVER_PID" 2>/dev/null || true
    wait "$SERVER_PID" 2>/dev/null || true
  fi
  SERVER_PID=""
}
trap cleanup EXIT INT TERM

wait_ready() {
  local port="$1"
  for _ in $(seq 1 120); do
    if curl --silent --fail "http://127.0.0.1:$port/health" >/dev/null; then
      return 0
    fi
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
      return 1
    fi
    sleep 1
  done
  return 1
}

evaluate_one() {
  local label="$1"
  local model="$2"
  local port="$3"
  local output="$4"
  local server_log="$WORKSPACE/logs/llama_server_${label}.log"

  test -s "$model" || { echo "MISSING model: $model" >&2; return 2; }
  if curl --silent --fail "http://127.0.0.1:$port/health" >/dev/null 2>&1; then
    echo "Port $port is already serving; refusing to reuse an unverified model process." >&2
    return 2
  fi
  # 每侧都启动自己的服务并绑定模型文件哈希，杜绝旧 server 复用造成的假对照。
  env -u LLAMA_EXTRA_ARGS -u LLAMA_ARG_REASONING -u LLAMA_ARG_THINK \
    WORKSPACE="$WORKSPACE" LLAMA_PORT="$port" LLAMA_MODEL="$model" \
    LLAMA_N_GPU_LAYERS=0 "$WORKSPACE/scripts/start_llama_server.sh" \
    >"$server_log" 2>&1 &
  SERVER_PID=$!
  wait_ready "$port" || {
    echo "llama-server failed to become ready; see $server_log" >&2
    return 2
  }

  python3 "$WORKSPACE/scripts/evaluate_instruction_following.py" \
    --dataset "$DATASET" \
    --base-url "http://127.0.0.1:$port/v1" \
    --model "$(basename "$model")" \
    --model-file "$model" \
    --temperature 0 \
    --max-tokens 192 \
    --seed 42 \
    --output "$output"
  cleanup
}

evaluate_one baseline "$BASELINE_MODEL" "$BASELINE_PORT" \
  "$WORKSPACE/logs/instruction_following_baseline_q8.json"
evaluate_one tuned "$TUNED_MODEL" "$TUNED_PORT" \
  "$WORKSPACE/logs/instruction_following_lora_q8.json"

python3 "$WORKSPACE/scripts/compare_instruction_following_reports.py" \
  --baseline "$WORKSPACE/logs/instruction_following_baseline_q8.json" \
  --tuned "$WORKSPACE/logs/instruction_following_lora_q8.json"
