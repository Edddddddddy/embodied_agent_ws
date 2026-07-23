#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
BASE_URL="${LLAMA_BASE_URL:-http://${LLAMA_HOST:-127.0.0.1}:${LLAMA_PORT:-8080}}"
API_BASE_URL="${BASE_URL%/}"
if [[ "$API_BASE_URL" != */v1 ]]; then
  API_BASE_URL="$API_BASE_URL/v1"
fi
HEALTH_BASE_URL="${API_BASE_URL%/v1}"
LOG="$(mktemp)"
PID=""
cleanup() {
  if [[ -n "$PID" ]]; then
    kill "$PID" 2>/dev/null || true
    wait "$PID" 2>/dev/null || true
  fi
  rm -f "$LOG"
}
trap cleanup EXIT

if ! curl -fsS "$HEALTH_BASE_URL/health" >/dev/null 2>&1; then
  bash "$WORKSPACE/scripts/start_llama_server.sh" >"$LOG" 2>&1 &
  PID=$!
fi

for _ in $(seq 1 60); do
  if curl -fsS "$HEALTH_BASE_URL/health" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

curl -fsS "$HEALTH_BASE_URL/health" >/dev/null || { cat "$LOG"; exit 1; }
python "$WORKSPACE/tools/evaluation/evaluate_instruction_following.py" \
  --base-url "$API_BASE_URL" \
  "$@"
