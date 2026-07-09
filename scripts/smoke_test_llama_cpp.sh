#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"

MODE="${1:-preflight}"
BASE_URL="${LLAMA_BASE_URL:-http://${LLAMA_HOST:-127.0.0.1}:${LLAMA_PORT:-8080}}"
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

if ! curl -fsS "$BASE_URL/health" >/dev/null 2>&1; then
  bash "$WORKSPACE/scripts/start_llama_server.sh" >"$LOG" 2>&1 &
  PID=$!
fi

for _ in $(seq 1 60); do
  if curl -fsS "$BASE_URL/health" >/dev/null 2>&1; then
    break
  fi
  sleep 1
done

if ! curl -fsS "$BASE_URL/health" >/dev/null 2>&1; then
  echo "llama-server did not become healthy. Recent log:" >&2
  tail -80 "$LOG" >&2 || true
  exit 1
fi

ARGS=(--base-url "$BASE_URL")
if [[ "$MODE" == "smoke" || "$MODE" == "--infer" ]]; then
  ARGS+=(--infer)
elif [[ "$MODE" != "preflight" ]]; then
  echo "Usage: smoke_test_llama_cpp.sh [preflight|smoke]" >&2
  exit 2
fi

python3 "$WORKSPACE/scripts/llama_cpp_preflight.py" "${ARGS[@]}"
