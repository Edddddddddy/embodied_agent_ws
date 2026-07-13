#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
SHERPA_KWS_ENV="${SHERPA_KWS_ENV:-$WORKSPACE/logs/sherpa_kws.env}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((180 + $$ % 40))}"

if [[ ! -f "$SHERPA_KWS_ENV" ]]; then
  echo "FAIL: missing $SHERPA_KWS_ENV" >&2
  echo "Run: bash scripts/setup_voice_kws_runtime.sh sherpa" >&2
  exit 1
fi

# shellcheck source=/dev/null
source "$SHERPA_KWS_ENV"

python3 "$WORKSPACE/scripts/voice_provider_preflight.py" \
  --mode offline \
  --vad-provider auto \
  --kws-provider sherpa \
  --sherpa-tokens "$SHERPA_KWS_TOKENS" \
  --sherpa-encoder "$SHERPA_KWS_ENCODER" \
  --sherpa-decoder "$SHERPA_KWS_DECODER" \
  --sherpa-joiner "$SHERPA_KWS_JOINER" \
  --sherpa-keywords-file "$SHERPA_KWS_KEYWORDS_FILE"

KWS_LOG="$(mktemp)"

setsid ros2 run embodied_voice_frontend keyword_wake --ros-args \
  -p mode:=sherpa \
  -p provider_name:=sherpa_kws_test \
  -p sherpa_tokens:="$SHERPA_KWS_TOKENS" \
  -p sherpa_encoder:="$SHERPA_KWS_ENCODER" \
  -p sherpa_decoder:="$SHERPA_KWS_DECODER" \
  -p sherpa_joiner:="$SHERPA_KWS_JOINER" \
  -p sherpa_keywords_file:="$SHERPA_KWS_KEYWORDS_FILE" \
  >"$KWS_LOG" 2>&1 &
KWS_PID=$!

cleanup() {
  kill -TERM -- "-$KWS_PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$KWS_PID" 2>/dev/null || true
  wait "$KWS_PID" 2>/dev/null || true
  rm -f "$KWS_LOG"
}
trap cleanup EXIT

NODE_READY=false
for _ in $(seq 1 180); do
  if grep -q "keyword wake sidecar ready: mode=sherpa" "$KWS_LOG"; then
    NODE_READY=true
    break
  fi
  if ! kill -0 "$KWS_PID" 2>/dev/null; then
    cat "$KWS_LOG" >&2
    exit 1
  fi
  sleep 0.1
done

if [[ "$NODE_READY" != "true" ]]; then
  echo "FAIL: keyword_wake sherpa node did not become ready before timeout" >&2
  cat "$KWS_LOG" >&2
  exit 1
fi

echo "PASS: Sherpa-ONNX KWS sidecar runtime starts with generated keyword env"
