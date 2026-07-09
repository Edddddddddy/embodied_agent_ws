#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODEL_DIR="${MODEL_DIR:-$WORKSPACE/models}"
PROFILE="openwakeword"
DRY_RUN="${VOICE_KWS_SETUP_DRY_RUN:-false}"
KWS_DIR="${KWS_DIR:-$MODEL_DIR/kws}"
SHERPA_KWS_ENV="${SHERPA_KWS_ENV:-$WORKSPACE/logs/sherpa_kws.env}"
SHERPA_ASR_DIR="${SHERPA_ASR_DIR:-$MODEL_DIR/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16}"
SHERPA_KWS_KEYWORDS_FILE="${SHERPA_KWS_KEYWORDS_FILE:-$KWS_DIR/xiaozhi_keywords.txt}"

usage() {
  cat >&2 <<'EOF'
Usage: setup_voice_kws_runtime.sh {openwakeword|sherpa|livekit|all} [--dry-run]

Install optional acoustic keyword wake runtime dependencies for continuous voice control.

Profiles:
  openwakeword  Install openWakeWord Python runtime.
  sherpa        Prepare sherpa-onnx KWS package/model paths and a default keywords file.
  livekit       Install LiveKit WakeWord Python runtime.
  all           Prepare openWakeWord, sherpa-onnx KWS, and LiveKit paths.

Environment:
  WORKSPACE=/home/ubuntu/embodied_agent_ws
  MODEL_DIR=$WORKSPACE/models
  VOICE_KWS_SETUP_DRY_RUN=true  Print commands without installing/downloading.
EOF
}

while (($#)); do
  case "$1" in
    --dry-run)
      DRY_RUN=true
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    openwakeword|sherpa|livekit|all)
      PROFILE="$1"
      ;;
    *)
      usage
      exit 2
      ;;
  esac
  shift
done

PYTHON_BIN="${PYTHON_BIN:-$WORKSPACE/.venv/bin/python}"
if [[ ! -x "$PYTHON_BIN" ]]; then
  PYTHON_BIN="python3"
fi

run_or_print() {
  if [[ "$DRY_RUN" == "true" ]]; then
    echo "DRY RUN: $*"
  else
    "$@"
  fi
}

write_sherpa_keywords() {
  if [[ "$DRY_RUN" == "true" ]]; then
    echo "DRY RUN: mkdir -p $KWS_DIR"
    echo "DRY RUN: write SHERPA_KWS_KEYWORDS_FILE=$SHERPA_KWS_KEYWORDS_FILE"
    return
  fi
  mkdir -p "$KWS_DIR"
  cat >"$SHERPA_KWS_KEYWORDS_FILE" <<'EOF'
小 智
你 好 小 智
EOF
}

print_sherpa_exports() {
  cat <<EOF
export SHERPA_KWS_TOKENS=$SHERPA_ASR_DIR/tokens.txt
export SHERPA_KWS_ENCODER=$SHERPA_ASR_DIR/encoder-epoch-99-avg-1.int8.onnx
export SHERPA_KWS_DECODER=$SHERPA_ASR_DIR/decoder-epoch-99-avg-1.int8.onnx
export SHERPA_KWS_JOINER=$SHERPA_ASR_DIR/joiner-epoch-99-avg-1.int8.onnx
export SHERPA_KWS_KEYWORDS_FILE=$SHERPA_KWS_KEYWORDS_FILE
EOF
}

write_sherpa_env() {
  if [[ "$DRY_RUN" == "true" ]]; then
    echo "DRY RUN: write SHERPA_KWS_ENV=$SHERPA_KWS_ENV"
    return
  fi
  mkdir -p "$(dirname "$SHERPA_KWS_ENV")"
  print_sherpa_exports >"$SHERPA_KWS_ENV"
}

install_openwakeword() {
  run_or_print "$PYTHON_BIN" -m pip install -e "$WORKSPACE/src/embodied_online_agent[kws]"
}

install_livekit() {
  run_or_print "$PYTHON_BIN" -m pip install -e "$WORKSPACE/src/embodied_online_agent[livekit-kws]"
}

prepare_sherpa() {
  run_or_print bash "$WORKSPACE/scripts/setup_sherpa_asr_runtime.sh"
  write_sherpa_keywords
  write_sherpa_env
}

echo "Voice KWS runtime setup: profile=$PROFILE workspace=$WORKSPACE"
echo "Python: $PYTHON_BIN"

case "$PROFILE" in
  openwakeword)
    install_openwakeword
    KWS_PROVIDER_TO_CHECK="openwakeword"
    ;;
  sherpa)
    prepare_sherpa
    KWS_PROVIDER_TO_CHECK="sherpa"
    ;;
  livekit)
    install_livekit
    KWS_PROVIDER_TO_CHECK="livekit"
    ;;
  all)
    run_or_print "$PYTHON_BIN" -m pip install -e "$WORKSPACE/src/embodied_online_agent[kws,livekit-kws]"
    prepare_sherpa
    KWS_PROVIDER_TO_CHECK="openwakeword"
    ;;
  *)
    usage
    exit 2
    ;;
esac

if [[ "$PROFILE" == "sherpa" || "$PROFILE" == "all" ]]; then
  print_sherpa_exports
fi

if [[ "$DRY_RUN" == "true" ]]; then
  if [[ "$KWS_PROVIDER_TO_CHECK" == "sherpa" ]]; then
    echo "DRY RUN: $PYTHON_BIN $WORKSPACE/scripts/voice_provider_preflight.py --mode offline --vad-provider auto --kws-provider sherpa --sherpa-tokens $SHERPA_ASR_DIR/tokens.txt --sherpa-encoder $SHERPA_ASR_DIR/encoder-epoch-99-avg-1.int8.onnx --sherpa-decoder $SHERPA_ASR_DIR/decoder-epoch-99-avg-1.int8.onnx --sherpa-joiner $SHERPA_ASR_DIR/joiner-epoch-99-avg-1.int8.onnx --sherpa-keywords-file $SHERPA_KWS_KEYWORDS_FILE"
  else
    echo "DRY RUN: $PYTHON_BIN $WORKSPACE/scripts/voice_provider_preflight.py --mode offline --vad-provider auto --kws-provider $KWS_PROVIDER_TO_CHECK"
  fi
  echo "DRY RUN: KWS_PROVIDER=$KWS_PROVIDER_TO_CHECK bash scripts/acceptance_test.sh provider-preflight"
  echo "DRY RUN: KWS_PROVIDER=$KWS_PROVIDER_TO_CHECK bash scripts/acceptance_test.sh continuous-offline"
  if [[ "$KWS_PROVIDER_TO_CHECK" == "sherpa" ]]; then
    echo "DRY RUN: source logs/sherpa_kws.env before running the checks above"
  fi
  exit 0
fi

if [[ "$KWS_PROVIDER_TO_CHECK" == "sherpa" ]]; then
  "$PYTHON_BIN" "$WORKSPACE/scripts/voice_provider_preflight.py" \
    --mode offline \
    --vad-provider auto \
    --kws-provider sherpa \
    --sherpa-tokens "$SHERPA_ASR_DIR/tokens.txt" \
    --sherpa-encoder "$SHERPA_ASR_DIR/encoder-epoch-99-avg-1.int8.onnx" \
    --sherpa-decoder "$SHERPA_ASR_DIR/decoder-epoch-99-avg-1.int8.onnx" \
    --sherpa-joiner "$SHERPA_ASR_DIR/joiner-epoch-99-avg-1.int8.onnx" \
    --sherpa-keywords-file "$SHERPA_KWS_KEYWORDS_FILE"
else
  "$PYTHON_BIN" "$WORKSPACE/scripts/voice_provider_preflight.py" \
    --mode offline \
    --vad-provider auto \
    --kws-provider "$KWS_PROVIDER_TO_CHECK"
fi

SHERPA_SOURCE_HINT=""
if [[ "$PROFILE" == "sherpa" || "$PROFILE" == "all" ]]; then
  SHERPA_SOURCE_HINT="  source logs/sherpa_kws.env"
fi

cat <<EOF

Voice KWS runtime prepared.

Recommended checks:
  source scripts/activate.sh
$SHERPA_SOURCE_HINT
  KWS_PROVIDER=$KWS_PROVIDER_TO_CHECK bash scripts/acceptance_test.sh provider-preflight
  KWS_PROVIDER=$KWS_PROVIDER_TO_CHECK bash scripts/acceptance_test.sh continuous-offline

For sherpa KWS, export these paths before launch:
$(print_sherpa_exports)
EOF
