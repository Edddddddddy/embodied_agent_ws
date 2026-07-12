#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
PROFILE="webrtc"
DRY_RUN="${VOICE_VAD_SETUP_DRY_RUN:-false}"
SILERO_VERSION="${SILERO_VAD_VERSION:-v6.2.1}"
SILERO_MODEL_DIR="${SILERO_VAD_MODEL_DIR:-$WORKSPACE/models/silero_vad}"
SILERO_MODEL_PATH="${SILERO_VAD_MODEL_PATH:-$SILERO_MODEL_DIR/silero_vad.onnx}"
SILERO_MODEL_URL="https://raw.githubusercontent.com/snakers4/silero-vad/$SILERO_VERSION/src/silero_vad/data/silero_vad.onnx"
SILERO_MODEL_SHA256="${SILERO_VAD_MODEL_SHA256:-1a153a22f4509e292a94e67d6f9b85e8deb25b4988682b7e174c65279d8788e3}"

usage() {
  cat >&2 <<'EOF'
Usage: setup_voice_vad_runtime.sh {webrtc|silero|all} [--dry-run]

Install optional mature VAD runtime dependencies for continuous voice control.

Profiles:
  webrtc  Install lightweight py-webrtcvad support.
  silero  Install lightweight ONNX Runtime and pinned Silero model (no PyTorch).
  all     Install both WebRTC and Silero VAD extras.

Environment:
  WORKSPACE=/home/ubuntu/embodied_agent_ws
  VOICE_VAD_SETUP_DRY_RUN=true  Print commands without installing packages.
  SILERO_VAD_VERSION=v6.2.1    Pinned upstream model version.
  SILERO_VAD_MODEL_DIR=...     Model destination directory.
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
    webrtc|silero|all)
      PROFILE="$1"
      ;;
    *)
      usage
      exit 2
      ;;
  esac
  shift
done

case "$PROFILE" in
  webrtc)
    EXTRAS="webrtc-vad"
    EXPECTED_PROVIDER="webrtc"
    ;;
  silero)
    EXTRAS="silero-vad"
    EXPECTED_PROVIDER="silero"
    ;;
  all)
    EXTRAS="webrtc-vad,silero-vad"
    EXPECTED_PROVIDER="silero"
    ;;
  *)
    usage
    exit 2
    ;;
esac

PYTHON_BIN="${PYTHON_BIN:-$WORKSPACE/.venv/bin/python}"
if [[ ! -x "$PYTHON_BIN" ]]; then
  PYTHON_BIN="python3"
fi
PACKAGE_SPEC="$WORKSPACE/src/embodied_voice_frontend[$EXTRAS]"

echo "Voice VAD runtime setup: profile=$PROFILE workspace=$WORKSPACE"
echo "Python: $PYTHON_BIN"

run_or_print() {
  if [[ "$DRY_RUN" == "true" ]]; then
    echo "DRY RUN: $*"
  else
    "$@"
  fi
}

run_or_print "$PYTHON_BIN" -m pip install -e "$PACKAGE_SPEC"

if [[ "$PROFILE" == "silero" || "$PROFILE" == "all" ]]; then
  if [[ "$DRY_RUN" == "true" ]]; then
    echo "DRY RUN: mkdir -p $SILERO_MODEL_DIR"
    echo "DRY RUN: curl -fL $SILERO_MODEL_URL -o $SILERO_MODEL_PATH.tmp"
    echo "DRY RUN: verify sha256=$SILERO_MODEL_SHA256 and install $SILERO_MODEL_PATH"
  else
    mkdir -p "$SILERO_MODEL_DIR"
    tmp_model="$SILERO_MODEL_PATH.tmp"
    rm -f "$tmp_model"
    curl -fL "$SILERO_MODEL_URL" -o "$tmp_model"
    actual_sha=$(sha256sum "$tmp_model" | awk '{print $1}')
    if [[ "$actual_sha" != "$SILERO_MODEL_SHA256" ]]; then
      rm -f "$tmp_model"
      echo "FAIL: Silero model checksum mismatch: expected=$SILERO_MODEL_SHA256 actual=$actual_sha" >&2
      exit 1
    fi
    mv "$tmp_model" "$SILERO_MODEL_PATH"
  fi
fi

if [[ "$DRY_RUN" == "true" ]]; then
  echo "DRY RUN: $PYTHON_BIN $WORKSPACE/scripts/voice_provider_preflight.py --mode offline --vad-provider auto --kws-provider none --silero-model-path $SILERO_MODEL_PATH"
  echo "DRY RUN: VAD_PROVIDER=$EXPECTED_PROVIDER bash scripts/acceptance_test.sh provider-preflight"
  echo "DRY RUN: VAD_PROVIDER=auto bash scripts/acceptance_test.sh continuous-offline"
  exit 0
fi

"$PYTHON_BIN" "$WORKSPACE/scripts/voice_provider_preflight.py" \
  --mode offline \
  --vad-provider auto \
  --kws-provider none \
  --silero-model-path "$SILERO_MODEL_PATH" \
  --silero-use-onnx true

if [[ "$PROFILE" == "silero" || "$PROFILE" == "all" ]]; then
  "$PYTHON_BIN" "$WORKSPACE/scripts/silero_onnx_smoke.py" \
    --model "$SILERO_MODEL_PATH"
fi

cat <<EOF

Voice VAD runtime installed.

Recommended checks:
  source scripts/activate.sh
  SILERO_VAD_MODEL_PATH=$SILERO_MODEL_PATH VAD_PROVIDER=$EXPECTED_PROVIDER bash scripts/acceptance_test.sh provider-preflight
  SILERO_VAD_MODEL_PATH=$SILERO_MODEL_PATH bash scripts/acceptance_test.sh silero-vad-runtime
  VAD_PROVIDER=auto bash scripts/acceptance_test.sh continuous-offline

Note:
  VAD_PROVIDER=auto still falls back safely if a provider is unavailable.
EOF
