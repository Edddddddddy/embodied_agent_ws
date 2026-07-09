#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
PROFILE="webrtc"
DRY_RUN="${VOICE_VAD_SETUP_DRY_RUN:-false}"

usage() {
  cat >&2 <<'EOF'
Usage: setup_voice_vad_runtime.sh {webrtc|silero|all} [--dry-run]

Install optional mature VAD runtime dependencies for continuous voice control.

Profiles:
  webrtc  Install lightweight py-webrtcvad support.
  silero  Install Silero VAD + ONNX Runtime support.
  all     Install both WebRTC and Silero VAD extras.

Environment:
  WORKSPACE=/home/ubuntu/embodied_agent_ws
  VOICE_VAD_SETUP_DRY_RUN=true  Print commands without installing packages.
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
PACKAGE_SPEC="$WORKSPACE/src/embodied_online_agent[$EXTRAS]"

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

if [[ "$DRY_RUN" == "true" ]]; then
  echo "DRY RUN: $PYTHON_BIN $WORKSPACE/scripts/voice_provider_preflight.py --mode offline --vad-provider auto --kws-provider none"
  echo "DRY RUN: VAD_PROVIDER=$EXPECTED_PROVIDER bash scripts/acceptance_test.sh provider-preflight"
  echo "DRY RUN: VAD_PROVIDER=auto bash scripts/acceptance_test.sh continuous-offline"
  exit 0
fi

"$PYTHON_BIN" "$WORKSPACE/scripts/voice_provider_preflight.py" \
  --mode offline \
  --vad-provider auto \
  --kws-provider none

cat <<EOF

Voice VAD runtime installed.

Recommended checks:
  source scripts/activate.sh
  VAD_PROVIDER=$EXPECTED_PROVIDER bash scripts/acceptance_test.sh provider-preflight
  VAD_PROVIDER=auto bash scripts/acceptance_test.sh continuous-offline

Note:
  VAD_PROVIDER=auto still falls back safely if a provider is unavailable.
EOF
