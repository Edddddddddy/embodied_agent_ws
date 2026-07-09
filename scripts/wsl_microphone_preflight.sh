#!/usr/bin/env bash
set -euo pipefail

DURATION="${MIC_PREFLIGHT_DURATION:-3}"
THRESHOLD="${MIC_PREFLIGHT_MIN_RMS:-0.001}"
SOURCE="${MIC_PREFLIGHT_SOURCE:-@DEFAULT_SOURCE@}"
RAW_FILE="$(mktemp)"

cleanup() {
  rm -f "$RAW_FILE"
}
trap cleanup EXIT

if ! command -v pactl >/dev/null 2>&1; then
  echo "FAIL: pactl not found; install pulseaudio-utils in WSL." >&2
  exit 1
fi
if ! command -v parecord >/dev/null 2>&1; then
  echo "FAIL: parecord not found; install pulseaudio-utils in WSL." >&2
  exit 1
fi

echo "WSL microphone preflight"
echo "PULSE_SERVER=${PULSE_SERVER:-<unset>}"
echo "Available PulseAudio sources:"
pactl list short sources || true
echo
echo "请现在对着麦克风说话 ${DURATION}s，例如：小智，向前走一秒"

set +e
timeout "$DURATION" parecord \
  --device="$SOURCE" \
  --format=s16le \
  --rate=16000 \
  --channels=1 \
  --raw \
  "$RAW_FILE" >/tmp/wsl_microphone_preflight_parecord.log 2>&1
STATUS=$?
set -e

if [[ "$STATUS" -ne 0 && "$STATUS" -ne 124 ]]; then
  echo "FAIL: parecord failed. Recent log:" >&2
  sed 's/^/  /' /tmp/wsl_microphone_preflight_parecord.log >&2 || true
  exit 1
fi
if [[ ! -s "$RAW_FILE" ]]; then
  echo "FAIL: no audio data captured from PulseAudio source=$SOURCE" >&2
  exit 1
fi

python3 - "$RAW_FILE" "$THRESHOLD" <<'PY'
import json
import math
import struct
import sys
from pathlib import Path

path = Path(sys.argv[1])
threshold = float(sys.argv[2])
data = path.read_bytes()
samples = struct.unpack("<" + "h" * (len(data) // 2), data[: len(data) // 2 * 2])
if not samples:
    print("FAIL: captured file has zero PCM samples", file=sys.stderr)
    raise SystemExit(1)
peak = max(abs(value) for value in samples)
rms = math.sqrt(sum(value * value for value in samples) / len(samples)) / 32768.0
report = {
    "samples": len(samples),
    "rms": round(rms, 6),
    "peak": peak,
    "threshold": threshold,
    "status": "PASS" if rms >= threshold else "BLOCKED",
}
print(json.dumps(report, ensure_ascii=False, indent=2))
if rms < threshold:
    print(
        "BLOCKED: WSL PulseAudio captured near-silence. "
        "Check Windows microphone permission/input device, then rerun this script.",
        file=sys.stderr,
    )
    raise SystemExit(1)
PY

echo "PASS: WSL microphone source has audible input"
