#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"

set +e
OUTPUT="$(bash "$WORKSPACE/scripts/acceptance_test.sh" help 2>&1)"
STATUS=$?
set -e

if [[ "$STATUS" -ne 0 ]]; then
  echo "FAIL: help must exit successfully" >&2
  exit 1
fi
grep -q "all" <<<"$OUTPUT"
grep -q "microphone-offline" <<<"$OUTPUT"
grep -q "microphone-online" <<<"$OUTPUT"
grep -q "openwakeword-sidecar" <<<"$OUTPUT"
grep -q "livekit-sidecar" <<<"$OUTPUT"
grep -q "kws-calibration" <<<"$OUTPUT"
grep -q "excludes interactive microphone" <<<"$OUTPUT"
echo "PASS: acceptance CLI documents automated and interactive delivery modes"
