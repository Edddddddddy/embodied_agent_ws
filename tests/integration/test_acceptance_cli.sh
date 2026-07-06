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
grep -q "core" <<<"$OUTPUT"
grep -q "microphone-offline" <<<"$OUTPUT"
grep -q "microphone-online" <<<"$OUTPUT"
grep -q "continuous-live-check" <<<"$OUTPUT"
grep -q "continuous-nav2-offline" <<<"$OUTPUT"
grep -q "continuous-nav2-online" <<<"$OUTPUT"
grep -q "continuous-nav2-evidence" <<<"$OUTPUT"
grep -q "continuous-nav2-live-check" <<<"$OUTPUT"
grep -q "navigation-demo" <<<"$OUTPUT"
grep -q "nav2-bridge" <<<"$OUTPUT"
grep -q "nav2-preflight" <<<"$OUTPUT"
grep -q "nav2-stage" <<<"$OUTPUT"
grep -q "nav2-turtlebot3" <<<"$OUTPUT"
grep -q "openwakeword-sidecar" <<<"$OUTPUT"
grep -q "livekit-sidecar" <<<"$OUTPUT"
grep -q "kws-calibration" <<<"$OUTPUT"
grep -q "voice-readiness" <<<"$OUTPUT"
grep -q "provider-preflight" <<<"$OUTPUT"
grep -q "continuous-endpoint" <<<"$OUTPUT"
grep -q "continuous-multi-command" <<<"$OUTPUT"
grep -q "continuous-navigation" <<<"$OUTPUT"
grep -q "continuous-soak" <<<"$OUTPUT"
grep -q "continuous-queue-full" <<<"$OUTPUT"
grep -q "continuous-ttl" <<<"$OUTPUT"
grep -q "continuous-timeout" <<<"$OUTPUT"
grep -q "excludes interactive microphone" <<<"$OUTPUT"
echo "PASS: acceptance CLI documents automated and interactive delivery modes"
