#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

LIVE_REPORT="$TMP_DIR/live.json"
SUMMARY_REPORT="$TMP_DIR/summary.json"
STARTED_AT="$(date +%s)"

set +e
OUTPUT="$({
  cd "$WORKSPACE"
  VOICE_BENCHMARK_DURATION=1 \
  VOICE_BENCHMARK_PROGRESS_INTERVAL=1 \
  VOICE_BENCHMARK_LIVE_REPORT="$LIVE_REPORT" \
  VOICE_BENCHMARK_REPORT="$SUMMARY_REPORT" \
    bash scripts/acceptance_test.sh continuous-voice-benchmark offline
} 2>&1)"
STATUS=$?
set -e

ELAPSED=$(( $(date +%s) - STARTED_AT ))
if (( ELAPSED > 10 )); then
  echo "FAIL: one-second benchmark did not exit promptly (${ELAPSED}s)" >&2
  exit 1
fi
if [[ ! -s "$LIVE_REPORT" ]]; then
  echo "FAIL: live report was not written" >&2
  exit 1
fi
if [[ ! -s "$SUMMARY_REPORT" ]]; then
  echo "FAIL: summary report was skipped after an incomplete live check" >&2
  exit 1
fi
if [[ "$OUTPUT" != *"剩余"* ]]; then
  echo "FAIL: benchmark printed no visible countdown/progress" >&2
  exit 1
fi
if [[ "$STATUS" -eq 0 ]]; then
  echo "FAIL: empty ROS observation unexpectedly passed" >&2
  exit 1
fi

python3 - "$LIVE_REPORT" "$SUMMARY_REPORT" <<'PY'
import json
import sys

live = json.load(open(sys.argv[1], encoding="utf-8"))
summary = json.load(open(sys.argv[2], encoding="utf-8"))
assert live["duration_s"] >= 1.0
assert summary["passed"] is False
PY

echo "PASS: benchmark exits on time, shows progress, and always writes both reports"

DRY_OUTPUT="$({
  cd "$WORKSPACE"
  CONTINUOUS_VOICE_EVIDENCE_DRY_RUN=true \
    bash scripts/acceptance_test.sh continuous-voice-evidence offline
} 2>&1)"
if [[ "$DRY_OUTPUT" != *"自动关闭后台 ROS/Gazebo 进程并退出"* ]]; then
  echo "FAIL: one-terminal evidence mode does not explain automatic cleanup" >&2
  exit 1
fi
if [[ "$DRY_OUTPUT" != *"continuous-voice-benchmark offline"* ]]; then
  echo "FAIL: one-terminal evidence mode does not invoke the benchmark" >&2
  exit 1
fi

echo "PASS: one-terminal evidence mode documents automatic startup and cleanup"
