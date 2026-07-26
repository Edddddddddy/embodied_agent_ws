#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WORKSPACE="${WORKSPACE:-$(cd -- "$SCRIPT_DIR/../../.." && pwd -P)}"
CLI="$WORKSPACE/scripts/acceptance_test.sh"

PUBLIC_HELP="$(bash "$CLI" --help)"
for mode in core continuous-offline continuous-online gazebo nav2-stage slam-nav-e2e unknown-world-slam-e2e voice-unknown-world-slam-e2e robotics-gate; do
  grep -q "^  $mode " <<<"$PUBLIC_HELP"
done
grep -q "slam-nav-e2e.*Known-world deterministic" <<<"$PUBLIC_HELP"
grep -q "unknown-world-slam-e2e.*Unknown-world autonomous exploration" <<<"$PUBLIC_HELP"
grep -q "voice-unknown-world-slam-e2e.*Live microphone.*full-evidence unknown-world" <<<"$PUBLIC_HELP"
if grep -q "openloris-replay-stage" <<<"$PUBLIC_HELP"; then
  echo "FAIL: evaluation modes leaked into stable public help" >&2
  exit 1
fi

ALL_HELP="$(bash "$CLI" --help-all)"
grep -q "Internal regression modes:" <<<"$ALL_HELP"
grep -q "Evaluation/experiment modes:" <<<"$ALL_HELP"
grep -q "architecture-facts" <<<"$ALL_HELP"
grep -q "showcase-gazebo-e2e" <<<"$ALL_HELP"
grep -q "voice-runtime-preflight" <<<"$ALL_HELP"
grep -q "openloris-replay-stage" <<<"$ALL_HELP"
for removed_mode in microphone-offline microphone-online all; do
  if grep -q "^  $removed_mode " <<<"$ALL_HELP"; then
    echo "FAIL: removed acceptance mode leaked into --help-all: $removed_mode" >&2
    exit 1
  fi
done

set +e
UNKNOWN_OUTPUT="$(bash "$CLI" mode-that-does-not-exist 2>&1)"
UNKNOWN_STATUS=$?
set -e
[[ "$UNKNOWN_STATUS" -eq 2 ]]
grep -q "Unknown acceptance mode" <<<"$UNKNOWN_OUTPUT"

# 复杂的带参模式仍走同一个 Python 注册表；dry-run 不启动 ROS/Gazebo。
DRY_RUN_OUTPUT="$(
  ROS_DOMAIN_ID=31 \
  CONTINUOUS_NAV2_EVIDENCE_DRY_RUN=true \
  CONTINUOUS_LIVE_CHECK_REPORT=/tmp/nav2-live-check-dry-run.json \
  CONTINUOUS_LIVE_CHECK_DURATION=9 \
  bash "$CLI" continuous-nav2-evidence offline
)"
grep -q "DRY RUN" <<<"$DRY_RUN_OUTPUT"
grep -q "continuous_nav2_voice_control.sh offline" <<<"$DRY_RUN_OUTPUT"
grep -q "/tmp/nav2-live-check-dry-run.json" <<<"$DRY_RUN_OUTPUT"

echo "PASS: thin acceptance CLI and registered internal modes"
