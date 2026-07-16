#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/ros_dds_env.sh"
cd "$WORKSPACE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 18))}"

REPORT="$WORKSPACE/logs/showcase/automatic_mission_cancel_dry_run.json"
NODE_LOG="$(mktemp)"
setsid ros2 run embodied_slam_tools voice_slam_session_orchestrator --ros-args \
  -p "workspace:=$WORKSPACE" \
  -p mode:=offline \
  -p "map_prefix:=$WORKSPACE/logs/showcase/dry_run_cancel_map" \
  -p dry_run:=true \
  -p dry_run_exploration_delay_s:=2.0 >"$NODE_LOG" 2>&1 &
NODE_PID=$!

cleanup() {
  kill -TERM -- "-$NODE_PID" 2>/dev/null || true
  wait "$NODE_PID" 2>/dev/null || true
  rm -f "$NODE_LOG"
}
trap cleanup EXIT INT TERM

if ! timeout 30 python3 tests/integration/test_voice_slam_session_orchestrator.py \
  --output "$REPORT" \
  --cancel-automatic-mission; then
  echo "---- automatic cancel orchestrator log ----" >&2
  cat "$NODE_LOG" >&2
  exit 1
fi

echo "PASS: urgent voice stop cancels automatic exploration and recovers to MAPPING"
