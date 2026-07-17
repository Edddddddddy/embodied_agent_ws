#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-211}"

LOG_FILE="$(mktemp)"
ros2 run embodied_navigation dynamic_obstacle_tracker_node >"$LOG_FILE" 2>&1 &
TRACKER_PID=$!
cleanup() {
  kill -TERM "$TRACKER_PID" 2>/dev/null || true
  wait "$TRACKER_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

if ! bash tools/acceptance/run_probe.sh tests/integration/slam_nav/test_dynamic_obstacle_tracker_ros.py; then
  cat "$LOG_FILE" >&2
  exit 1
fi
