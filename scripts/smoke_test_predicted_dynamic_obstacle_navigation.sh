#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 20))}"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-/opt/ros/jazzy/share}"

MAP_FILE="${SLAM_LOCALIZATION_MAP:-$WORKSPACE/logs/slam_ceres_map.yaml}"
PARAMS_FILE="${SLAM_NAV2_PARAMS:-$WORKSPACE/logs/slam_nav2_params.yaml}"
REPORT_FILE="${DYNAMIC_NAVIGATION_REPORT:-$WORKSPACE/logs/dynamic_obstacle_navigation_report.json}"
MOTION_MODEL="${DYNAMIC_MOTION_MODEL:-constant_velocity}"
if [[ ! -s "$MAP_FILE" ]]; then
  echo "Missing generated map: $MAP_FILE" >&2
  echo "Run: bash scripts/acceptance_test.sh slam-benchmark" >&2
  exit 2
fi
python3 scripts/build_slam_nav2_params.py --output "$PARAMS_FILE"

LAUNCH_LOG="$(mktemp)"
setsid ros2 launch embodied_slam localization_navigation.launch.py \
  map:="$MAP_FILE" params_file:="$PARAMS_FILE" \
  motion_model:="$MOTION_MODEL" \
  headless:=True use_rviz:=False >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  for _ in $(seq 1 30); do
    kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  rm -f "$LAUNCH_LOG"
}
trap cleanup EXIT

if ! python3 tests/integration/test_predicted_dynamic_obstacle_navigation.py \
    --timeout "${DYNAMIC_NAVIGATION_TIMEOUT:-160}" --motion-model "$MOTION_MODEL" \
    --output "$REPORT_FILE"; then
  echo "---- predicted dynamic obstacle launch log (last 240 lines) ----" >&2
  tail -n 240 "$LAUNCH_LOG" >&2
  exit 1
fi
echo "Dynamic obstacle evidence: $REPORT_FILE"
