#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 20))}"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-/opt/ros/jazzy/share}"

SLAM_SOLVER="${SLAM_SOLVER:-ceres}"
if [[ "$SLAM_SOLVER" != "ceres" && "$SLAM_SOLVER" != "gtsam" ]]; then
  echo "SLAM_SOLVER must be ceres or gtsam" >&2
  exit 2
fi
PARAMS_FILE="$WORKSPACE/install/embodied_slam/share/embodied_slam/config/slam_mapping_${SLAM_SOLVER}.yaml"
REPORT_FILE="${SLAM_BASELINE_REPORT:-$WORKSPACE/logs/slam_${SLAM_SOLVER}_report.json}"
MAP_PREFIX="${SLAM_MAP_PREFIX:-$WORKSPACE/logs/slam_${SLAM_SOLVER}_map}"
LAUNCH_LOG="$(mktemp)"
setsid ros2 launch embodied_slam mapping_baseline.launch.py \
  params_file:="$PARAMS_FILE" headless:=true use_rviz:=false auto_drive:=true \
  >"$LAUNCH_LOG" 2>&1 &
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

if ! bash tests/integration/run_probe.sh tests/integration/slam_nav/test_slam_mapping_baseline.py \
    --timeout "${SLAM_BASELINE_TIMEOUT:-150}" \
    --settle "${SLAM_BASELINE_SETTLE:-5}" \
    --solver "$SLAM_SOLVER" \
    --output "$REPORT_FILE"; then
  echo "---- mapping launch log (last 160 lines) ----" >&2
  tail -n 160 "$LAUNCH_LOG" >&2
  exit 1
fi

# 地图保存必须发生在 slam_toolbox 仍存活时；生成的 YAML/PGM 直接供下一阶段 AMCL 使用。
ros2 run nav2_map_server map_saver_cli -f "$MAP_PREFIX" --ros-args \
  -p save_map_timeout:=10.0 -p map_subscribe_transient_local:=true
test -s "${MAP_PREFIX}.yaml"
test -s "${MAP_PREFIX}.pgm"

echo "SLAM baseline evidence: $REPORT_FILE"
echo "Saved map: ${MAP_PREFIX}.yaml"
