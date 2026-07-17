#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"
# Fast DDS 的标准端口公式只允许 domain id <= 232；预留 190-209 给该重型测试。
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((190 + $$ % 20))}"
export GZ_SIM_RESOURCE_PATH="${GZ_SIM_RESOURCE_PATH:-/opt/ros/jazzy/share}"

MAP_FILE="${SLAM_LOCALIZATION_MAP:-$WORKSPACE/logs/slam_ceres_map.yaml}"
REPORT_FILE="${SLAM_NAVIGATION_REPORT:-$WORKSPACE/logs/slam_navigation_report.json}"
NAV2_PARAMS_FILE="${SLAM_NAV2_PARAMS:-$WORKSPACE/logs/slam_nav2_params.yaml}"
if [[ ! -s "$MAP_FILE" ]]; then
  echo "Missing generated map: $MAP_FILE" >&2
  echo "Run: bash scripts/acceptance_test.sh slam-benchmark" >&2
  exit 2
fi

# 保留 Nav2 官方完整参数，只叠加本项目的预测动态障碍层，避免复制一份很快过时的配置。
python3 scripts/build_slam_nav2_params.py --output "$NAV2_PARAMS_FILE"

LAUNCH_LOG="$(mktemp)"
setsid ros2 launch embodied_slam localization_navigation.launch.py \
  map:="$MAP_FILE" params_file:="$NAV2_PARAMS_FILE" \
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

if ! bash tools/acceptance/run_probe.sh tests/integration/slam_nav/test_slam_localization_navigation.py \
    --timeout "${SLAM_NAVIGATION_TIMEOUT:-140}" --output "$REPORT_FILE"; then
  echo "---- localization/navigation launch log (last 200 lines) ----" >&2
  tail -n 200 "$LAUNCH_LOG" >&2
  exit 1
fi
echo "Localization/navigation evidence: $REPORT_FILE"
