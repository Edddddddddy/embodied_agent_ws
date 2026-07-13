#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

TMP_DIR="$(mktemp -d /tmp/openloris-replay-smoke.XXXXXX)"
cleanup() {
  # 只删除本脚本创建且带固定前缀的临时目录，防止变量异常时扩大清理范围。
  case "$TMP_DIR" in
    /tmp/openloris-replay-smoke.*) rm -rf -- "$TMP_DIR" ;;
  esac
}
trap cleanup EXIT

BAG_PATH="$TMP_DIR/bag"
python3 scripts/generate_openloris_replay_fixture.py --output "$BAG_PATH"
ros2 run embodied_slam_tools openloris_rosbag_inspect "$BAG_PATH" \
  --output "$TMP_DIR/contract.json"

for backend in ceres gtsam; do
  estimate="$TMP_DIR/${backend}.tum"
  launch_log="$TMP_DIR/${backend}.log"
  params_file="$WORKSPACE/install/embodied_slam/share/embodied_slam/config/openloris_mapping_${backend}.yaml"
  ROS_DOMAIN_ID="$((150 + $$ % 80))" timeout 40 \
    ros2 launch embodied_slam openloris_mapping.launch.py \
      bag_path:="$BAG_PATH" params_file:="$params_file" \
      replay_rate:=8.0 startup_delay_s:=2.0 output_path:="$estimate" use_rviz:=false \
      >"$launch_log" 2>&1

  pose_count="$(grep -cv '^#' "$estimate")"
  if (( pose_count < 20 )); then
    tail -n 120 "$launch_log" >&2
    echo "FAIL: $backend recorded only $pose_count poses" >&2
    exit 1
  fi
  grep -q "replay complete" "$launch_log"
  grep -q "'duplicates': 1" "$launch_log"
  if grep -q -E "Traceback|process has died" "$launch_log"; then
    tail -n 120 "$launch_log" >&2
    echo "FAIL: $backend replay did not shut down cleanly" >&2
    exit 1
  fi
  echo "PASS: $backend replay -> $pose_count map-frame poses"
done

echo "PASS: OpenLORIS ROS bag -> ROS 2 TF/LaserScan -> slam_toolbox replay adapter"
