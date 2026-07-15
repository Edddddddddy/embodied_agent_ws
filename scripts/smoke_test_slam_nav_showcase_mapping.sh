#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/ros_dds_env.sh"
cd "$WORKSPACE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 18))}"

WORLD="$WORKSPACE/src/embodied_simulation/worlds/showcase_apartment.sdf.xacro"
STATIC_MAP="$WORKSPACE/src/embodied_simulation/maps/showcase_apartment.yaml"
OUTPUT_DIR="${SHOWCASE_MAPPING_DIR:-$WORKSPACE/logs/showcase/mapping_smoke}"
MAP_PREFIX="$OUTPUT_DIR/voice_built_map"
LAUNCH_LOG="$(mktemp)"
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR"

setsid ros2 launch embodied_simulation voice_nav2_turtlebot3.launch.py \
  launch_agent:=false \
  headless:=true use_rviz:=false \
  slam:=True \
  executor_plugin:=embodied_simulation/GazeboRobotExecutor \
  world:="$WORLD" map:="$STATIC_MAP" \
  x_pose:=-4.15 y_pose:=-3.15 yaw:=0.0 \
  use_composition:=true >"$LAUNCH_LOG" 2>&1 &
LAUNCH_PID=$!

cleanup() {
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  for _ in $(seq 1 40); do
    kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  rm -f "$LAUNCH_LOG"
}
trap cleanup EXIT INT TERM

MAP_DISCOVERED=false
for _ in $(seq 1 120); do
  if ros2 topic list 2>/dev/null | grep -qx /map; then
    MAP_DISCOVERED=true
    break
  fi
  sleep 0.5
done
if [[ "$MAP_DISCOVERED" != "true" ]] || ! timeout 30 ros2 topic echo /map --once >/dev/null; then
  echo "---- mapping launch log (last 220 lines) ----" >&2
  tail -n 220 "$LAUNCH_LOG" >&2
  echo "FAIL: slam_toolbox did not publish /map" >&2
  exit 1
fi

# 与真实语音最终走同一个 ExecuteRobotCommand Action；这里绕过麦克风只验证
# mapping profile 选择 Gazebo executor 后，探索动作不会被 Nav2 executor 拒绝。
ros2 run embodied_agent_cpp typed_action_demo_client move 0.18 2.0 \
  --ros-args -p result_timeout_s:=20.0
sleep 2
ros2 run nav2_map_server map_saver_cli -f "$MAP_PREFIX" \
  --ros-args -p save_map_timeout:=15.0

python3 - "$MAP_PREFIX.yaml" "$MAP_PREFIX.pgm" <<'PY'
from pathlib import Path
import sys
import yaml

yaml_path, pgm_path = map(Path, sys.argv[1:])
assert yaml_path.stat().st_size > 80
assert pgm_path.stat().st_size > 1000
metadata = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
assert 0.02 <= float(metadata["resolution"]) <= 0.10
tokens = [token for line in pgm_path.read_bytes().splitlines() if not line.startswith(b"#") for token in line.split()]
assert tokens[0] in {b"P5", b"P2"}
print(f"saved map={yaml_path} resolution={metadata['resolution']}")
PY

echo "PASS: realistic world -> slam_toolbox /map -> typed exploration action -> map_saver"
