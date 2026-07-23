#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/ros_dds_env.sh"
cd "$WORKSPACE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 18))}"
export EMBODIED_NAV2_PLACES_FILE="$WORKSPACE/src/embodied_simulation/config/showcase_mapping_places.yaml"

WORLD="$WORKSPACE/src/embodied_simulation/worlds/showcase_apartment.sdf.xacro"
MAP="$WORKSPACE/src/embodied_simulation/maps/showcase_apartment_slam_frame.yaml"
REPORT="${SHOWCASE_NAV_REPORT:-$WORKSPACE/logs/showcase/slam_frame_navigation_report.json}"
LAUNCH_LOG="$(mktemp)"

python3 scripts/generate_showcase_scene.py --check

setsid ros2 launch embodied_simulation voice_nav2_turtlebot3.launch.py \
  launch_agent:=true \
  agent_type:=offline \
  provider_mode:=mock \
  microphone_enabled:=false \
  wake_word_enabled:=false \
  continuous_control_enabled:=false \
  lifecycle_autostart:=true \
  headless:="${SHOWCASE_HEADLESS:-true}" \
  use_rviz:=false \
  slam:=False \
  world:="$WORLD" \
  map:="$MAP" \
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

if ! timeout "${SHOWCASE_NAV_TIMEOUT:-150}" \
  bash tools/acceptance/run_probe.sh tools/acceptance/probes/slam_nav/nav2_turtlebot3_voice.py \
    --initial-x 0.0 --initial-y 0.0 --initial-yaw 0.0 \
    --navigate-text "去客厅" \
    --navigate-timeout "${SHOWCASE_NAV_TIMEOUT:-120}" \
    --skip-patrol \
    --output "$REPORT"; then
  echo "---- showcase launch log (last 220 lines) ----" >&2
  tail -n 220 "$LAUNCH_LOG" >&2
  exit 1
fi

echo "PASS: text ASR surrogate -> Agent -> typed Action -> SLAM-frame Nav2 motion"
echo "Showcase evidence: $REPORT"
