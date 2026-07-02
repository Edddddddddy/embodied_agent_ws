#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((100 + $$ % 100))}"

LOG_FILE="$(mktemp)"
setsid ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  gui:=false rviz:=false launch_agent:=false use_typed_actions:=true \
  >"$LOG_FILE" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  for _ in $(seq 1 20); do
    kill -0 -- "-$LAUNCH_PID" 2>/dev/null || break
    sleep 0.1
  done
  kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

if ! REQUIRE_TYPED_ACTION_RESULT=true \
  python "$WORKSPACE/scripts/test_gazebo_motion.py"
then
  cat "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: typed ROS Action -> cmd_vel -> Gazebo odometry -> terminal result"
