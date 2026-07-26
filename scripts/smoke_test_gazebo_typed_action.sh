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

# 先证明 /clock 持续推进；仅看到 odom/scan topic 不足以证明 use_sim_time
# 可用，冻结的 Gazebo clock 会让所有按时长执行的 Action 永远停在 0%。
if ! python "$WORKSPACE/scripts/simulation_readiness_check.py" \
  --timeout 35.0 --json
then
  echo "readiness failed; launch log follows" >&2
  cat "$LOG_FILE" >&2
  exit 1
fi

if ! REQUIRE_TYPED_ACTION_RESULT=true \
  bash "$WORKSPACE/tools/acceptance/run_probe.sh" "$WORKSPACE/tools/acceptance/probes/control/gazebo_motion.py"
then
  cat "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: typed ROS Action -> cmd_vel -> Gazebo odometry -> terminal result"
