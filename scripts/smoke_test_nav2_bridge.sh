#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/lifecycle_utils.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((150 + $$ % 30))}"

LOG_FILE="$(mktemp)"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  sleep 0.2
  for pid in "${PIDS[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

setsid ros2 launch embodied_simulation simulation_control.launch.py \
  use_typed_actions:=true \
  executor_plugin:=embodied_simulation/Nav2RobotExecutor \
  >"$LOG_FILE" 2>&1 &
PIDS+=("$!")
setsid ros2 run embodied_agent_cpp action_guard >>"$LOG_FILE" 2>&1 &
PIDS+=("$!")
setsid ros2 run embodied_online_agent online_agent --ros-args \
  -p mode:=mock \
  -p microphone_enabled:=false \
  -p wake_word_enabled:=false \
  -p action_sequence_wait_timeout_s:=12.0 \
  >>"$LOG_FILE" 2>&1 &
PIDS+=("$!")

activate_lifecycle_node action_guard
wait_for_topic_subscribers /robot/action_command_typed
if ! timeout 45 python3 "$WORKSPACE/tests/integration/test_nav2_bridge_sequence.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: voice nav command -> Nav2 NavigateToPose/FollowWaypoints action goals"
