#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/lifecycle_utils.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((180 + $$ % 40))}"

LOG_FILE="$(mktemp)"
setsid ros2 launch embodied_simulation simulation_control.launch.py \
  use_typed_actions:=true \
  executor_plugin:=embodied_simulation/MockRobotExecutor \
  readiness_profile:=demo \
  readiness_required_components:=agent,action_guard,typed_action_bridge,simulation_control \
  >"$LOG_FILE" 2>&1 &
CONTROL_PID=$!
setsid ros2 run embodied_agent_cpp action_guard >>"$LOG_FILE" 2>&1 &
GUARD_PID=$!
setsid ros2 run embodied_online_agent online_agent --ros-args \
  -p mode:=mock \
  -p microphone_enabled:=false \
  -p wake_word_enabled:=false \
  -p action_sequence_wait_timeout_s:=10.0 \
  >>"$LOG_FILE" 2>&1 &
AGENT_PID=$!

cleanup() {
  kill -TERM -- "-$CONTROL_PID" "-$GUARD_PID" "-$AGENT_PID" 2>/dev/null || true
  sleep 0.2
  kill -KILL -- "-$CONTROL_PID" "-$GUARD_PID" "-$AGENT_PID" 2>/dev/null || true
  wait "$CONTROL_PID" "$GUARD_PID" "$AGENT_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

activate_lifecycle_node action_guard
python3 "$WORKSPACE/scripts/system_readiness_check.py" \
  --timeout 15 --profile demo
if ! timeout 35 bash "$WORKSPACE/tools/acceptance/run_probe.sh" "$WORKSPACE/tools/acceptance/probes/control/demo_sequence.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: text demo -> ordered actions -> Action/BT -> mock arc motion"
