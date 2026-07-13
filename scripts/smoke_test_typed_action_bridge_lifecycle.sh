#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/lifecycle_utils.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((120 + $$ % 80))}"

LOG_FILE="$(mktemp)"
setsid ros2 launch embodied_simulation simulation_control.launch.py \
  autostart:=false \
  use_typed_actions:=true \
  executor_plugin:=embodied_simulation/MockRobotExecutor >"$LOG_FILE" 2>&1 &
CONTROL_PID=$!
cleanup() {
  kill -TERM -- "-$CONTROL_PID" 2>/dev/null || true
  kill -KILL -- "-$CONTROL_PID" 2>/dev/null || true
  wait "$CONTROL_PID" 2>/dev/null || true
  rm -f "$LOG_FILE"
}
trap cleanup EXIT

activate_lifecycle_node simulation_control
timeout 12 python3 "$WORKSPACE/scripts/activate_lifecycle_node.py" \
  typed_action_bridge --target-state inactive
wait_for_topic_subscribers /robot/action_command_typed 1

# configured/inactive 时消息必须被拒绝，不能留到下一次 activate 后偷偷执行。
ros2 topic pub --once /robot/action_command_typed \
  embodied_agent_interfaces/msg/RobotCommand \
  "{command_id: inactive-command, source: lifecycle_probe, action_type: 1, priority: true}" \
  >/dev/null
for _ in $(seq 1 40); do
  if grep -q "rejected command while lifecycle inactive: command_id=inactive-command" "$LOG_FILE"; then
    break
  fi
  sleep 0.1
done
grep -q "rejected command while lifecycle inactive: command_id=inactive-command" "$LOG_FILE"

activate_lifecycle_node typed_action_bridge
timeout 15 python3 "$WORKSPACE/tests/integration/test_typed_action_bridge_lifecycle.py" \
  --command-id first-activation

timeout 12 python3 "$WORKSPACE/scripts/activate_lifecycle_node.py" \
  typed_action_bridge --target-state inactive
timeout 12 python3 "$WORKSPACE/scripts/activate_lifecycle_node.py" \
  typed_action_bridge --target-state unconfigured
timeout 12 python3 "$WORKSPACE/scripts/activate_lifecycle_node.py" \
  typed_action_bridge --target-state inactive
activate_lifecycle_node typed_action_bridge
timeout 15 python3 "$WORKSPACE/tests/integration/test_typed_action_bridge_lifecycle.py" \
  --command-id reactivated

echo "PASS: typed Action bridge inactive reject -> activate -> cleanup -> reactivate"
