#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/lifecycle_utils.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((120 + $$ % 80))}"

LOG_FILE="$(mktemp)"
TYPED_FILE="$(mktemp)"
LEGACY_FILE="$(mktemp)"
setsid ros2 run embodied_agent_cpp action_guard >"$LOG_FILE" 2>&1 &
GUARD_PID=$!
cleanup() {
  kill -TERM -- "-$GUARD_PID" 2>/dev/null || true
  kill -KILL -- "-$GUARD_PID" 2>/dev/null || true
  wait "$GUARD_PID" 2>/dev/null || true
  rm -f "$LOG_FILE" "$TYPED_FILE" "$LEGACY_FILE"
}
trap cleanup EXIT

for _ in $(seq 1 50); do
  ros2 node list 2>/dev/null | grep -qx /action_guard && break
  sleep 0.1
done
activate_lifecycle_node action_guard

timeout 8 ros2 topic echo --once /robot/action_command_typed \
  embodied_agent_interfaces/msg/RobotCommand >"$TYPED_FILE" &
TYPED_PID=$!
timeout 8 ros2 topic echo --once /robot/action_command \
  std_msgs/msg/String >"$LEGACY_FILE" &
LEGACY_PID=$!
wait_for_topic_subscribers /robot/action_command_typed
wait_for_topic_subscribers /robot/action_command
ros2 topic pub --once /agent/action_candidate std_msgs/msg/String \
  "{data: '{\"name\":\"move\",\"arguments\":{\"linear_x\":9.0,\"duration_s\":20.0}}'}" \
  >/dev/null

wait "$TYPED_PID" || { cat "$LOG_FILE" >&2; exit 1; }
wait "$LEGACY_PID" || { cat "$LOG_FILE" >&2; exit 1; }
grep -q "action_type: 2" "$TYPED_FILE"
grep -q "linear_x: 0.5" "$TYPED_FILE"
grep -q "duration_s: 10.0" "$TYPED_FILE"
grep -q "command_id: guard-" "$TYPED_FILE"
grep -q '"name":"move"' "$LEGACY_FILE"
grep -q '"linear_x":0.5' "$LEGACY_FILE"
echo "PASS: ActionGuard dual-published equivalent legacy and typed commands"
