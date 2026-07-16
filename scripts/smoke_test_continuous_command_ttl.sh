#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
AGENT_KIND="${1:-online}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/lifecycle_utils.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 20))}"

if [[ "$AGENT_KIND" != "online" && "$AGENT_KIND" != "offline" ]]; then
  echo "Usage: $0 {online|offline}" >&2
  exit 2
fi

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
  executor_plugin:=embodied_simulation/MockRobotExecutor \
  >"$LOG_FILE" 2>&1 &
PIDS+=("$!")
setsid ros2 run embodied_agent_cpp action_guard >>"$LOG_FILE" 2>&1 &
PIDS+=("$!")

COMMON_ARGS=(
  -p mode:=mock
  -p microphone_enabled:=false
  -p wake_word_enabled:=true
  -p continuous_control_enabled:=true
  -p voice_session_timeout_s:=60.0
  -p continuous_command_max_age_s:=0.25
  -p action_sequence_wait_timeout_s:=10.0
)

if [[ "$AGENT_KIND" == "online" ]]; then
  setsid ros2 run embodied_online_agent online_agent --ros-args \
    "${COMMON_ARGS[@]}" \
    >>"$LOG_FILE" 2>&1 &
else
  setsid ros2 run embodied_offline_agent offline_agent --ros-args \
    "${COMMON_ARGS[@]}" \
    >>"$LOG_FILE" 2>&1 &
fi
PIDS+=("$!")

activate_lifecycle_node action_guard
wait_for_topic_subscribers /robot/action_command_typed
if ! timeout 45 bash "$WORKSPACE/tests/integration/run_probe.sh" "$WORKSPACE/tests/integration/voice/test_continuous_command_ttl.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: busy continuous command -> stale queued command expired before execution ($AGENT_KIND)"
