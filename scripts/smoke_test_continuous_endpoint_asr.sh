#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
AGENT_KIND="${1:-online}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/lifecycle_utils.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((190 + $$ % 20))}"

if [[ "$AGENT_KIND" != "online" && "$AGENT_KIND" != "offline" ]]; then
  echo "Usage: $0 {online|offline}" >&2
  exit 2
fi

MOCK_ASR_FINALS="小智|向前走一秒|左转|前进|后退一秒|把灯|我九十|退出"
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

if [[ "$AGENT_KIND" == "online" ]]; then
  setsid ros2 run embodied_online_agent online_agent --ros-args \
    -p mode:=mock \
    -p microphone_enabled:=true \
    -p wake_word_enabled:=true \
    -p continuous_control_enabled:=true \
    -p asr_commit_delay_ms:=100 \
    -p mock_asr_finals:="$MOCK_ASR_FINALS" \
    -p voice_session_timeout_s:=60.0 \
    -p action_sequence_wait_timeout_s:=12.0 \
    >>"$LOG_FILE" 2>&1 &
else
  setsid ros2 run embodied_offline_agent offline_agent --ros-args \
    -p mode:=mock \
    -p microphone_enabled:=true \
    -p wake_word_enabled:=true \
    -p continuous_control_enabled:=true \
    -p asr_commit_delay_ms:=100 \
    -p mock_asr_finals:="$MOCK_ASR_FINALS" \
    -p voice_session_timeout_s:=60.0 \
    -p action_sequence_wait_timeout_s:=12.0 \
    >>"$LOG_FILE" 2>&1 &
fi
PIDS+=("$!")

activate_lifecycle_node action_guard
wait_for_topic_subscribers /robot/action_command_typed
if ! timeout 55 python3 "$WORKSPACE/tests/integration/test_continuous_endpoint_asr.py"; then
  cat "$LOG_FILE" >&2
  exit 1
fi
echo "PASS: speech_ended endpoint commits drive continuous ASR commands ($AGENT_KIND)"
