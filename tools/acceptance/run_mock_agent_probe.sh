#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WORKSPACE="${WORKSPACE:-$(cd -- "$SCRIPT_DIR/../.." && pwd -P)}"
SCENARIO="${1:-}"
AGENT_KIND="${2:-online}"

source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/lifecycle_utils.sh"

if [[ "$AGENT_KIND" != "online" && "$AGENT_KIND" != "offline" ]]; then
  echo "Usage: $0 SCENARIO {online|offline}" >&2
  exit 2
fi

# 每个场景只声明差异；ROS 图、Agent 启动和进程组清理由本 runner 统一负责。
# 这样新增队列策略时只需要增加 probe 与一条配置，不再复制整套启动脚手架。
PROBE=""
PROBE_TIMEOUT_S=45
DOMAIN_BASE=180
DOMAIN_SPAN=20
VOICE_SESSION_TIMEOUT_S=60.0
ACTION_WAIT_TIMEOUT_S=10.0
MICROPHONE_ENABLED=false
START_KEYWORD_WAKE=false
PASS_MESSAGE=""
EXTRA_AGENT_ARGS=()

case "$SCENARIO" in
  continuous)
    PROBE="tools/acceptance/probes/voice/continuous_voice_control.py"
    PROBE_TIMEOUT_S=35
    DOMAIN_BASE=200
    PASS_MESSAGE="one wake word -> queued commands -> priority stop -> zero cmd_vel"
    ;;
  soak)
    PROBE="tools/acceptance/probes/voice/continuous_voice_soak.py"
    PROBE_TIMEOUT_S=70
    DOMAIN_BASE=210
    VOICE_SESSION_TIMEOUT_S=90.0
    ACTION_WAIT_TIMEOUT_S=12.0
    EXTRA_AGENT_ARGS+=(-p continuous_command_queue_size:=16)
    PASS_MESSAGE="long continuous session keeps accepting queued commands"
    ;;
  endpoint)
    PROBE="tools/acceptance/probes/voice/continuous_endpoint_asr.py"
    PROBE_TIMEOUT_S=55
    DOMAIN_BASE=190
    ACTION_WAIT_TIMEOUT_S=12.0
    MICROPHONE_ENABLED=true
    EXTRA_AGENT_ARGS+=(
      -p asr_commit_delay_ms:=100
      -p "mock_asr_finals:=小智|向前走一秒|左转|前进|后退一秒|把灯|把灯|我九十|退出"
      -p "mock_asr_partials:=-|-|-|-|-|把灯设成蓝色|-|-|-"
    )
    PASS_MESSAGE="speech_ended endpoint commits drive continuous ASR commands"
    ;;
  multi-command)
    PROBE="tools/acceptance/probes/voice/continuous_multi_command.py"
    PROBE_TIMEOUT_S=45
    PASS_MESSAGE="NLU multi-command ASR final -> queued ordered actions"
    ;;
  queue-full)
    PROBE="tools/acceptance/probes/voice/continuous_queue_full.py"
    PROBE_TIMEOUT_S=35
    DOMAIN_BASE=220
    DOMAIN_SPAN=10
    EXTRA_AGENT_ARGS+=(
      -p continuous_command_queue_size:=1
      -p continuous_command_max_age_s:=30.0
    )
    PASS_MESSAGE="bounded continuous queue publishes queue_full rejection and feedback"
    ;;
  ttl)
    PROBE="tools/acceptance/probes/voice/continuous_command_ttl.py"
    DOMAIN_BASE=210
    EXTRA_AGENT_ARGS+=(-p continuous_command_max_age_s:=0.25)
    PASS_MESSAGE="busy continuous command -> stale queued command expired before execution"
    ;;
  timeout)
    PROBE="tools/acceptance/probes/voice/continuous_session_timeout.py"
    PROBE_TIMEOUT_S=35
    DOMAIN_BASE=230
    DOMAIN_SPAN=3
    VOICE_SESSION_TIMEOUT_S=0.8
    PASS_MESSAGE="continuous session timeout rejects stale commands and accepts a fresh wake"
    ;;
  kws)
    PROBE="tools/acceptance/probes/voice/continuous_kws_sidecar.py"
    PROBE_TIMEOUT_S=35
    START_KEYWORD_WAKE=true
    PASS_MESSAGE="keyword_wake sidecar -> Agent session -> robot action"
    ;;
  navigation)
    PROBE="tools/acceptance/probes/slam_nav/continuous_navigation_queue.py"
    PROBE_TIMEOUT_S=55
    DOMAIN_BASE=175
    ACTION_WAIT_TIMEOUT_S=18.0
    PASS_MESSAGE="continuous voice -> queued target navigation and waypoint patrol"
    ;;
  navigation-natural)
    PROBE="tools/acceptance/probes/slam_nav/continuous_navigation_natural.py"
    PROBE_TIMEOUT_S=55
    DOMAIN_BASE=195
    ACTION_WAIT_TIMEOUT_S=18.0
    PASS_MESSAGE="natural speech -> waypoint patrol queue"
    ;;
  *)
    echo "Unknown mock Agent scenario: $SCENARIO" >&2
    echo "Available: continuous soak endpoint multi-command queue-full ttl timeout kws navigation navigation-natural" >&2
    exit 2
    ;;
esac

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((DOMAIN_BASE + $$ % DOMAIN_SPAN))}"
LOG_FILE="$(mktemp)"
PIDS=()

cleanup() {
  set +e
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  sleep 0.2
  for pid in "${PIDS[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  rm -f "$LOG_FILE"
  return 0
}
trap cleanup EXIT

setsid ros2 launch embodied_simulation simulation_control.launch.py \
  use_typed_actions:=true \
  executor_plugin:=embodied_simulation/MockRobotExecutor \
  >"$LOG_FILE" 2>&1 &
PIDS+=("$!")

setsid ros2 run embodied_agent_cpp action_guard >>"$LOG_FILE" 2>&1 &
PIDS+=("$!")

if [[ "$START_KEYWORD_WAKE" == "true" ]]; then
  setsid ros2 run embodied_voice_frontend keyword_wake --ros-args \
    -p mode:=mock_text \
    -p provider_name:=mock_kws \
    >>"$LOG_FILE" 2>&1 &
  PIDS+=("$!")
fi

AGENT_ARGS=(
  -p mode:=mock
  -p microphone_enabled:="$MICROPHONE_ENABLED"
  -p wake_word_enabled:=true
  -p continuous_control_enabled:=true
  -p voice_session_timeout_s:="$VOICE_SESSION_TIMEOUT_S"
  -p action_sequence_wait_timeout_s:="$ACTION_WAIT_TIMEOUT_S"
  "${EXTRA_AGENT_ARGS[@]}"
)

if [[ "$AGENT_KIND" == "online" ]]; then
  setsid ros2 run embodied_online_agent online_agent --ros-args \
    "${AGENT_ARGS[@]}" >>"$LOG_FILE" 2>&1 &
else
  setsid ros2 run embodied_offline_agent offline_agent --ros-args \
    "${AGENT_ARGS[@]}" >>"$LOG_FILE" 2>&1 &
fi
PIDS+=("$!")

if ! activate_lifecycle_node action_guard; then
  cat "$LOG_FILE" >&2
  exit 1
fi
if ! wait_for_topic_subscribers /robot/action_command_typed; then
  cat "$LOG_FILE" >&2
  exit 1
fi
if ! timeout "$PROBE_TIMEOUT_S" bash "$WORKSPACE/tools/acceptance/run_probe.sh" \
  "$WORKSPACE/$PROBE"; then
  cat "$LOG_FILE" >&2
  exit 1
fi

echo "PASS: $PASS_MESSAGE ($AGENT_KIND)"
