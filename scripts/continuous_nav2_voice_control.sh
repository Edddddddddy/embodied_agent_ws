#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODE="${1:-offline}"
VOICE_CONTROL_PROFILE="${VOICE_CONTROL_PROFILE:-normal}"

PROFILE_SESSION_TIMEOUT=150
PROFILE_COMMAND_QUEUE_SIZE=6
PROFILE_COMMAND_MAX_AGE=180
PROFILE_DUPLICATE_WINDOW_S=1.2
PROFILE_COMMAND_NORMALIZATION_FUZZY_THRESHOLD=0.82
PROFILE_SPEECH_START_THRESHOLD=0.018
PROFILE_SPEECH_END_SILENCE_S=0.75
PROFILE_MIN_UTTERANCE_MS=100
PROFILE_MAX_UTTERANCE_S=12.0
PROFILE_ASR_COMMIT_DELAY_MS=300

case "$VOICE_CONTROL_PROFILE" in
  normal) ;;
  quiet)
    PROFILE_SESSION_TIMEOUT=180
    PROFILE_COMMAND_QUEUE_SIZE=8
    PROFILE_SPEECH_START_THRESHOLD=0.014
    PROFILE_SPEECH_END_SILENCE_S=0.65
    PROFILE_MIN_UTTERANCE_MS=80
    ;;
  noisy_room)
    PROFILE_SESSION_TIMEOUT=120
    PROFILE_COMMAND_QUEUE_SIZE=5
    PROFILE_COMMAND_NORMALIZATION_FUZZY_THRESHOLD=0.78
    PROFILE_SPEECH_START_THRESHOLD=0.026
    PROFILE_SPEECH_END_SILENCE_S=0.9
    PROFILE_MIN_UTTERANCE_MS=180
    PROFILE_MAX_UTTERANCE_S=10.0
    ;;
  *)
    echo "unknown VOICE_CONTROL_PROFILE=$VOICE_CONTROL_PROFILE; expected normal, quiet, or noisy_room" >&2
    exit 2
    ;;
esac

SESSION_TIMEOUT="${VOICE_SESSION_TIMEOUT:-$PROFILE_SESSION_TIMEOUT}"
COMMAND_QUEUE_SIZE="${CONTINUOUS_COMMAND_QUEUE_SIZE:-$PROFILE_COMMAND_QUEUE_SIZE}"
COMMAND_MAX_AGE="${CONTINUOUS_COMMAND_MAX_AGE:-$PROFILE_COMMAND_MAX_AGE}"
COMMAND_DUPLICATE_WINDOW="${CONTINUOUS_DUPLICATE_WINDOW_S:-$PROFILE_DUPLICATE_WINDOW_S}"
COMMAND_NORMALIZATION_ENABLED="${COMMAND_NORMALIZATION_ENABLED:-true}"
COMMAND_NORMALIZATION_FEEDBACK_ENABLED="${COMMAND_NORMALIZATION_FEEDBACK_ENABLED:-true}"
COMMAND_NORMALIZATION_FUZZY_THRESHOLD="${COMMAND_NORMALIZATION_FUZZY_THRESHOLD:-$PROFILE_COMMAND_NORMALIZATION_FUZZY_THRESHOLD}"
COMMAND_NORMALIZATION_PATH="${COMMAND_NORMALIZATION_PATH:-}"
COMMAND_COMPLETION_ENABLED="${COMMAND_COMPLETION_ENABLED:-true}"
ASR_COMMIT_DELAY_MS="${ASR_COMMIT_DELAY_MS:-$PROFILE_ASR_COMMIT_DELAY_MS}"
WAKE_WORD_ENABLED="${WAKE_WORD_ENABLED:-true}"
SPEAKER_ENABLED="${SPEAKER_ENABLED:-false}"
VAD_PROVIDER="${VAD_PROVIDER:-energy}"
SPEECH_START_THRESHOLD="${SPEECH_START_THRESHOLD:-$PROFILE_SPEECH_START_THRESHOLD}"
SPEECH_END_SILENCE_S="${SPEECH_END_SILENCE_S:-$PROFILE_SPEECH_END_SILENCE_S}"
MIN_UTTERANCE_MS="${MIN_UTTERANCE_MS:-$PROFILE_MIN_UTTERANCE_MS}"
MAX_UTTERANCE_S="${MAX_UTTERANCE_S:-$PROFILE_MAX_UTTERANCE_S}"
KWS_PROVIDER="${KWS_PROVIDER:-none}"
AUDIO_ENHANCER="${AUDIO_ENHANCER:-nlms}"
AEC_ENABLED="${AEC_ENABLED:-true}"
NOISE_SUPPRESSION_ENABLED="${NOISE_SUPPRESSION_ENABLED:-false}"
AUTO_GAIN_ENABLED="${AUTO_GAIN_ENABLED:-false}"
HEADLESS="${HEADLESS:-true}"
USE_RVIZ="${USE_RVIZ:-false}"
NAV_ACTION_TIMEOUT_S="${NAV_ACTION_TIMEOUT_S:-240.0}"
INITIAL_X="${NAV2_INITIAL_X:-${X_POSE:--2.0}}"
INITIAL_Y="${NAV2_INITIAL_Y:-${Y_POSE:--0.5}}"
INITIAL_YAW="${NAV2_INITIAL_YAW:-${YAW:-0.0}}"
MONITOR_ENABLED="${CONTINUOUS_MONITOR_ENABLED:-true}"
MONITOR_AUDIO_SAMPLE_LIMIT="${CONTINUOUS_MONITOR_AUDIO_SAMPLE_LIMIT:-600}"
PRINT_CONFIG="${CONTINUOUS_PRINT_CONFIG:-false}"
PREFLIGHT_ENABLED="${CONTINUOUS_PREFLIGHT_ENABLED:-true}"
READINESS_ENABLED="${CONTINUOUS_READINESS_ENABLED:-true}"
READINESS_DURATION="${CONTINUOUS_READINESS_DURATION:-3.0}"

source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((220 + $$ % 60))}"

if [[ "$MODE" != "offline" && "$MODE" != "online" ]]; then
  echo "Usage: $0 {offline|online}" >&2
  exit 2
fi

add_launch_arg() {
  LAUNCH_ARGS+=("$1:=$2")
}

add_optional_launch_arg() {
  [[ -z "$2" ]] || add_launch_arg "$1" "$2"
}

build_launch_args() {
  LAUNCH_ARGS=(embodied_simulation voice_nav2_turtlebot3.launch.py)
  add_launch_arg launch_agent true
  add_launch_arg agent_type "$MODE"
  add_launch_arg provider_mode "$MODE"
  add_launch_arg microphone_enabled true
  add_launch_arg capture_enabled true
  add_launch_arg speaker_enabled "$SPEAKER_ENABLED"
  add_launch_arg wake_word_enabled "$WAKE_WORD_ENABLED"
  add_launch_arg continuous_control_enabled true
  add_launch_arg voice_session_timeout_s "$SESSION_TIMEOUT"
  add_launch_arg continuous_command_queue_size "$COMMAND_QUEUE_SIZE"
  add_launch_arg continuous_command_max_age_s "$COMMAND_MAX_AGE"
  add_launch_arg continuous_duplicate_window_s "$COMMAND_DUPLICATE_WINDOW"
  add_launch_arg command_normalization_enabled "$COMMAND_NORMALIZATION_ENABLED"
  add_launch_arg command_normalization_feedback_enabled "$COMMAND_NORMALIZATION_FEEDBACK_ENABLED"
  add_launch_arg command_normalization_fuzzy_threshold "$COMMAND_NORMALIZATION_FUZZY_THRESHOLD"
  add_optional_launch_arg command_normalization_path "$COMMAND_NORMALIZATION_PATH"
  add_launch_arg command_completion_enabled "$COMMAND_COMPLETION_ENABLED"
  add_launch_arg asr_commit_delay_ms "$ASR_COMMIT_DELAY_MS"
  add_launch_arg vad_provider "$VAD_PROVIDER"
  add_launch_arg speech_start_threshold "$SPEECH_START_THRESHOLD"
  add_launch_arg speech_end_silence_s "$SPEECH_END_SILENCE_S"
  add_launch_arg min_utterance_ms "$MIN_UTTERANCE_MS"
  add_launch_arg max_utterance_s "$MAX_UTTERANCE_S"
  add_launch_arg kws_provider "$KWS_PROVIDER"
  add_launch_arg audio_enhancer "$AUDIO_ENHANCER"
  add_launch_arg aec_enabled "$AEC_ENABLED"
  add_launch_arg noise_suppression_enabled "$NOISE_SUPPRESSION_ENABLED"
  add_launch_arg auto_gain_enabled "$AUTO_GAIN_ENABLED"
  add_launch_arg use_rviz "$USE_RVIZ"
  add_launch_arg headless "$HEADLESS"
  add_launch_arg nav_action_timeout_s "$NAV_ACTION_TIMEOUT_S"
  add_launch_arg x_pose "$INITIAL_X"
  add_launch_arg y_pose "$INITIAL_Y"
  add_launch_arg yaw "$INITIAL_YAW"
}

print_configuration() {
  build_launch_args
  cat <<EOF
ROS_DOMAIN_ID=$ROS_DOMAIN_ID，Nav2 连续语音导航模式=$MODE

推荐话术：
  小智
  去门口
  前往书桌
  依次去门口、书桌、起点
  停止巡航
  退出控制

通过标准：
  终端看到 [asr] / [queue] / [action] / [result]；
  Gazebo 中 TurtleBot3 按目标点移动；
  /robot/action_result 至少出现 navigate_to 和 follow_waypoints success；
  退出控制后 session sleeping，最终 /cmd_vel 归零。

量化辅助验收：
  CONTINUOUS_LIVE_CHECK_DURATION=240 bash scripts/acceptance_test.sh continuous-nav2-live-check $MODE

VOICE_CONTROL_PROFILE=$VOICE_CONTROL_PROFILE
VOICE_SESSION_TIMEOUT=$SESSION_TIMEOUT
CONTINUOUS_COMMAND_QUEUE_SIZE=$COMMAND_QUEUE_SIZE
CONTINUOUS_COMMAND_MAX_AGE=$COMMAND_MAX_AGE
SPEECH_START_THRESHOLD=$SPEECH_START_THRESHOLD
SPEECH_END_SILENCE_S=$SPEECH_END_SILENCE_S
ASR_COMMIT_DELAY_MS=$ASR_COMMIT_DELAY_MS
NAV_ACTION_TIMEOUT_S=$NAV_ACTION_TIMEOUT_S
NAV2_INITIAL_X=$INITIAL_X
NAV2_INITIAL_Y=$INITIAL_Y
NAV2_INITIAL_YAW=$INITIAL_YAW

ros2 launch \\
EOF
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    echo "  $arg \\"
  done
  echo "  # 注：空的可选路径参数会省略。"
}

if [[ "$PRINT_CONFIG" == "true" ]]; then
  print_configuration
  exit 0
fi

if [[ "$PREFLIGHT_ENABLED" == "true" ]]; then
  bash "$WORKSPACE/scripts/smoke_test_nav2_preflight.sh"
  python3 "$WORKSPACE/scripts/voice_provider_preflight.py" \
    --mode "$MODE" \
    --vad-provider "$VAD_PROVIDER" \
    --kws-provider "$KWS_PROVIDER"
fi

if ! pactl list short sources 2>/dev/null | grep -q .; then
  echo "FAIL: WSL 中没有可用麦克风 source；请先检查 WSLg 音频权限。" >&2
  exit 1
fi

SERVER_PID=""
LAUNCH_PID=""
MONITOR_PID=""
cleanup() {
  [[ -z "$LAUNCH_PID" ]] || kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  [[ -z "$MONITOR_PID" ]] || kill -INT "$MONITOR_PID" 2>/dev/null || true
  [[ -z "$SERVER_PID" ]] || kill "$SERVER_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" "$MONITOR_PID" "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [[ "$MODE" == "offline" ]] && \
  ! curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1
then
  bash "$WORKSPACE/scripts/start_llama_server.sh" &
  SERVER_PID=$!
  for _ in $(seq 1 30); do
    if curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1; then break; fi
    sleep 1
  done
  curl -fsS http://127.0.0.1:8080/health >/dev/null || {
    echo "FAIL: llama.cpp server 未在 30 秒内启动。" >&2
    exit 1
  }
fi

print_configuration

build_launch_args
setsid ros2 launch "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

if [[ "$MONITOR_ENABLED" == "true" ]]; then
  python3 "$WORKSPACE/scripts/continuous_voice_monitor.py" \
    --audio-sample-limit "$MONITOR_AUDIO_SAMPLE_LIMIT" &
  MONITOR_PID=$!
fi

echo
echo "等待 Nav2/AMCL 订阅 /initialpose，并发布初始位姿..."
python3 "$WORKSPACE/scripts/publish_nav2_initial_pose.py" \
  --x "$INITIAL_X" --y "$INITIAL_Y" --yaw "$INITIAL_YAW"

if [[ "$READINESS_ENABLED" == "true" ]]; then
  echo
  echo "正在进行连续语音 readiness check（${READINESS_DURATION}s）..."
  if python3 "$WORKSPACE/scripts/voice_control_readiness_check.py" \
    --duration "$READINESS_DURATION"; then
    echo "系统已就绪，可以开始说：小智"
  else
    echo "WARN: readiness check 未完全通过；仍继续运行。请检查麦克风/VAD/profile。" >&2
  fi
fi

wait "$LAUNCH_PID"
