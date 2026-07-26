#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WORKSPACE="${WORKSPACE:-$(cd -- "$SCRIPT_DIR/.." && pwd -P)}"
source "$SCRIPT_DIR/lifecycle_utils.sh"
embodied_resolve_runtime_root

MODE="${1:-offline}"
PROVIDER_MODE="${NAV2_PROVIDER_MODE:-$MODE}"
APPLY_VOICE_CALIBRATION="${APPLY_VOICE_CALIBRATION:-auto}"
PRINT_CONFIG="${CONTINUOUS_PRINT_CONFIG:-false}"
VOICE_CALIBRATION_ENV_EXPLICIT=false
[[ ! -v VOICE_CALIBRATION_ENV ]] || VOICE_CALIBRATION_ENV_EXPLICIT=true
LLAMA_SERVER="${LLAMA_SERVER:-$EMBODIED_RUNTIME_ROOT/third_party/llama.cpp/build/bin/llama-server}"
LLAMA_MODEL="${LLAMA_MODEL:-$EMBODIED_RUNTIME_ROOT/models/Qwen3-0.6B-Q8_0.gguf}"
VOICE_CALIBRATION_ENV="${VOICE_CALIBRATION_ENV:-$EMBODIED_RUNTIME_ROOT/logs/voice_calibration.env}"
VOICE_CALIBRATION_ENV_APPLIED=false

# Nav2/SLAM 演示必须复用普通 continuous-offline 的麦克风校准；否则同一台
# WSL 机器会在控制演示中走 low_gain/Pulse，而在建图演示中退回近静音 PortAudio。
CALIBRATION_PROTECTED_KEYS=(
  VOICE_CONTROL_PROFILE
  SPEECH_START_THRESHOLD
  VAD_SPEECH_START_MS
  SPEECH_END_SILENCE_S
  MIN_UTTERANCE_MS
  MAX_UTTERANCE_S
  ASR_COMMIT_DELAY_MS
  VAD_PROVIDER
  KWS_PROVIDER
  OPENWAKEWORD_THRESHOLD
  LIVEKIT_WAKEWORD_THRESHOLD
  AEC_ENABLED
)
CALIBRATION_EXPLICIT_KEYS=()

remember_explicit_calibration_env() {
  local key
  for key in "${CALIBRATION_PROTECTED_KEYS[@]}"; do
    if [[ -v "$key" ]]; then
      CALIBRATION_EXPLICIT_KEYS+=("$key")
      printf -v "CALIBRATION_EXPLICIT_${key}" '%s' "${!key}"
    fi
  done
}

restore_explicit_calibration_env() {
  local key value_var
  for key in "${CALIBRATION_EXPLICIT_KEYS[@]}"; do
    value_var="CALIBRATION_EXPLICIT_${key}"
    printf -v "$key" '%s' "${!value_var}"
    export "$key"
  done
}

apply_voice_calibration_env() {
  remember_explicit_calibration_env
  # 该文件由 voice_calibration_report.py 生成，只包含受控的 export KEY=value。
  # shellcheck source=/dev/null
  source "$VOICE_CALIBRATION_ENV"
  restore_explicit_calibration_env
  VOICE_CALIBRATION_ENV_APPLIED=true
}

if [[ "$APPLY_VOICE_CALIBRATION" == "true" ]]; then
  if [[ -f "$VOICE_CALIBRATION_ENV" ]]; then
    apply_voice_calibration_env
  else
    echo "WARN: APPLY_VOICE_CALIBRATION=true but VOICE_CALIBRATION_ENV not found: $VOICE_CALIBRATION_ENV" >&2
  fi
elif [[ "$APPLY_VOICE_CALIBRATION" == "auto" ]]; then
  if [[ "$PRINT_CONFIG" == "true" && "$VOICE_CALIBRATION_ENV_EXPLICIT" != "true" ]]; then
    # 纯配置审计不能被主 worktree 上的现场校准静默污染；真实启动仍默认复用校准。
    :
  else
    [[ ! -f "$VOICE_CALIBRATION_ENV" ]] || apply_voice_calibration_env
  fi
elif [[ "$APPLY_VOICE_CALIBRATION" != "false" ]]; then
  echo "unknown APPLY_VOICE_CALIBRATION=$APPLY_VOICE_CALIBRATION; expected auto, true, or false" >&2
  exit 2
fi

MICROPHONE_ENABLED="${NAV2_MICROPHONE_ENABLED:-true}"
CAPTURE_ENABLED="${NAV2_CAPTURE_ENABLED:-true}"
VOICE_CONTROL_PROFILE="${VOICE_CONTROL_PROFILE:-normal}"
# shellcheck source=voice_control_profile.sh
source "$WORKSPACE/scripts/voice_control_profile.sh"
apply_voice_control_profile_defaults "navigation" "$VOICE_CONTROL_PROFILE"

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
ASR_HOTWORDS_SCORE="${ASR_HOTWORDS_SCORE:-3.0}"
ASR_PARTIAL_MERGE_ENABLED="${ASR_PARTIAL_MERGE_ENABLED:-true}"
ASR_PARTIAL_MAX_AGE_S="${ASR_PARTIAL_MAX_AGE_S:-2.0}"
WAKE_WORD_ENABLED="${WAKE_WORD_ENABLED:-true}"
SPEAKER_ENABLED="${SPEAKER_ENABLED:-false}"
PULSE_CAPTURE_BRIDGE="${PULSE_CAPTURE_BRIDGE:-auto}"
PULSE_CAPTURE_SOURCE="${PULSE_CAPTURE_SOURCE:-@DEFAULT_SOURCE@}"
VAD_PROVIDER_REQUESTED="${VAD_PROVIDER:-auto}"
VAD_PROVIDER="$VAD_PROVIDER_REQUESTED"
SPEECH_START_THRESHOLD="${SPEECH_START_THRESHOLD:-$PROFILE_SPEECH_START_THRESHOLD}"
VAD_SPEECH_START_MS="${VAD_SPEECH_START_MS:-$PROFILE_VAD_SPEECH_START_MS}"
SPEECH_END_SILENCE_S="${SPEECH_END_SILENCE_S:-$PROFILE_SPEECH_END_SILENCE_S}"
MIN_UTTERANCE_MS="${MIN_UTTERANCE_MS:-$PROFILE_MIN_UTTERANCE_MS}"
MAX_UTTERANCE_S="${MAX_UTTERANCE_S:-$PROFILE_MAX_UTTERANCE_S}"
SILERO_VAD_MODEL_PATH="${SILERO_VAD_MODEL_PATH:-$EMBODIED_RUNTIME_ROOT/models/silero_vad/silero_vad.onnx}"
SILERO_VAD_USE_ONNX="${SILERO_VAD_USE_ONNX:-true}"
SILERO_VAD_THRESHOLD="${SILERO_VAD_THRESHOLD:-0.5}"
SILERO_VAD_END_THRESHOLD="${SILERO_VAD_END_THRESHOLD:-0.35}"
KWS_PROVIDER="${KWS_PROVIDER:-none}"
AUDIO_ENHANCER="${AUDIO_ENHANCER:-nlms}"
# 没有扬声器参考流时无需宣称 AEC active；显式打开 speaker 后仍可手工启用。
AEC_ENABLED="${AEC_ENABLED:-${PROFILE_AEC_ENABLED:-$SPEAKER_ENABLED}}"
NOISE_SUPPRESSION_ENABLED="${NOISE_SUPPRESSION_ENABLED:-false}"
AUTO_GAIN_ENABLED="${AUTO_GAIN_ENABLED:-false}"
HEADLESS="${HEADLESS:-true}"
USE_RVIZ="${USE_RVIZ:-false}"
NAV_ACTION_TIMEOUT_S="${NAV_ACTION_TIMEOUT_S:-240.0}"
INITIAL_X="${NAV2_INITIAL_X:-${X_POSE:--2.0}}"
INITIAL_Y="${NAV2_INITIAL_Y:-${Y_POSE:--0.5}}"
INITIAL_YAW="${NAV2_INITIAL_YAW:-${YAW:-0.0}}"
# Gazebo 出生位姿与 AMCL 初始位姿通常相同，但重载 SLAM 地图时两者属于
# 不同坐标系：机器人仍在 world 出生点，AMCL 则要使用以建图起点为原点的 map 坐标。
SPAWN_X="${NAV2_SPAWN_X:-$INITIAL_X}"
SPAWN_Y="${NAV2_SPAWN_Y:-$INITIAL_Y}"
SPAWN_YAW="${NAV2_SPAWN_YAW:-$INITIAL_YAW}"
NAV2_SLAM="${NAV2_SLAM:-false}"
NAV2_WORLD="${NAV2_WORLD:-}"
NAV2_MAP="${NAV2_MAP:-}"
NAV2_PARAMS_FILE="${NAV2_PARAMS_FILE:-}"
NAV2_PLACES_FILE="${NAV2_PLACES_FILE:-}"
NAV2_EXECUTOR_PLUGIN="${NAV2_EXECUTOR_PLUGIN:-embodied_simulation/Nav2RobotExecutor}"
NAV2_ENABLE_DYNAMIC_OBSTACLE_LAYER="${NAV2_ENABLE_DYNAMIC_OBSTACLE_LAYER:-false}"
# 持久会话由 StageProcessManager 显式注入该标志。base 只装配一次语音、
# Gazebo、机器人、RViz 与 Nav2 common；SLAM/AMCL executor 由阶段 launch 持有。
SHOWCASE_PERSISTENT_SESSION="${SHOWCASE_PERSISTENT_SESSION:-false}"
SHOWCASE_SESSION_DIR="${SHOWCASE_SESSION_DIR:-}"
SHOWCASE_MAP_PREFIX="${SHOWCASE_MAP_PREFIX:-}"
# 多控制源演示显式开启；旧的语音/Nav2 验收默认关闭以保持兼容。
CONTROL_AUTHORITY_ENABLED="${CONTROL_AUTHORITY_ENABLED:-false}"
# manager 与 quiescence coordinator 必须由整场 showcase 会话持有。阶段脚本
# 只消费控制权状态，不能在没有旧 Explore/Nav2 terminal 证据时自行恢复自治。
CONTROL_AUTHORITY_MANAGER_ENABLED="${CONTROL_AUTHORITY_MANAGER_ENABLED:-false}"
AUTHORITY_STATE_HEARTBEAT_MS="${AUTHORITY_STATE_HEARTBEAT_MS:-200}"
MONITOR_ENABLED="${CONTINUOUS_MONITOR_ENABLED:-true}"
MONITOR_AUDIO_SAMPLE_LIMIT="${CONTINUOUS_MONITOR_AUDIO_SAMPLE_LIMIT:-600}"
PREFLIGHT_ENABLED="${CONTINUOUS_PREFLIGHT_ENABLED:-true}"
READINESS_ENABLED="${CONTINUOUS_READINESS_ENABLED:-true}"
READINESS_DURATION="${CONTINUOUS_READINESS_DURATION:-4.0}"
# 真实麦克风演示必须以可听输入作为启动门槛；mock provider 或明确关闭
# 麦克风的确定性测试没有声学输入，因此默认保留非阻断 readiness。
READINESS_REQUIRED_DEFAULT=true
if [[ "$MICROPHONE_ENABLED" != "true" || "$PROVIDER_MODE" == "mock" ]]; then
  READINESS_REQUIRED_DEFAULT=false
fi
READINESS_REQUIRED="${CONTINUOUS_READINESS_REQUIRED:-$READINESS_REQUIRED_DEFAULT}"
SYSTEM_READINESS_TIMEOUT="${SYSTEM_READINESS_TIMEOUT:-60.0}"
SYSTEM_READINESS_STALE_TIMEOUT_S="${SYSTEM_READINESS_STALE_TIMEOUT_S:-30.0}"

# 子脚本必须复用本入口已经审计过的同一组运行时资产，不能重新退回代码 worktree。
export LLAMA_SERVER LLAMA_MODEL SILERO_VAD_MODEL_PATH VOICE_CALIBRATION_ENV

# `CONTINUOUS_PRINT_CONFIG` 是无副作用的部署审计入口；它不应强迫一个尚未
# build 的 feature worktree 先生成 install 层。真实启动仍必须完整激活 ROS。
if [[ "$PRINT_CONFIG" != "true" ]]; then
  source "$WORKSPACE/scripts/activate.sh"
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((140 + $$ % 80))}"
export GZ_PARTITION="${GZ_PARTITION:-embodied_agent_${ROS_DOMAIN_ID}}"
export IGN_PARTITION="${IGN_PARTITION:-$GZ_PARTITION}"

if [[ "$MODE" != "offline" && "$MODE" != "online" ]]; then
  echo "Usage: $0 {offline|online}" >&2
  exit 2
fi
if [[ "$CONTROL_AUTHORITY_ENABLED" != "true" && \
      "$CONTROL_AUTHORITY_ENABLED" != "false" ]]; then
  echo "FAIL: CONTROL_AUTHORITY_ENABLED must be true or false." >&2
  exit 2
fi
if [[ "$CONTROL_AUTHORITY_MANAGER_ENABLED" != "true" && \
      "$CONTROL_AUTHORITY_MANAGER_ENABLED" != "false" ]]; then
  echo "FAIL: CONTROL_AUTHORITY_MANAGER_ENABLED must be true or false." >&2
  exit 2
fi
if [[ "$SHOWCASE_PERSISTENT_SESSION" != "true" && \
      "$SHOWCASE_PERSISTENT_SESSION" != "false" ]]; then
  echo "FAIL: SHOWCASE_PERSISTENT_SESSION must be true or false." >&2
  exit 2
fi
if [[ "$SHOWCASE_PERSISTENT_SESSION" == "true" && \
      "$CONTROL_AUTHORITY_ENABLED" != "true" ]]; then
  echo "FAIL: persistent base requires CONTROL_AUTHORITY_ENABLED=true." >&2
  echo "      Start it through voice_slam_nav_showcase.sh auto/base." >&2
  exit 2
fi
if [[ "$SHOWCASE_PERSISTENT_SESSION" == "true" ]]; then
  # base 首次就绪门禁与 mapping stage 对齐；navigation 切换由编排器
  # 使用另一 exact profile 再验证，禁止复用旧阶段的 ready。
  SYSTEM_READINESS_PROFILE="persistent_mapping_stage"
  INITIAL_POSE_POLICY="stage"
elif [[ "$NAV2_SLAM" == "true" ]]; then
  SYSTEM_READINESS_PROFILE="voice_nav2"
  INITIAL_POSE_POLICY="slam"
else
  SYSTEM_READINESS_PROFILE="voice_nav2"
  INITIAL_POSE_POLICY="amcl"
fi
if [[ "$CONTROL_AUTHORITY_ENABLED" == "true" && \
      "$CONTROL_AUTHORITY_MANAGER_ENABLED" == "true" ]]; then
  echo "FAIL: stage launch cannot own control_authority without a quiescence coordinator." >&2
  echo "      Use voice_slam_nav_showcase.sh auto, which owns one session-level manager." >&2
  exit 2
fi

resolve_vad_provider() {
  if [[ "$VAD_PROVIDER_REQUESTED" != "auto" ]]; then
    VAD_PROVIDER="$VAD_PROVIDER_REQUESTED"
    return
  fi

  local report
  if ! report=$(python3 "$WORKSPACE/scripts/voice_provider_preflight.py" \
    --mode "$MODE" \
    --vad-provider auto \
    --kws-provider none \
    --silero-model-path "$SILERO_VAD_MODEL_PATH" \
    --silero-use-onnx "$SILERO_VAD_USE_ONNX" \
    --json 2>/dev/null); then
    echo "WARN: VAD_PROVIDER=auto 预检失败，降级为 energy VAD。" >&2
    VAD_PROVIDER="energy"
    return
  fi

  VAD_PROVIDER=$(printf '%s\n' "$report" | python3 -c 'import json,sys; print(json.load(sys.stdin).get("vad_provider", "energy"))')
  local vad_warnings
  vad_warnings=$(printf '%s\n' "$report" | python3 -c 'import json,sys; print("; ".join(json.load(sys.stdin).get("warnings", [])))')
  local vad_recommendations
  vad_recommendations=$(printf '%s\n' "$report" | python3 -c 'import json,sys; print("; ".join(json.load(sys.stdin).get("recommendations", [])))')
  if [[ -n "$vad_warnings" ]]; then
    echo "INFO: VAD_PROVIDER=auto -> $VAD_PROVIDER ($vad_warnings)" >&2
  else
    echo "INFO: VAD_PROVIDER=auto -> $VAD_PROVIDER" >&2
  fi
  if [[ -n "$vad_recommendations" ]]; then
    echo "INFO: mature VAD setup suggestion: $vad_recommendations" >&2
  fi
}

resolve_vad_provider
PULSE_CAPTURE_BRIDGE_ACTIVE=false
if [[ "$PULSE_CAPTURE_BRIDGE" == "true" ]]; then
  [[ "$MICROPHONE_ENABLED" != "true" ]] || PULSE_CAPTURE_BRIDGE_ACTIVE=true
elif [[ "$PULSE_CAPTURE_BRIDGE" == "auto" ]]; then
  # 确定性 Gazebo/CI 门禁会关闭麦克风；此时不能因为 WSLg 可用就额外
  # 拉起 parecord，否则既浪费资源，也会让“无声学依赖”的测试等待音频状态。
  if [[ "$MICROPHONE_ENABLED" == "true" ]] && \
    [[ -n "${PULSE_SERVER:-}" ]] && command -v parecord >/dev/null 2>&1; then
    PULSE_CAPTURE_BRIDGE_ACTIVE=true
  fi
elif [[ "$PULSE_CAPTURE_BRIDGE" != "false" ]]; then
  echo "unknown PULSE_CAPTURE_BRIDGE=$PULSE_CAPTURE_BRIDGE; expected auto, true, or false" >&2
  exit 2
fi

# Silero/WebRTC sidecar 已经持有端点状态机，Pulse bridge 此时只负责稳定采集，
# 不能再次发布 speech_ended；energy 模式才由 bridge 产生 endpoint。
PULSE_ENDPOINT_EVENTS_ENABLED=true
if [[ "$VAD_PROVIDER" == "silero" || "$VAD_PROVIDER" == "webrtc" ]]; then
  PULSE_ENDPOINT_EVENTS_ENABLED=false
fi

add_launch_arg() {
  LAUNCH_ARGS+=("$1:=$2")
}

add_optional_launch_arg() {
  [[ -z "$2" ]] || add_launch_arg "$1" "$2"
}

build_launch_args() {
  if [[ "$SHOWCASE_PERSISTENT_SESSION" == "true" ]]; then
    LAUNCH_ARGS=(embodied_simulation persistent_voice_nav_base.launch.py)
  else
    LAUNCH_ARGS=(embodied_simulation voice_nav2_turtlebot3.launch.py)
  fi
  local effective_capture_enabled="$CAPTURE_ENABLED"
  if [[ "$PULSE_CAPTURE_BRIDGE_ACTIVE" == "true" ]]; then
    effective_capture_enabled=false
  fi
  add_launch_arg launch_agent true
  add_launch_arg agent_type "$MODE"
  add_launch_arg provider_mode "$PROVIDER_MODE"
  add_launch_arg microphone_enabled "$MICROPHONE_ENABLED"
  add_launch_arg capture_enabled "$effective_capture_enabled"
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
  add_launch_arg asr_hotwords_score "$ASR_HOTWORDS_SCORE"
  add_launch_arg asr_partial_merge_enabled "$ASR_PARTIAL_MERGE_ENABLED"
  add_launch_arg asr_partial_max_age_s "$ASR_PARTIAL_MAX_AGE_S"
  add_launch_arg vad_provider "$VAD_PROVIDER"
  add_launch_arg speech_start_threshold "$SPEECH_START_THRESHOLD"
  add_launch_arg vad_speech_start_ms "$VAD_SPEECH_START_MS"
  add_launch_arg speech_end_silence_s "$SPEECH_END_SILENCE_S"
  add_launch_arg min_utterance_ms "$MIN_UTTERANCE_MS"
  add_launch_arg max_utterance_s "$MAX_UTTERANCE_S"
  add_optional_launch_arg silero_model_path "$SILERO_VAD_MODEL_PATH"
  add_launch_arg silero_use_onnx "$SILERO_VAD_USE_ONNX"
  add_launch_arg silero_threshold "$SILERO_VAD_THRESHOLD"
  add_launch_arg silero_end_threshold "$SILERO_VAD_END_THRESHOLD"
  add_launch_arg kws_provider "$KWS_PROVIDER"
  add_launch_arg audio_enhancer "$AUDIO_ENHANCER"
  add_launch_arg aec_enabled "$AEC_ENABLED"
  add_launch_arg noise_suppression_enabled "$NOISE_SUPPRESSION_ENABLED"
  add_launch_arg auto_gain_enabled "$AUTO_GAIN_ENABLED"
  add_launch_arg use_rviz "$USE_RVIZ"
  add_launch_arg headless "$HEADLESS"
  add_launch_arg control_authority_enabled "$CONTROL_AUTHORITY_ENABLED"
  add_optional_launch_arg world "$NAV2_WORLD"
  add_optional_launch_arg params_file "$NAV2_PARAMS_FILE"
  add_launch_arg x_pose "$SPAWN_X"
  add_launch_arg y_pose "$SPAWN_Y"
  add_launch_arg yaw "$SPAWN_YAW"
  if [[ "$SHOWCASE_PERSISTENT_SESSION" == "true" ]]; then
    # partition 必须显式传入 launch；只依赖 shell 默认值会让阶段进程在手工
    # 调试时意外连到另一个 Gazebo world。
    add_launch_arg gz_partition "$GZ_PARTITION"
  else
    add_launch_arg readiness_stale_timeout_s "$SYSTEM_READINESS_STALE_TIMEOUT_S"
    add_launch_arg nav_action_timeout_s "$NAV_ACTION_TIMEOUT_S"
    add_launch_arg authority_state_heartbeat_ms "$AUTHORITY_STATE_HEARTBEAT_MS"
    add_launch_arg executor_plugin "$NAV2_EXECUTOR_PLUGIN"
    add_launch_arg enable_dynamic_obstacle_layer "$NAV2_ENABLE_DYNAMIC_OBSTACLE_LAYER"
    add_launch_arg control_authority_manager_enabled "$CONTROL_AUTHORITY_MANAGER_ENABLED"
    add_launch_arg slam "$NAV2_SLAM"
    add_optional_launch_arg map "$NAV2_MAP"
  fi
}

print_configuration() {
  build_launch_args
  cat <<EOF
ROS_DOMAIN_ID=$ROS_DOMAIN_ID，Nav2 连续语音导航模式=$MODE
GZ_PARTITION=$GZ_PARTITION（隔离 Gazebo Transport，避免残留世界抢占 /clock）
SHOWCASE_PERSISTENT_SESSION=$SHOWCASE_PERSISTENT_SESSION
SHOWCASE_SESSION_DIR=${SHOWCASE_SESSION_DIR:-<legacy>}
SHOWCASE_MAP_PREFIX=${SHOWCASE_MAP_PREFIX:-<legacy>}
SYSTEM_READINESS_PROFILE=$SYSTEM_READINESS_PROFILE
INITIAL_POSE_POLICY=$INITIAL_POSE_POLICY
PROVIDER_MODE=$PROVIDER_MODE
MICROPHONE_ENABLED=$MICROPHONE_ENABLED
CAPTURE_ENABLED=$CAPTURE_ENABLED
EMBODIED_RUNTIME_ROOT=$EMBODIED_RUNTIME_ROOT
LLAMA_SERVER=$LLAMA_SERVER
LLAMA_MODEL=$LLAMA_MODEL

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
APPLY_VOICE_CALIBRATION=$APPLY_VOICE_CALIBRATION
VOICE_CALIBRATION_ENV=$VOICE_CALIBRATION_ENV（applied=$VOICE_CALIBRATION_ENV_APPLIED）
VOICE_SESSION_TIMEOUT=$SESSION_TIMEOUT
CONTINUOUS_COMMAND_QUEUE_SIZE=$COMMAND_QUEUE_SIZE
CONTINUOUS_COMMAND_MAX_AGE=$COMMAND_MAX_AGE
SPEECH_START_THRESHOLD=$SPEECH_START_THRESHOLD
VAD_SPEECH_START_MS=$VAD_SPEECH_START_MS
SPEECH_END_SILENCE_S=$SPEECH_END_SILENCE_S
VAD_PROVIDER=$VAD_PROVIDER（requested=$VAD_PROVIDER_REQUESTED；auto 会优先 Silero，其次 WebRTC，最后降级 energy）
SILERO_VAD_MODEL_PATH=$SILERO_VAD_MODEL_PATH
SILERO_VAD_USE_ONNX=$SILERO_VAD_USE_ONNX
SILERO_VAD_THRESHOLD=$SILERO_VAD_THRESHOLD
SILERO_VAD_END_THRESHOLD=$SILERO_VAD_END_THRESHOLD
KWS_PROVIDER=$KWS_PROVIDER
ASR_COMMIT_DELAY_MS=$ASR_COMMIT_DELAY_MS
ASR_HOTWORDS_SCORE=$ASR_HOTWORDS_SCORE
ASR_PARTIAL_MERGE_ENABLED=$ASR_PARTIAL_MERGE_ENABLED
ASR_PARTIAL_MAX_AGE_S=$ASR_PARTIAL_MAX_AGE_S
PULSE_CAPTURE_BRIDGE=$PULSE_CAPTURE_BRIDGE（active=$PULSE_CAPTURE_BRIDGE_ACTIVE）
PULSE_CAPTURE_SOURCE=$PULSE_CAPTURE_SOURCE
PULSE_ENDPOINT_EVENTS_ENABLED=$PULSE_ENDPOINT_EVENTS_ENABLED
AEC_ENABLED=$AEC_ENABLED
CONTINUOUS_READINESS_ENABLED=$READINESS_ENABLED
CONTINUOUS_READINESS_DURATION=$READINESS_DURATION
CONTINUOUS_READINESS_REQUIRED=$READINESS_REQUIRED
NAV_ACTION_TIMEOUT_S=$NAV_ACTION_TIMEOUT_S
SYSTEM_READINESS_STALE_TIMEOUT_S=$SYSTEM_READINESS_STALE_TIMEOUT_S
NAV2_INITIAL_X=$INITIAL_X
NAV2_INITIAL_Y=$INITIAL_Y
NAV2_INITIAL_YAW=$INITIAL_YAW
NAV2_SPAWN_X=$SPAWN_X
NAV2_SPAWN_Y=$SPAWN_Y
NAV2_SPAWN_YAW=$SPAWN_YAW
NAV2_SLAM=$NAV2_SLAM
NAV2_WORLD=${NAV2_WORLD:-<default>}
NAV2_MAP=${NAV2_MAP:-<default>}
NAV2_PARAMS_FILE=${NAV2_PARAMS_FILE:-<default>}
NAV2_PLACES_FILE=${NAV2_PLACES_FILE:-<default>}
NAV2_EXECUTOR_PLUGIN=$NAV2_EXECUTOR_PLUGIN
CONTROL_AUTHORITY_ENABLED=$CONTROL_AUTHORITY_ENABLED
CONTROL_AUTHORITY_MANAGER_ENABLED=$CONTROL_AUTHORITY_MANAGER_ENABLED
AUTHORITY_STATE_HEARTBEAT_MS=$AUTHORITY_STATE_HEARTBEAT_MS

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

wait_for_external_authority_manager() {
  [[ "$CONTROL_AUTHORITY_ENABLED" == "true" ]] || return 0
  local attempt set_type ack_type coordinator_type
  for attempt in $(seq 1 30); do
    set_type="$(ros2 service type /control/set_authority 2>/dev/null || true)"
    ack_type="$(ros2 service type /control/acknowledge_autonomy_quiescence 2>/dev/null || true)"
    coordinator_type="$(ros2 topic type /slam/session_state 2>/dev/null || true)"
    if [[ "$set_type" == "embodied_agent_interfaces/srv/SetControlAuthority" && \
          "$ack_type" == "embodied_agent_interfaces/srv/AcknowledgeAutonomyQuiescence" && \
          "$coordinator_type" == "embodied_agent_interfaces/msg/SlamSessionState" ]]; then
      return 0
    fi
    sleep 0.2
  done
  echo "FAIL: control authority enabled, but typed manager/coordinator readiness is incomplete." >&2
  echo "      Expected /control services and /slam/session_state from the session orchestrator." >&2
  echo "      Start this stage through: bash scripts/voice_slam_nav_showcase.sh auto $MODE" >&2
  return 1
}

wait_for_external_authority_manager

if [[ "$PREFLIGHT_ENABLED" == "true" ]]; then
  bash "$WORKSPACE/scripts/smoke_test_nav2_preflight.sh"
  python3 "$WORKSPACE/scripts/voice_provider_preflight.py" \
    --mode "$MODE" \
    --vad-provider "$VAD_PROVIDER" \
    --kws-provider "$KWS_PROVIDER"
fi

if [[ "$MICROPHONE_ENABLED" == "true" ]] && \
  ! pactl list short sources 2>/dev/null | grep -q .; then
  echo "FAIL: WSL 中没有可用麦克风 source；请先检查 WSLg 音频权限。" >&2
  exit 1
fi

SERVER_PID=""
LAUNCH_PID=""
MONITOR_PID=""
PULSE_BRIDGE_PID=""
cleanup() {
  [[ -z "$LAUNCH_PID" ]] || kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
  [[ -z "$MONITOR_PID" ]] || kill -INT "$MONITOR_PID" 2>/dev/null || true
  [[ -z "$PULSE_BRIDGE_PID" ]] || kill -INT "$PULSE_BRIDGE_PID" 2>/dev/null || true
  [[ -z "$SERVER_PID" ]] || kill "$SERVER_PID" 2>/dev/null || true
  wait "$LAUNCH_PID" "$MONITOR_PID" "$PULSE_BRIDGE_PID" "$SERVER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

start_pulse_capture_bridge() {
  [[ "$PULSE_CAPTURE_BRIDGE_ACTIVE" == "true" ]] || return 0
  python3 "$WORKSPACE/scripts/pulse_audio_capture_bridge.py" \
    --source "$PULSE_CAPTURE_SOURCE" \
    --sample-rate 16000 \
    --frame-ms 20 \
    --vad-rms-threshold "$SPEECH_START_THRESHOLD" \
    --speech-end-silence-s "$SPEECH_END_SILENCE_S" \
    --min-utterance-ms "$MIN_UTTERANCE_MS" \
    --max-utterance-s "$MAX_UTTERANCE_S" \
    --endpoint-events-enabled "$PULSE_ENDPOINT_EVENTS_ENABLED" &
  PULSE_BRIDGE_PID=$!

  # 只有 bridge 真正连上所选 Pulse source 后才关闭 PortAudio。parecord 的
  # source 无效时 Python 进程会立即退出；这里 fail-fast，避免 Gazebo 已启动
  # 却始终没有 /audio/clean_pcm、现场表现成“系统卡住”。
  local probe
  for probe in $(seq 1 10); do
    sleep 0.1
    if ! kill -0 "$PULSE_BRIDGE_PID" 2>/dev/null; then
      local exit_code=0
      wait "$PULSE_BRIDGE_PID" || exit_code=$?
      PULSE_BRIDGE_PID=""
      echo "FAIL: PulseAudio capture bridge startup failed (exit=$exit_code, source=$PULSE_CAPTURE_SOURCE)." >&2
      echo "      请检查 pactl list short sources，或设置 PULSE_CAPTURE_BRIDGE=false 回退 PortAudio。" >&2
      return 1
    fi
  done
  echo "[audio] PulseAudio capture bridge startup check passed: source=$PULSE_CAPTURE_SOURCE"
}

# offline Agent 只有真实 llama.cpp provider 才需要独立 server；自动化/CI 的 mock
# provider 必须能在没有模型资产时启动同一 ROS/Nav2 拓扑。
if [[ "$MODE" == "offline" && "$PROVIDER_MODE" != "mock" ]] && \
  ! curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1
then
  bash "$WORKSPACE/scripts/start_llama_server.sh" &
  SERVER_PID=$!
  for _ in $(seq 1 30); do
    if curl -fsS http://127.0.0.1:8080/health >/dev/null 2>&1; then
      break
    fi
    if ! kill -0 "$SERVER_PID" 2>/dev/null; then
      server_exit=0
      wait "$SERVER_PID" || server_exit=$?
      SERVER_PID=""
      # 缺二进制、缺模型或动态库错误都应立即返回原始退出码，不能伪装成 30 秒健康超时。
      echo "FAIL: llama.cpp server 进程提前退出 (exit=$server_exit)。" >&2
      exit 1
    fi
    sleep 1
  done
  curl -fsS http://127.0.0.1:8080/health >/dev/null || {
    echo "FAIL: llama.cpp server 未在 30 秒内启动。" >&2
    exit 1
  }
fi

print_configuration

start_pulse_capture_bridge
build_launch_args
if [[ -n "$NAV2_PLACES_FILE" ]]; then
  export EMBODIED_NAV2_PLACES_FILE="$NAV2_PLACES_FILE"
fi
setsid ros2 launch "${LAUNCH_ARGS[@]}" &
LAUNCH_PID=$!

if [[ "$MONITOR_ENABLED" == "true" ]]; then
  python3 "$WORKSPACE/scripts/continuous_voice_monitor.py" \
    --audio-sample-limit "$MONITOR_AUDIO_SAMPLE_LIMIT" &
  MONITOR_PID=$!
fi

if [[ "$INITIAL_POSE_POLICY" == "amcl" ]]; then
  echo
  echo "等待 Nav2/AMCL 订阅 /initialpose，并发布初始位姿..."
  python3 "$WORKSPACE/scripts/publish_nav2_initial_pose.py" \
    --x "$INITIAL_X" --y "$INITIAL_Y" --yaw "$INITIAL_YAW"
elif [[ "$INITIAL_POSE_POLICY" == "slam" ]]; then
  echo
  echo "SLAM mapping 模式由 slam_toolbox 发布 map->odom，不向 AMCL 发布 /initialpose。"
else
  echo
  echo "持久 base 不拥有定位 provider；/initialpose 由 navigation stage 在 AMCL 启动后发布。"
fi

echo
echo "正在等待语音/Nav2 组件就绪（typed system readiness）..."
python3 "$WORKSPACE/scripts/system_readiness_check.py" \
  --timeout "$SYSTEM_READINESS_TIMEOUT" --profile "$SYSTEM_READINESS_PROFILE"

if [[ "$READINESS_ENABLED" == "true" ]]; then
  readiness_args=(--duration "$READINESS_DURATION")
  if [[ "$READINESS_REQUIRED" == "true" ]]; then
    readiness_args+=(--require-speech)
  fi
  echo
  echo "正在进行连续语音 readiness check（${READINESS_DURATION}s）..."
  if [[ "$READINESS_REQUIRED" == "true" ]]; then
    echo "请在接下来的 ${READINESS_DURATION}s 采样窗口内持续说完整话，例如：小智，去门口；不要安静等待。"
  fi
  if python3 "$WORKSPACE/scripts/voice_control_readiness_check.py" \
    "${readiness_args[@]}"; then
    echo "系统已就绪，可以开始说：小智"
  else
    if [[ "$READINESS_REQUIRED" == "true" ]]; then
      echo "FAIL: 真实麦克风 readiness 未通过；为避免把近静音或残缺 ASR 送入导航队列，当前停止 continuous-nav2-$MODE。" >&2
      echo "      请现在对着麦克风清晰说一句完整指令，例如：小智，去门口；确认有声音后重新运行。" >&2
      exit 1
    fi
    echo "WARN: readiness check 未完全通过；仍继续运行。请检查麦克风/VAD/profile。" >&2
  fi
fi

wait "$LAUNCH_PID"
