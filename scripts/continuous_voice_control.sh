#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODE="${1:-offline}"
APPLY_VOICE_CALIBRATION="${APPLY_VOICE_CALIBRATION:-auto}"
VOICE_CALIBRATION_ENV="${VOICE_CALIBRATION_ENV:-$WORKSPACE/logs/voice_calibration.env}"
VOICE_CALIBRATION_ENV_APPLIED=false

CALIBRATION_PROTECTED_KEYS=(
  VOICE_CONTROL_PROFILE
  SPEECH_START_THRESHOLD
  VAD_SPEECH_START_MS
  SPEECH_END_SILENCE_S
  MIN_UTTERANCE_MS
  MAX_UTTERANCE_S
  ASR_COMMIT_DELAY_MS
  VAD_PROVIDER
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
  if [[ -f "$VOICE_CALIBRATION_ENV" ]]; then
    # 该文件由 voice_calibration_report.py 生成，只包含 export KEY=value。
    # 在 profile 计算前加载，才能让推荐的 VOICE_CONTROL_PROFILE/SPEECH_START_THRESHOLD 生效。
    # auto 模式下仍保留用户显式传入的关键环境变量，避免旧校准文件覆盖现场手工调参。
    remember_explicit_calibration_env
    # shellcheck source=/dev/null
    source "$VOICE_CALIBRATION_ENV"
    restore_explicit_calibration_env
    VOICE_CALIBRATION_ENV_APPLIED=true
  fi
}

if [[ "$APPLY_VOICE_CALIBRATION" == "true" ]]; then
  if [[ -f "$VOICE_CALIBRATION_ENV" ]]; then
    apply_voice_calibration_env
  else
    echo "WARN: APPLY_VOICE_CALIBRATION=true but VOICE_CALIBRATION_ENV not found: $VOICE_CALIBRATION_ENV" >&2
  fi
elif [[ "$APPLY_VOICE_CALIBRATION" == "auto" ]]; then
  if [[ -f "$VOICE_CALIBRATION_ENV" ]]; then
    apply_voice_calibration_env
  fi
elif [[ "$APPLY_VOICE_CALIBRATION" != "false" ]]; then
  echo "unknown APPLY_VOICE_CALIBRATION=$APPLY_VOICE_CALIBRATION; expected auto, true, or false" >&2
  exit 2
fi
VOICE_CONTROL_PROFILE="${VOICE_CONTROL_PROFILE:-normal}"
# 真实麦克风现场通常没有时间逐项调 VAD/队列/纠错参数，因此提供几个预设档。
# 下面的 PROFILE_* 只作为默认值；用户显式传入的环境变量会在 case 之后覆盖它们。
PROFILE_SESSION_TIMEOUT=60
PROFILE_COMMAND_QUEUE_SIZE=8
PROFILE_COMMAND_MAX_AGE=30
PROFILE_DUPLICATE_WINDOW_S=1.2
PROFILE_COMMAND_NORMALIZATION_FUZZY_THRESHOLD=0.82
PROFILE_SPEECH_START_THRESHOLD=0.018
PROFILE_VAD_SPEECH_START_MS=96
PROFILE_SPEECH_END_SILENCE_S=0.7
PROFILE_MIN_UTTERANCE_MS=100
PROFILE_MAX_UTTERANCE_S=12.0
PROFILE_ASR_COMMIT_DELAY_MS=300

case "$VOICE_CONTROL_PROFILE" in
  normal)
    ;;
  quiet)
    PROFILE_SESSION_TIMEOUT=75
    PROFILE_COMMAND_QUEUE_SIZE=10
    PROFILE_COMMAND_NORMALIZATION_FUZZY_THRESHOLD=0.82
    PROFILE_SPEECH_START_THRESHOLD=0.014
    PROFILE_VAD_SPEECH_START_MS=64
    PROFILE_SPEECH_END_SILENCE_S=0.6
    PROFILE_MIN_UTTERANCE_MS=80
    PROFILE_MAX_UTTERANCE_S=12.0
    ;;
  low_gain)
    PROFILE_SESSION_TIMEOUT=75
    PROFILE_COMMAND_QUEUE_SIZE=10
    PROFILE_COMMAND_NORMALIZATION_FUZZY_THRESHOLD=0.82
    PROFILE_SPEECH_START_THRESHOLD=0.0012
    PROFILE_SPEECH_END_SILENCE_S=0.8
    PROFILE_MIN_UTTERANCE_MS=120
    PROFILE_MAX_UTTERANCE_S=12.0
    PROFILE_ASR_COMMIT_DELAY_MS=450
    PROFILE_AEC_ENABLED=false
    ;;
  noisy_room)
    PROFILE_SESSION_TIMEOUT=45
    PROFILE_COMMAND_QUEUE_SIZE=5
    PROFILE_COMMAND_MAX_AGE=20
    PROFILE_COMMAND_NORMALIZATION_FUZZY_THRESHOLD=0.78
    PROFILE_SPEECH_START_THRESHOLD=0.026
    PROFILE_VAD_SPEECH_START_MS=160
    PROFILE_SPEECH_END_SILENCE_S=0.85
    PROFILE_MIN_UTTERANCE_MS=180
    PROFILE_MAX_UTTERANCE_S=10.0
    ;;
  *)
    echo "unknown VOICE_CONTROL_PROFILE=$VOICE_CONTROL_PROFILE; expected normal, quiet, low_gain, or noisy_room" >&2
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
PULSE_CAPTURE_BRIDGE="${PULSE_CAPTURE_BRIDGE:-auto}"
PULSE_CAPTURE_SOURCE="${PULSE_CAPTURE_SOURCE:-@DEFAULT_SOURCE@}"
VAD_PROVIDER_REQUESTED="${VAD_PROVIDER:-auto}"
VAD_PROVIDER="$VAD_PROVIDER_REQUESTED"
SPEECH_START_THRESHOLD="${SPEECH_START_THRESHOLD:-$PROFILE_SPEECH_START_THRESHOLD}"
VAD_SPEECH_START_MS="${VAD_SPEECH_START_MS:-$PROFILE_VAD_SPEECH_START_MS}"
SPEECH_END_SILENCE_S="${SPEECH_END_SILENCE_S:-$PROFILE_SPEECH_END_SILENCE_S}"
MIN_UTTERANCE_MS="${MIN_UTTERANCE_MS:-$PROFILE_MIN_UTTERANCE_MS}"
MAX_UTTERANCE_S="${MAX_UTTERANCE_S:-$PROFILE_MAX_UTTERANCE_S}"
SILERO_VAD_MODEL_PATH="${SILERO_VAD_MODEL_PATH:-$WORKSPACE/models/silero_vad/silero_vad.onnx}"
SILERO_VAD_USE_ONNX="${SILERO_VAD_USE_ONNX:-true}"
SILERO_VAD_THRESHOLD="${SILERO_VAD_THRESHOLD:-0.5}"
SILERO_VAD_END_THRESHOLD="${SILERO_VAD_END_THRESHOLD:-0.35}"
KWS_PROVIDER="${KWS_PROVIDER:-none}"
SHERPA_KWS_TOKENS="${SHERPA_KWS_TOKENS:-}"
SHERPA_KWS_ENCODER="${SHERPA_KWS_ENCODER:-}"
SHERPA_KWS_DECODER="${SHERPA_KWS_DECODER:-}"
SHERPA_KWS_JOINER="${SHERPA_KWS_JOINER:-}"
SHERPA_KWS_KEYWORDS_FILE="${SHERPA_KWS_KEYWORDS_FILE:-}"
OPENWAKEWORD_MODELS="${OPENWAKEWORD_MODELS:-}"
OPENWAKEWORD_THRESHOLD="${OPENWAKEWORD_THRESHOLD:-0.5}"
LIVEKIT_WAKEWORD_MODELS="${LIVEKIT_WAKEWORD_MODELS:-}"
LIVEKIT_WAKEWORD_THRESHOLD="${LIVEKIT_WAKEWORD_THRESHOLD:-0.5}"
AUDIO_ENHANCER="${AUDIO_ENHANCER:-nlms}"
AEC_ENABLED="${AEC_ENABLED:-${PROFILE_AEC_ENABLED:-true}}"
NOISE_SUPPRESSION_ENABLED="${NOISE_SUPPRESSION_ENABLED:-false}"
AUTO_GAIN_ENABLED="${AUTO_GAIN_ENABLED:-false}"
GUI_ENABLED="${GUI_ENABLED:-true}"
MONITOR_ENABLED="${CONTINUOUS_MONITOR_ENABLED:-true}"
MONITOR_AUDIO_SAMPLE_LIMIT="${CONTINUOUS_MONITOR_AUDIO_SAMPLE_LIMIT:-600}"
MONITOR_SAMPLE_LOG="${CONTINUOUS_SAMPLE_LOG:-}"
PRINT_CONFIG="${CONTINUOUS_PRINT_CONFIG:-false}"
PREFLIGHT_ENABLED="${CONTINUOUS_PREFLIGHT_ENABLED:-true}"
READINESS_ENABLED="${CONTINUOUS_READINESS_ENABLED:-true}"
READINESS_DURATION="${CONTINUOUS_READINESS_DURATION:-3.0}"
SIMULATION_READINESS_ENABLED="${SIMULATION_READINESS_ENABLED:-true}"
SIMULATION_READINESS_REQUIRED="${SIMULATION_READINESS_REQUIRED:-true}"
SIMULATION_READINESS_TIMEOUT="${SIMULATION_READINESS_TIMEOUT:-35.0}"
SIMULATION_CLEANUP_STALE="${SIMULATION_CLEANUP_STALE:-false}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((140 + $$ % 80))}"

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

PULSE_CAPTURE_BRIDGE_ACTIVE=false
if [[ "$PULSE_CAPTURE_BRIDGE" == "true" ]]; then
  PULSE_CAPTURE_BRIDGE_ACTIVE=true
elif [[ "$PULSE_CAPTURE_BRIDGE" == "auto" ]]; then
  if [[ -n "${PULSE_SERVER:-}" ]] && command -v parecord >/dev/null 2>&1; then
    PULSE_CAPTURE_BRIDGE_ACTIVE=true
  fi
elif [[ "$PULSE_CAPTURE_BRIDGE" != "false" ]]; then
  echo "unknown PULSE_CAPTURE_BRIDGE=$PULSE_CAPTURE_BRIDGE; expected auto, true, or false" >&2
  exit 2
fi

if [[ "$MODE" != "offline" && "$MODE" != "online" ]]; then
  echo "Usage: $0 {offline|online}" >&2
  exit 2
fi

resolve_vad_provider
PULSE_ENDPOINT_EVENTS_ENABLED=true
if [[ "$VAD_PROVIDER" == "silero" || "$VAD_PROVIDER" == "webrtc" ]]; then
  PULSE_ENDPOINT_EVENTS_ENABLED=false
fi

print_configuration() {
  cat <<EOF
ROS_DOMAIN_ID=$ROS_DOMAIN_ID，连续语音控制模式=$MODE
FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-<unset>}（默认 UDPv4，用于规避 WSL FastDDS SHM 锁报错）
EMBODIED_ALLOW_FASTDDS_SHM=${EMBODIED_ALLOW_FASTDDS_SHM:-false}

建议演示话术：
  小智
  向前走一秒
  左转九十度
  后退一秒
  绕圈
  走正方形
  停下
  退出控制

说明：一次“小智”唤醒后，${SESSION_TIMEOUT}s 内可连续说多条命令；等待超过 ${COMMAND_MAX_AGE}s 的普通命令会过期跳过；Ctrl-C 退出脚本。
终端会持续打印 [session] / [asr] / [queue] / [action] / [feedback] / [result] 链路事件。
通过标准：至少识别 6 条 ASR final、产生 4 个以上动作、看到 [session] awake 与 sleeping，最后 /cmd_vel 归零。
如需量化验收，请在第二终端运行：CONTINUOUS_LIVE_CHECK_DURATION=180 bash scripts/acceptance_test.sh continuous-live-check $MODE
VOICE_CONTROL_PROFILE=$VOICE_CONTROL_PROFILE（normal/quiet/low_gain/noisy_room；显式环境变量会覆盖 profile 默认值）
APPLY_VOICE_CALIBRATION=$APPLY_VOICE_CALIBRATION（auto/true/false；auto 会在校准 env 存在时加载，且不覆盖显式环境变量）
VOICE_CALIBRATION_ENV=$VOICE_CALIBRATION_ENV（applied=$VOICE_CALIBRATION_ENV_APPLIED）
VOICE_SESSION_TIMEOUT=$SESSION_TIMEOUT
CONTINUOUS_COMMAND_QUEUE_SIZE=$COMMAND_QUEUE_SIZE
COMMAND_NORMALIZATION_ENABLED=$COMMAND_NORMALIZATION_ENABLED
COMMAND_NORMALIZATION_FEEDBACK_ENABLED=$COMMAND_NORMALIZATION_FEEDBACK_ENABLED
COMMAND_NORMALIZATION_FUZZY_THRESHOLD=$COMMAND_NORMALIZATION_FUZZY_THRESHOLD
COMMAND_NORMALIZATION_PATH=$COMMAND_NORMALIZATION_PATH
COMMAND_COMPLETION_ENABLED=$COMMAND_COMPLETION_ENABLED
ASR_COMMIT_DELAY_MS=$ASR_COMMIT_DELAY_MS
WAKE_WORD_ENABLED=$WAKE_WORD_ENABLED
SPEAKER_ENABLED=$SPEAKER_ENABLED
PULSE_CAPTURE_BRIDGE=$PULSE_CAPTURE_BRIDGE（active=$PULSE_CAPTURE_BRIDGE_ACTIVE；WSLg 下用于绕过 PortAudio/ALSA 默认输入近静音问题）
PULSE_CAPTURE_SOURCE=$PULSE_CAPTURE_SOURCE
PULSE_ENDPOINT_EVENTS_ENABLED=$PULSE_ENDPOINT_EVENTS_ENABLED（Silero/WebRTC VAD 接管 endpoint 时自动为 false）
VAD_PROVIDER=$VAD_PROVIDER（requested=$VAD_PROVIDER_REQUESTED；auto 会优先 Silero，其次 WebRTC，最后降级 energy）
SPEECH_START_THRESHOLD=$SPEECH_START_THRESHOLD（energy VAD RMS 起始阈值）
VAD_SPEECH_START_MS=$VAD_SPEECH_START_MS（Silero/WebRTC 连续人声确认时间；抑制单帧噪声误触发）
SPEECH_END_SILENCE_S=$SPEECH_END_SILENCE_S
MIN_UTTERANCE_MS=$MIN_UTTERANCE_MS
MAX_UTTERANCE_S=$MAX_UTTERANCE_S
SILERO_VAD_MODEL_PATH=$SILERO_VAD_MODEL_PATH
SILERO_VAD_USE_ONNX=$SILERO_VAD_USE_ONNX
SILERO_VAD_THRESHOLD=$SILERO_VAD_THRESHOLD
SILERO_VAD_END_THRESHOLD=$SILERO_VAD_END_THRESHOLD（低于起始阈值形成滞回，避免临界概率反复断句）
KWS_PROVIDER=$KWS_PROVIDER（默认 none；mock_text 用于 sidecar 验收，sherpa/openwakeword/livekit 用于真实 KWS）
SHERPA_KWS_TOKENS=$SHERPA_KWS_TOKENS
SHERPA_KWS_ENCODER=$SHERPA_KWS_ENCODER
SHERPA_KWS_DECODER=$SHERPA_KWS_DECODER
SHERPA_KWS_JOINER=$SHERPA_KWS_JOINER
SHERPA_KWS_KEYWORDS_FILE=$SHERPA_KWS_KEYWORDS_FILE
OPENWAKEWORD_MODELS=$OPENWAKEWORD_MODELS（逗号分隔多个模型）
OPENWAKEWORD_THRESHOLD=$OPENWAKEWORD_THRESHOLD
LIVEKIT_WAKEWORD_MODELS=$LIVEKIT_WAKEWORD_MODELS（逗号分隔多个模型）
LIVEKIT_WAKEWORD_THRESHOLD=$LIVEKIT_WAKEWORD_THRESHOLD
AUDIO_ENHANCER=$AUDIO_ENHANCER（当前可用 nlms；webrtc 为后续增强预留，会 fallback 并在 metrics 中显示）
AEC_ENABLED=$AEC_ENABLED
NOISE_SUPPRESSION_ENABLED=$NOISE_SUPPRESSION_ENABLED
AUTO_GAIN_ENABLED=$AUTO_GAIN_ENABLED
CONTINUOUS_MONITOR_ENABLED=$MONITOR_ENABLED
CONTINUOUS_MONITOR_AUDIO_SAMPLE_LIMIT=$MONITOR_AUDIO_SAMPLE_LIMIT
CONTINUOUS_SAMPLE_LOG=$MONITOR_SAMPLE_LOG（可选 JSONL；保存真实 ASR/NLU/action 样本用于回归）
CONTINUOUS_PREFLIGHT_ENABLED=$PREFLIGHT_ENABLED
CONTINUOUS_READINESS_ENABLED=$READINESS_ENABLED
CONTINUOUS_READINESS_DURATION=$READINESS_DURATION
SIMULATION_READINESS_ENABLED=$SIMULATION_READINESS_ENABLED
SIMULATION_READINESS_REQUIRED=$SIMULATION_READINESS_REQUIRED
SIMULATION_READINESS_TIMEOUT=$SIMULATION_READINESS_TIMEOUT
SIMULATION_CLEANUP_STALE=$SIMULATION_CLEANUP_STALE（true 时启动前清理残留 Gazebo/ROS 仿真进程）
CONTINUOUS_COMMAND_MAX_AGE=$COMMAND_MAX_AGE
CONTINUOUS_DUPLICATE_WINDOW_S=$COMMAND_DUPLICATE_WINDOW
GUI_ENABLED=$GUI_ENABLED

EOF
  print_launch_command
}

add_launch_arg() {
  LAUNCH_ARGS+=("$1:=$2")
}

add_optional_launch_arg() {
  # ROS 2 launch 会把空值参数 `name:=` 判定为 malformed argument。
  # 因此可选模型路径/配置路径为空时直接省略，交给 launch 文件里的默认值。
  [[ -z "$2" ]] || add_launch_arg "$1" "$2"
}

build_launch_args() {
  LAUNCH_ARGS=(embodied_simulation voice_turtlebot3.launch.py)
  local capture_enabled=true
  if [[ "$PULSE_CAPTURE_BRIDGE_ACTIVE" == "true" ]]; then
    capture_enabled=false
  fi
  add_launch_arg gui "$GUI_ENABLED"
  add_launch_arg rviz false
  add_launch_arg launch_agent true
  add_launch_arg agent_type "$MODE"
  add_launch_arg provider_mode "$MODE"
  add_launch_arg microphone_enabled true
  add_launch_arg capture_enabled "$capture_enabled"
  add_launch_arg speaker_enabled "$SPEAKER_ENABLED"
  add_launch_arg vad_provider "$VAD_PROVIDER"
  add_launch_arg kws_provider "$KWS_PROVIDER"
  add_launch_arg speech_start_threshold "$SPEECH_START_THRESHOLD"
  add_launch_arg vad_speech_start_ms "$VAD_SPEECH_START_MS"
  add_launch_arg speech_end_silence_s "$SPEECH_END_SILENCE_S"
  add_launch_arg min_utterance_ms "$MIN_UTTERANCE_MS"
  add_launch_arg max_utterance_s "$MAX_UTTERANCE_S"
  add_optional_launch_arg silero_model_path "$SILERO_VAD_MODEL_PATH"
  add_launch_arg silero_use_onnx "$SILERO_VAD_USE_ONNX"
  add_launch_arg silero_threshold "$SILERO_VAD_THRESHOLD"
  add_launch_arg silero_end_threshold "$SILERO_VAD_END_THRESHOLD"
  add_optional_launch_arg sherpa_tokens "$SHERPA_KWS_TOKENS"
  add_optional_launch_arg sherpa_encoder "$SHERPA_KWS_ENCODER"
  add_optional_launch_arg sherpa_decoder "$SHERPA_KWS_DECODER"
  add_optional_launch_arg sherpa_joiner "$SHERPA_KWS_JOINER"
  add_optional_launch_arg sherpa_keywords_file "$SHERPA_KWS_KEYWORDS_FILE"
  add_optional_launch_arg openwakeword_models "$OPENWAKEWORD_MODELS"
  add_launch_arg openwakeword_threshold "$OPENWAKEWORD_THRESHOLD"
  add_optional_launch_arg livekit_wakeword_models "$LIVEKIT_WAKEWORD_MODELS"
  add_launch_arg livekit_wakeword_threshold "$LIVEKIT_WAKEWORD_THRESHOLD"
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
  add_launch_arg audio_enhancer "$AUDIO_ENHANCER"
  add_launch_arg aec_enabled "$AEC_ENABLED"
  add_launch_arg noise_suppression_enabled "$NOISE_SUPPRESSION_ENABLED"
  add_launch_arg auto_gain_enabled "$AUTO_GAIN_ENABLED"
}

print_launch_command() {
  build_launch_args
  echo "ros2 launch \\"
  local arg
  for arg in "${LAUNCH_ARGS[@]}"; do
    echo "  $arg \\"
  done
  echo "  # 注：空的可选模型/配置路径参数会省略，避免 ROS 2 launch 收到非法的 name:=。"
}

if [[ "$PRINT_CONFIG" == "true" ]]; then
  print_configuration
  exit 0
fi

if [[ "$PREFLIGHT_ENABLED" == "true" ]]; then
  python3 "$WORKSPACE/scripts/voice_provider_preflight.py" \
    --mode "$MODE" \
    --vad-provider "$VAD_PROVIDER" \
    --kws-provider "$KWS_PROVIDER" \
    --silero-model-path "$SILERO_VAD_MODEL_PATH" \
    --silero-use-onnx "$SILERO_VAD_USE_ONNX" \
    --sherpa-tokens "$SHERPA_KWS_TOKENS" \
    --sherpa-encoder "$SHERPA_KWS_ENCODER" \
    --sherpa-decoder "$SHERPA_KWS_DECODER" \
    --sherpa-joiner "$SHERPA_KWS_JOINER" \
    --sherpa-keywords-file "$SHERPA_KWS_KEYWORDS_FILE" \
    --openwakeword-models "$OPENWAKEWORD_MODELS" \
    --livekit-wakeword-models "$LIVEKIT_WAKEWORD_MODELS"
fi

if ! pactl list short sources 2>/dev/null | grep -q .; then
  echo "FAIL: WSL 中没有可用麦克风 source；请先检查 WSLg 音频权限。" >&2
  exit 1
fi

if [[ "$SIMULATION_CLEANUP_STALE" == "true" ]]; then
  CLEANUP_CONFIRM=true bash "$WORKSPACE/scripts/cleanup_simulation_processes.sh" || true
else
  if bash "$WORKSPACE/scripts/cleanup_simulation_processes.sh" >/tmp/embodied_agent_stale_simulation_check.log 2>&1; then
    :
  else
    echo "WARN: 检测到可能残留的 Gazebo/ROS 仿真进程，可能导致 Gazebo GUI 空世界或小车模型不出现。" >&2
    echo "      建议先运行：CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh" >&2
    echo "      或本次直接运行：SIMULATION_CLEANUP_STALE=true bash scripts/acceptance_test.sh continuous-$MODE" >&2
    sed 's/^/      /' /tmp/embodied_agent_stale_simulation_check.log >&2 || true
  fi
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
if [[ "$PULSE_CAPTURE_BRIDGE_ACTIVE" == "true" ]]; then
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
fi
if [[ "$MONITOR_ENABLED" == "true" ]]; then
  MONITOR_ARGS=(--audio-sample-limit "$MONITOR_AUDIO_SAMPLE_LIMIT")
  if [[ -n "$MONITOR_SAMPLE_LOG" ]]; then
    MONITOR_ARGS+=(--sample-output "$MONITOR_SAMPLE_LOG")
  fi
  python3 "$WORKSPACE/scripts/continuous_voice_monitor.py" "${MONITOR_ARGS[@]}" &
  MONITOR_PID=$!
fi

if [[ "$SIMULATION_READINESS_ENABLED" == "true" ]]; then
  echo
  echo "正在检查 Gazebo/TurtleBot3 小车模型 readiness（timeout=${SIMULATION_READINESS_TIMEOUT}s）..."
  if python3 "$WORKSPACE/scripts/simulation_readiness_check.py" \
    --timeout "$SIMULATION_READINESS_TIMEOUT"; then
    echo "仿真小车已就绪：已收到 /odom 与 /scan，/cmd_vel 和 ROS 2 Action 链路在线。"
  else
    echo "FAIL: Gazebo/TurtleBot3 小车模型未就绪；为避免演示时只跑语音不动小车，当前停止 continuous-$MODE。" >&2
    echo "排查建议：先运行 bash scripts/acceptance_test.sh gazebo；若需要无 GUI 演示可设置 GUI_ENABLED=false。" >&2
    if [[ "$SIMULATION_READINESS_REQUIRED" == "true" ]]; then
      exit 1
    fi
    echo "WARN: SIMULATION_READINESS_REQUIRED=false，继续运行但小车可能不可见或不可控。" >&2
  fi
fi

if [[ "$READINESS_ENABLED" == "true" ]]; then
  readiness_args=(--duration "$READINESS_DURATION")
  if [[ "$KWS_PROVIDER" == "sherpa" || "$KWS_PROVIDER" == "openwakeword" || "$KWS_PROVIDER" == "livekit" ]]; then
    readiness_args+=(--require-kws)
  fi
  echo
  echo "正在进行连续语音 readiness check（${READINESS_DURATION}s），请保持麦克风环境接近演示现场..."
  if python3 "$WORKSPACE/scripts/voice_control_readiness_check.py" "${readiness_args[@]}"; then
    echo "系统已就绪，可以开始说：小智"
  else
    echo "WARN: readiness check 未完全通过；仍继续运行。建议按顺序检查：麦克风 source、SPEECH_START_THRESHOLD、VOICE_CONTROL_PROFILE=low_gain/quiet/noisy_room，以及可选 KWS 模型路径。" >&2
  fi
fi

wait "$LAUNCH_PID"
