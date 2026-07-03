#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODE="${1:-offline}"
SESSION_TIMEOUT="${VOICE_SESSION_TIMEOUT:-60}"
COMMAND_MAX_AGE="${CONTINUOUS_COMMAND_MAX_AGE:-30}"
WAKE_WORD_ENABLED="${WAKE_WORD_ENABLED:-true}"
SPEAKER_ENABLED="${SPEAKER_ENABLED:-false}"
VAD_PROVIDER="${VAD_PROVIDER:-energy}"
SPEECH_START_THRESHOLD="${SPEECH_START_THRESHOLD:-0.018}"
SPEECH_END_SILENCE_S="${SPEECH_END_SILENCE_S:-0.4}"
MIN_UTTERANCE_MS="${MIN_UTTERANCE_MS:-100}"
MAX_UTTERANCE_S="${MAX_UTTERANCE_S:-12.0}"
SILERO_VAD_MODEL_PATH="${SILERO_VAD_MODEL_PATH:-}"
SILERO_VAD_USE_ONNX="${SILERO_VAD_USE_ONNX:-true}"
SILERO_VAD_THRESHOLD="${SILERO_VAD_THRESHOLD:-0.5}"
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
AEC_ENABLED="${AEC_ENABLED:-true}"
NOISE_SUPPRESSION_ENABLED="${NOISE_SUPPRESSION_ENABLED:-false}"
AUTO_GAIN_ENABLED="${AUTO_GAIN_ENABLED:-false}"
GUI_ENABLED="${GUI_ENABLED:-true}"
MONITOR_ENABLED="${CONTINUOUS_MONITOR_ENABLED:-true}"
PRINT_CONFIG="${CONTINUOUS_PRINT_CONFIG:-false}"
PREFLIGHT_ENABLED="${CONTINUOUS_PREFLIGHT_ENABLED:-true}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((140 + $$ % 80))}"

if [[ "$MODE" != "offline" && "$MODE" != "online" ]]; then
  echo "Usage: $0 {offline|online}" >&2
  exit 2
fi

print_configuration() {
  cat <<EOF
ROS_DOMAIN_ID=$ROS_DOMAIN_ID，连续语音控制模式=$MODE

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
WAKE_WORD_ENABLED=$WAKE_WORD_ENABLED
SPEAKER_ENABLED=$SPEAKER_ENABLED
VAD_PROVIDER=$VAD_PROVIDER（默认 energy；安装 silero-vad 后可设为 silero）
SPEECH_START_THRESHOLD=$SPEECH_START_THRESHOLD（energy VAD RMS 起始阈值）
SPEECH_END_SILENCE_S=$SPEECH_END_SILENCE_S
MIN_UTTERANCE_MS=$MIN_UTTERANCE_MS
MAX_UTTERANCE_S=$MAX_UTTERANCE_S
SILERO_VAD_MODEL_PATH=$SILERO_VAD_MODEL_PATH
SILERO_VAD_USE_ONNX=$SILERO_VAD_USE_ONNX
SILERO_VAD_THRESHOLD=$SILERO_VAD_THRESHOLD
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
CONTINUOUS_PREFLIGHT_ENABLED=$PREFLIGHT_ENABLED
GUI_ENABLED=$GUI_ENABLED

ros2 launch embodied_simulation voice_turtlebot3.launch.py \\
  gui:=$GUI_ENABLED rviz:=false launch_agent:=true agent_type:=$MODE \\
  provider_mode:=$MODE microphone_enabled:=true capture_enabled:=true \\
  speaker_enabled:=$SPEAKER_ENABLED vad_provider:=$VAD_PROVIDER kws_provider:=$KWS_PROVIDER \\
  speech_start_threshold:=$SPEECH_START_THRESHOLD speech_end_silence_s:=$SPEECH_END_SILENCE_S \\
  min_utterance_ms:=$MIN_UTTERANCE_MS max_utterance_s:=$MAX_UTTERANCE_S \\
  silero_model_path:="$SILERO_VAD_MODEL_PATH" silero_use_onnx:=$SILERO_VAD_USE_ONNX \\
  silero_threshold:=$SILERO_VAD_THRESHOLD \\
  sherpa_tokens:="$SHERPA_KWS_TOKENS" sherpa_encoder:="$SHERPA_KWS_ENCODER" \\
  sherpa_decoder:="$SHERPA_KWS_DECODER" sherpa_joiner:="$SHERPA_KWS_JOINER" \\
  sherpa_keywords_file:="$SHERPA_KWS_KEYWORDS_FILE" \\
  openwakeword_models:="$OPENWAKEWORD_MODELS" openwakeword_threshold:=$OPENWAKEWORD_THRESHOLD \\
  livekit_wakeword_models:="$LIVEKIT_WAKEWORD_MODELS" livekit_wakeword_threshold:=$LIVEKIT_WAKEWORD_THRESHOLD \\
  wake_word_enabled:=$WAKE_WORD_ENABLED continuous_control_enabled:=true \\
  voice_session_timeout_s:=$SESSION_TIMEOUT continuous_command_max_age_s:=$COMMAND_MAX_AGE \\
  audio_enhancer:=$AUDIO_ENHANCER aec_enabled:=$AEC_ENABLED \\
  noise_suppression_enabled:=$NOISE_SUPPRESSION_ENABLED auto_gain_enabled:=$AUTO_GAIN_ENABLED
EOF
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

setsid ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  gui:="$GUI_ENABLED" rviz:=false launch_agent:=true agent_type:="$MODE" \
  provider_mode:="$MODE" microphone_enabled:=true capture_enabled:=true \
  speaker_enabled:="$SPEAKER_ENABLED" vad_provider:="$VAD_PROVIDER" kws_provider:="$KWS_PROVIDER" \
  speech_start_threshold:="$SPEECH_START_THRESHOLD" speech_end_silence_s:="$SPEECH_END_SILENCE_S" \
  min_utterance_ms:="$MIN_UTTERANCE_MS" max_utterance_s:="$MAX_UTTERANCE_S" \
  silero_model_path:="$SILERO_VAD_MODEL_PATH" silero_use_onnx:="$SILERO_VAD_USE_ONNX" \
  silero_threshold:="$SILERO_VAD_THRESHOLD" \
  sherpa_tokens:="$SHERPA_KWS_TOKENS" sherpa_encoder:="$SHERPA_KWS_ENCODER" \
  sherpa_decoder:="$SHERPA_KWS_DECODER" sherpa_joiner:="$SHERPA_KWS_JOINER" \
  sherpa_keywords_file:="$SHERPA_KWS_KEYWORDS_FILE" \
  openwakeword_models:="$OPENWAKEWORD_MODELS" openwakeword_threshold:="$OPENWAKEWORD_THRESHOLD" \
  livekit_wakeword_models:="$LIVEKIT_WAKEWORD_MODELS" livekit_wakeword_threshold:="$LIVEKIT_WAKEWORD_THRESHOLD" \
  wake_word_enabled:="$WAKE_WORD_ENABLED" \
  continuous_control_enabled:=true voice_session_timeout_s:="$SESSION_TIMEOUT" \
  continuous_command_max_age_s:="$COMMAND_MAX_AGE" \
  audio_enhancer:="$AUDIO_ENHANCER" aec_enabled:="$AEC_ENABLED" \
  noise_suppression_enabled:="$NOISE_SUPPRESSION_ENABLED" \
  auto_gain_enabled:="$AUTO_GAIN_ENABLED" &
LAUNCH_PID=$!
if [[ "$MONITOR_ENABLED" == "true" ]]; then
  python3 "$WORKSPACE/scripts/continuous_voice_monitor.py" &
  MONITOR_PID=$!
fi
wait "$LAUNCH_PID"
