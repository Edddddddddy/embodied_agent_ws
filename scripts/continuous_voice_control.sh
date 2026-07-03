#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODE="${1:-offline}"
SESSION_TIMEOUT="${VOICE_SESSION_TIMEOUT:-60}"
SPEAKER_ENABLED="${SPEAKER_ENABLED:-false}"
VAD_PROVIDER="${VAD_PROVIDER:-energy}"
KWS_PROVIDER="${KWS_PROVIDER:-none}"
GUI_ENABLED="${GUI_ENABLED:-true}"
MONITOR_ENABLED="${CONTINUOUS_MONITOR_ENABLED:-true}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((140 + $$ % 80))}"

if [[ "$MODE" != "offline" && "$MODE" != "online" ]]; then
  echo "Usage: $0 {offline|online}" >&2
  exit 2
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
  [[ -z "$MONITOR_PID" ]] || kill "$MONITOR_PID" 2>/dev/null || true
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

说明：一次“小智”唤醒后，${SESSION_TIMEOUT}s 内可连续说多条命令；Ctrl-C 退出脚本。
终端会持续打印 [session] / [asr] / [queue] / [action] / [result] 链路事件。
VAD_PROVIDER=$VAD_PROVIDER（默认 energy；安装 silero-vad 后可设为 silero）
KWS_PROVIDER=$KWS_PROVIDER（默认 none；mock_text 用于 sidecar 验收，sherpa 用于真实 KWS）
EOF

setsid ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  gui:="$GUI_ENABLED" rviz:=false launch_agent:=true agent_type:="$MODE" \
  provider_mode:="$MODE" microphone_enabled:=true capture_enabled:=true \
  speaker_enabled:="$SPEAKER_ENABLED" vad_provider:="$VAD_PROVIDER" kws_provider:="$KWS_PROVIDER" wake_word_enabled:=true \
  continuous_control_enabled:=true voice_session_timeout_s:="$SESSION_TIMEOUT" &
LAUNCH_PID=$!
if [[ "$MONITOR_ENABLED" == "true" ]]; then
  python3 "$WORKSPACE/scripts/continuous_voice_monitor.py" &
  MONITOR_PID=$!
fi
wait "$LAUNCH_PID"
