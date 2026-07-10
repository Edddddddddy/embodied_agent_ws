#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODE="${1:-offline}"
CHECK_DURATION="${VOICE_BENCHMARK_DURATION:-180}"
STARTUP_WAIT="${CONTINUOUS_VOICE_EVIDENCE_STARTUP_WAIT:-25}"
LIVE_REPORT="${VOICE_BENCHMARK_LIVE_REPORT:-logs/continuous_voice_${MODE}_live_report.json}"
SUMMARY_REPORT="${VOICE_BENCHMARK_REPORT:-logs/voice_benchmark_report.json}"
DRY_RUN="${CONTINUOUS_VOICE_EVIDENCE_DRY_RUN:-false}"

if [[ "$MODE" != "offline" && "$MODE" != "online" ]]; then
  echo "Usage: $0 {offline|online}" >&2
  exit 2
fi

source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((220 + $$ % 20))}"
LIVE_REPORT="$(realpath -m "$LIVE_REPORT")"
SUMMARY_REPORT="$(realpath -m "$SUMMARY_REPORT")"
mkdir -p "$(dirname "$LIVE_REPORT")" "$(dirname "$SUMMARY_REPORT")"

CONTROL_PID=""
cleanup() {
  if [[ -n "$CONTROL_PID" ]]; then
    # 先只通知父脚本，让它自己的 trap 按顺序关闭 launch/monitor/llama-server；
    # 直接给整个进程组同时发信号会让 llama-server 收到双重中断并打印误导性的 backtrace。
    kill -TERM "$CONTROL_PID" 2>/dev/null || true
    for _ in $(seq 1 50); do
      kill -0 "$CONTROL_PID" 2>/dev/null || break
      sleep 0.1
    done
    if kill -0 "$CONTROL_PID" 2>/dev/null; then
      kill -TERM -- "-$CONTROL_PID" 2>/dev/null || true
    fi
    wait "$CONTROL_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

cat <<EOF
ROS_DOMAIN_ID=$ROS_DOMAIN_ID，连续语音一键留证模式=$MODE

这个入口会自动完成：
  1. 后台启动真实麦克风与仿真控制链路；
  2. 等待 ${STARTUP_WAIT}s；
  3. 计分 ${CHECK_DURATION}s，并持续显示倒计时；
  4. 无论 PASS/FAIL 都写出两份报告；
  5. 自动关闭后台 ROS/Gazebo 进程并退出。

固定话术：
  小智
  向前走一秒
  左转九十度
  后退一秒
  右转九十度
  绕圈
  挥手两次
  把灯设成蓝色
  去门口
  取消导航
  停下
  退出控制

现场事件报告：$LIVE_REPORT
量化汇总报告：$SUMMARY_REPORT
EOF

if [[ "$DRY_RUN" == "true" ]]; then
  cat <<EOF
DRY RUN: 不启动麦克风、ROS 或 Gazebo。
控制命令：ROS_DOMAIN_ID=$ROS_DOMAIN_ID bash scripts/continuous_voice_control.sh $MODE
计分命令：ROS_DOMAIN_ID=$ROS_DOMAIN_ID bash scripts/acceptance_test.sh continuous-voice-benchmark $MODE
EOF
  exit 0
fi

setsid bash "$WORKSPACE/scripts/continuous_voice_control.sh" "$MODE" &
CONTROL_PID=$!

echo "等待控制链路启动 ${STARTUP_WAIT}s..."
sleep "$STARTUP_WAIT"
if ! kill -0 "$CONTROL_PID" 2>/dev/null; then
  echo "FAIL: 连续语音控制链路提前退出，请检查上方启动日志。" >&2
  exit 1
fi

echo
echo "开始计分，请按固定话术说话。"
STATUS=0
if VOICE_BENCHMARK_DURATION="$CHECK_DURATION" \
    VOICE_BENCHMARK_CONTROL_MANAGED=true \
    VOICE_BENCHMARK_LIVE_REPORT="$LIVE_REPORT" \
    VOICE_BENCHMARK_REPORT="$SUMMARY_REPORT" \
    bash scripts/acceptance_test.sh continuous-voice-benchmark "$MODE"; then
  STATUS=0
else
  STATUS=$?
fi

echo
echo "计分结束，正在自动关闭后台控制链路。"
echo "现场事件报告：$LIVE_REPORT"
echo "量化汇总报告：$SUMMARY_REPORT"
exit "$STATUS"
