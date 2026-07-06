#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
MODE="${1:-offline}"
REPORT_PATH="${CONTINUOUS_LIVE_CHECK_REPORT:-logs/nav2-live-check-$(date +%Y%m%d-%H%M%S).json}"
CHECK_DURATION="${CONTINUOUS_LIVE_CHECK_DURATION:-240}"
STARTUP_WAIT="${CONTINUOUS_NAV2_EVIDENCE_STARTUP_WAIT:-35}"
DRY_RUN="${CONTINUOUS_NAV2_EVIDENCE_DRY_RUN:-false}"

if [[ "$MODE" != "offline" && "$MODE" != "online" ]]; then
  echo "Usage: $0 {offline|online}" >&2
  exit 2
fi

source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((240 + $$ % 40))}"
mkdir -p "$(dirname "$REPORT_PATH")"

LIVE_CHECK_ARGS=(
  --scenario nav2
  --duration "$CHECK_DURATION"
  --output "$REPORT_PATH"
  --min-asr "${CONTINUOUS_NAV2_LIVE_MIN_ASR:-4}"
  --min-candidates "${CONTINUOUS_NAV2_LIVE_MIN_CANDIDATES:-2}"
  --min-success "${CONTINUOUS_NAV2_LIVE_MIN_SUCCESS:-2}"
  --require-candidate navigate_to
  --require-candidate follow_waypoints
)

CONTROL_PID=""
cleanup() {
  if [[ -n "$CONTROL_PID" ]]; then
    # control 脚本内部还会启动 ros2 launch / monitor；按进程组清理，避免遗留 Gazebo/Nav2。
    kill -TERM -- "-$CONTROL_PID" 2>/dev/null || kill -TERM "$CONTROL_PID" 2>/dev/null || true
    wait "$CONTROL_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

cat <<EOF
ROS_DOMAIN_ID=$ROS_DOMAIN_ID，Nav2 真实麦克风一键留证模式=$MODE

即将后台启动连续语音 Nav2 控制，并在前台统计现场证据。
报告文件：$REPORT_PATH
统计窗口：${CHECK_DURATION}s

推荐话术：
  小智
  去门口
  前往书桌
  依次去门口、书桌、起点
  停止巡航
  退出控制

EOF

if [[ "$DRY_RUN" == "true" ]]; then
  cat <<EOF
DRY RUN: 不启动 Gazebo/Nav2/Agent，也不占用麦克风。
将执行的控制命令：
  ROS_DOMAIN_ID=$ROS_DOMAIN_ID bash scripts/continuous_nav2_voice_control.sh $MODE
将执行的计分命令：
  python3 scripts/continuous_live_check.py ${LIVE_CHECK_ARGS[*]}
EOF
  exit 0
fi

setsid bash "$WORKSPACE/scripts/continuous_nav2_voice_control.sh" "$MODE" &
CONTROL_PID=$!

echo "等待控制链路启动 ${STARTUP_WAIT}s..."
sleep "$STARTUP_WAIT"

if ! kill -0 "$CONTROL_PID" 2>/dev/null; then
  echo "FAIL: continuous_nav2_voice_control.sh 已提前退出，请查看上方日志。" >&2
  exit 1
fi

echo
echo "开始现场计分。现在请按推荐话术说话。"
set +e
python3 scripts/continuous_live_check.py "${LIVE_CHECK_ARGS[@]}"
CHECK_STATUS=$?
set -e

echo
echo "现场报告已保存：$REPORT_PATH"
if [[ "$CHECK_STATUS" -eq 0 ]]; then
  echo "PASS: Nav2 真实麦克风连续导航/巡航现场证据充分"
else
  echo "FAIL: Nav2 真实麦克风连续导航/巡航现场证据不足；可用下面命令复核报告："
  echo "  bash scripts/acceptance_test.sh continuous-nav2-live-report $REPORT_PATH"
fi

exit "$CHECK_STATUS"
