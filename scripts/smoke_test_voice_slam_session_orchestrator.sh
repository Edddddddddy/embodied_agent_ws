#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/ros_dds_env.sh"
cd "$WORKSPACE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 18))}"

REPORT="${SHOWCASE_SESSION_REPORT:-$WORKSPACE/logs/showcase/session_orchestrator_report.json}"
NODE_LOG="$(mktemp)"

setsid ros2 run embodied_slam_tools voice_slam_session_orchestrator --ros-args \
  -p "workspace:=$WORKSPACE" \
  -p mode:=offline \
  -p "map_prefix:=$WORKSPACE/logs/showcase/dry_run_map" \
  -p dry_run:=true >"$NODE_LOG" 2>&1 &
NODE_PID=$!

cleanup() {
  if [[ -n "${NODE_PID:-}" ]]; then
    kill -TERM -- "-$NODE_PID" 2>/dev/null || true
    wait "$NODE_PID" 2>/dev/null || true
  fi
  rm -f "$NODE_LOG"
}
trap cleanup EXIT INT TERM

if ! timeout 30 bash tools/acceptance/run_probe.sh tools/acceptance/probes/slam_nav/session_orchestrator.py \
  --output "$REPORT"; then
  echo "---- orchestrator log ----" >&2
  cat "$NODE_LOG" >&2
  exit 1
fi

# Ctrl+C 是人工主演示的正常退出路径。显式验证节点只能关闭 rclpy context 一次，
# 防止终端最后出现 RCLError，让成功演示看起来像进程崩溃。
kill -INT -- "-$NODE_PID" 2>/dev/null || true
wait "$NODE_PID" 2>/dev/null || true
NODE_PID=""
if grep -q "rcl_shutdown already called" "$NODE_LOG"; then
  echo "FAIL: orchestrator performs duplicate rclpy shutdown on SIGINT" >&2
  cat "$NODE_LOG" >&2
  exit 1
fi

echo "PASS: ASR session intent -> typed ManageSlamSession Action -> ordered stage FSM"
echo "Evidence: $REPORT"
