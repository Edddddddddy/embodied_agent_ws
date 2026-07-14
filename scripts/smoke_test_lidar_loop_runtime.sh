#!/usr/bin/env bash
set -eo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
# shellcheck disable=SC1091
source "${LIDAR_LOOP_SETUP_FILE:-$WORKSPACE/install/setup.bash}"
if [[ -n "${LIDAR_LOOP_INTERFACE_SETUP_FILE:-}" ]]; then
  # 仅供隔离 worktree 构建使用；正常验收的统一 install/setup.bash 已包含该接口。
  # shellcheck disable=SC1090
  source "$LIDAR_LOOP_INTERFACE_SETUP_FILE"
fi
set -u

NODE_LOG="${LIDAR_LOOP_NODE_LOG:-/tmp/lidar_loop_runtime_node.log}"
ros2 run embodied_slam lidar_loop_candidate_node --ros-args \
  -p minimum_points:=20 \
  -p minimum_temporal_separation_s:=1.0 \
  -p sample_interval_s:=0.5 \
  -p minimum_similarity:=0.5 >"$NODE_LOG" 2>&1 &
NODE_PID=$!

cleanup() {
  kill "$NODE_PID" 2>/dev/null || true
  wait "$NODE_PID" 2>/dev/null || true
}
trap cleanup EXIT

if ! python3 "$WORKSPACE/tests/integration/test_lidar_loop_runtime.py"; then
  echo "---- lidar_loop_candidate_node log ----" >&2
  tail -80 "$NODE_LOG" >&2 || true
  exit 1
fi
