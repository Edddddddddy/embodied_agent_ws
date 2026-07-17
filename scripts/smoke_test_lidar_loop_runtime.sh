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
VERIFIER_LOG="${LIDAR_LOOP_VERIFIER_LOG:-/tmp/lidar_loop_verifier_node.log}"
GATE_LOG="${LIDAR_LOOP_GATE_LOG:-/tmp/lidar_loop_constraint_gate_node.log}"
ADAPTER_LOG="${LIDAR_LOOP_ADAPTER_LOG:-/tmp/lidar_loop_constraint_adapter.log}"
ros2 run embodied_slam lidar_loop_candidate_node --ros-args \
  -p minimum_points:=20 \
  -p minimum_temporal_separation_s:=1.0 \
  -p sample_interval_s:=0.5 \
  -p minimum_similarity:=0.5 >"$NODE_LOG" 2>&1 &
NODE_PID=$!
ros2 run embodied_slam lidar_loop_verifier_node --ros-args \
  -p point_stride:=1 \
  -p minimum_points:=20 \
  -p maximum_cached_scans:=32 >"$VERIFIER_LOG" 2>&1 &
VERIFIER_PID=$!
ros2 run embodied_slam lidar_loop_constraint_gate_node --ros-args \
  -p commit_enabled:=false >"$GATE_LOG" 2>&1 &
GATE_PID=$!
ros2 launch embodied_slam instrumented_online_async.launch.py \
  autostart:=true \
  use_sim_time:=false \
  external_loop_constraint_enabled:=true >"$ADAPTER_LOG" 2>&1 &
ADAPTER_PID=$!

cleanup() {
  kill "$NODE_PID" 2>/dev/null || true
  kill "$VERIFIER_PID" 2>/dev/null || true
  kill "$GATE_PID" 2>/dev/null || true
  kill "$ADAPTER_PID" 2>/dev/null || true
  wait "$NODE_PID" 2>/dev/null || true
  wait "$VERIFIER_PID" 2>/dev/null || true
  wait "$GATE_PID" 2>/dev/null || true
  wait "$ADAPTER_PID" 2>/dev/null || true
}
trap cleanup EXIT

if ! bash "$WORKSPACE/tools/acceptance/run_probe.sh" \
    "$WORKSPACE/tests/integration/slam_nav/test_lidar_loop_runtime.py"; then
  echo "---- lidar_loop_candidate_node log ----" >&2
  tail -80 "$NODE_LOG" >&2 || true
  echo "---- lidar_loop_verifier_node log ----" >&2
  tail -80 "$VERIFIER_LOG" >&2 || true
  echo "---- lidar_loop_constraint_gate_node log ----" >&2
  tail -80 "$GATE_LOG" >&2 || true
  echo "---- instrumented slam_toolbox adapter log ----" >&2
  tail -100 "$ADAPTER_LOG" >&2 || true
  exit 1
fi
