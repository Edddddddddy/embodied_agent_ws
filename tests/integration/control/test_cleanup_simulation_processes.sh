#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd -P)"
cd "$WORKSPACE"

# 用改写 argv[0] 的无害 sleep 模拟遗留节点，避免测试依赖真实 Gazebo。
bash -c 'exec -a "/tmp/embodied_simulation/simulation_control_node" sleep 30' &
FAKE_PID=$!
NON_TARGET_PID=""
cleanup() {
  kill -KILL "$FAKE_PID" 2>/dev/null || true
  [[ -z "$NON_TARGET_PID" ]] || kill -KILL "$NON_TARGET_PID" 2>/dev/null || true
}
trap cleanup EXIT

sleep 0.1
kill -0 "$FAKE_PID"
OUTPUT="$(CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh)"
printf '%s\n' "$OUTPUT"

if kill -0 "$FAKE_PID" 2>/dev/null; then
  echo "FAIL: cleanup left the simulated stale process alive" >&2
  exit 1
fi
grep -q "PASS: stale Gazebo/ROS simulation processes terminated" <<<"$OUTPUT"

# 编译/测试命令可能包含节点源码文件名，但不是运行中的 ROS 可执行文件。
bash -c 'exec -a "python3 -m py_compile test_voice_slam_session_orchestrator.py" sleep 30' &
NON_TARGET_PID=$!
sleep 0.1
OUTPUT="$(bash scripts/cleanup_simulation_processes.sh)"
grep -q "PASS: no stale Gazebo/ROS simulation processes found" <<<"$OUTPUT"
kill -0 "$NON_TARGET_PID"
echo "PASS: cleanup targets stale processes without killing its caller"
