#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((170 + $$ % 80))}"

LOG_FILE="$(mktemp)"
CLIENT_LOG="$(mktemp)"
REPORT_DIR="$(mktemp -d)"
FEEDBACK_LOG="$REPORT_DIR/action_feedback.log"
EVIDENCE_DIR="$WORKSPACE/logs"
EVIDENCE_FILE="$EVIDENCE_DIR/cpp_action_lifecycle_report.json"
FEEDBACK_PID=""

setsid ros2 launch embodied_simulation simulation_control.launch.py \
  use_typed_actions:=false \
  executor_plugin:=embodied_simulation/MockRobotExecutor \
  action_timeout_s:=3.0 \
  >"$LOG_FILE" 2>&1 &
SERVER_PID=$!

cleanup() {
  if [[ -n "$FEEDBACK_PID" ]]; then
    kill -TERM "$FEEDBACK_PID" 2>/dev/null || true
    wait "$FEEDBACK_PID" 2>/dev/null || true
  fi
  kill -TERM -- "-$SERVER_PID" 2>/dev/null || true
  kill -KILL -- "-$SERVER_PID" 2>/dev/null || true
  wait "$SERVER_PID" 2>/dev/null || true
  rm -f "$LOG_FILE" "$CLIENT_LOG"
  rm -rf "$REPORT_DIR"
}
trap cleanup EXIT

for _ in $(seq 1 80); do
  if ros2 action list 2>/dev/null | grep -qx "/robot/execute_command"; then
    break
  fi
  sleep 0.1
done

if ! ros2 action list 2>/dev/null | grep -qx "/robot/execute_command"; then
  echo "typed action server did not appear" >&2
  cat "$LOG_FILE" >&2
  exit 1
fi

# 直接监听 action feedback wire topic；即使首次 DDS 握手使客户端回调较晚，
# 仍可证明服务端持续发布 PHASE_EXECUTING/progress，而不是只返回最终结果。
timeout 30 ros2 topic echo /robot/execute_command/_action/feedback \
  >"$FEEDBACK_LOG" 2>&1 &
FEEDBACK_PID=$!
sleep 0.5

run_case() {
  local name="$1"
  local expected="$2"
  local cancel_after="$3"
  local duration="$4"
  local report="$REPORT_DIR/$name.json"
  : >"$CLIENT_LOG"
  if ! timeout 20 ros2 run embodied_agent_cpp typed_action_demo_client \
    move 0.10 "$duration" --ros-args \
    -p "expected_outcome:=$expected" \
    -p "cancel_after_s:=$cancel_after" \
    -p "result_timeout_s:=5.0" \
    -p "report_path:=$report" >"$CLIENT_LOG" 2>&1; then
    echo "typed_action_demo_client scenario failed: $name" >&2
    cat "$CLIENT_LOG" >&2
    cat "$LOG_FILE" >&2
    exit 1
  fi
  test -s "$report" || {
    echo "typed_action_demo_client did not create report: $name" >&2
    cat "$CLIENT_LOG" >&2
    exit 1
  }
  grep "CPP_ACTION_REPORT" "$CLIENT_LOG"
}

# 三条路径共用同一 action server，证明终态不是 mock 日志拼出来的：
# 短动作成功；长动作由客户端发起原生 cancel；长动作由服务端硬超时终止。
run_case success succeeded -1.0 1.50
run_case cancel canceled 0.50 2.0
run_case timeout timed_out -1.0 5.0

kill -TERM "$FEEDBACK_PID" 2>/dev/null || true
wait "$FEEDBACK_PID" 2>/dev/null || true
FEEDBACK_PID=""

mkdir -p "$EVIDENCE_DIR"
python3 "$WORKSPACE/scripts/audit_cpp_action_reports.py" \
  "$REPORT_DIR/success.json" \
  "$REPORT_DIR/cancel.json" \
  "$REPORT_DIR/timeout.json" \
  --feedback-log "$FEEDBACK_LOG" \
  --output "$EVIDENCE_FILE"

echo "PASS: C++ typed Action success/cancel/timeout lifecycle verified"
echo "Evidence: $EVIDENCE_FILE"
