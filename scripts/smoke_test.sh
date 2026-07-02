#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((20 + $$ % 20))}"
TMP_DIR="$(mktemp -d)"
LAUNCH_PID=""
STARTED_DEMO=false
ACTION_ECHO_PID=""
METRICS_ECHO_PID=""

cleanup() {
  for pid in "$ACTION_ECHO_PID" "$METRICS_ECHO_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  if [[ "$STARTED_DEMO" == true ]] && [[ -n "$LAUNCH_PID" ]]; then
    kill -INT -- "-$LAUNCH_PID" 2>/dev/null || true
    for _ in $(seq 1 30); do
      if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
        break
      fi
      sleep 0.1
    done
    if kill -0 "$LAUNCH_PID" 2>/dev/null; then
      kill -TERM -- "-$LAUNCH_PID" 2>/dev/null || true
    fi
    sleep 0.2
    kill -KILL -- "-$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  rm -f "$TMP_DIR/action.out" "$TMP_DIR/metrics.out" "$TMP_DIR/launch.log"
  rmdir "$TMP_DIR" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

source "$WORKSPACE/scripts/activate.sh"

if ros2 node list 2>/dev/null | grep -qx '/online_agent'; then
  MODE="$(ros2 param get /online_agent mode 2>/dev/null || true)"
  if [[ "$MODE" != *mock* ]]; then
    echo "FAIL: the existing /online_agent is not running in mock mode."
    exit 1
  fi
  echo "INFO: using the existing /online_agent node."
else
  setsid ros2 launch embodied_online_agent demo.launch.py \
    >"$TMP_DIR/launch.log" 2>&1 &
  LAUNCH_PID=$!
  STARTED_DEMO=true

  READY=false
  for _ in $(seq 1 50); do
    if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
      echo "FAIL: demo launch exited unexpectedly."
      cat "$TMP_DIR/launch.log"
      exit 1
    fi
    if ros2 node list 2>/dev/null | grep -qx '/online_agent'; then
      READY=true
      break
    fi
    sleep 0.1
  done

  if [[ "$READY" != true ]]; then
    echo "FAIL: /online_agent did not become ready."
    cat "$TMP_DIR/launch.log"
    exit 1
  fi
fi

if ! python "$WORKSPACE/tests/integration/test_mock_online_pipeline.py"; then
  echo "FAIL: mock online Agent pipeline did not complete."
  cat "$TMP_DIR/launch.log"
  exit 1
fi
echo "PASS: mock Agent published the expected action and latency metrics."
