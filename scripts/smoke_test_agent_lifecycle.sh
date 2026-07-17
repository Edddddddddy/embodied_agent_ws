#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MODE="${1:-online}"
source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

case "$MODE" in
  online)
    PACKAGE="embodied_online_agent"
    EXECUTABLE="online_agent"
    NODE_NAME="online_agent"
    ;;
  offline)
    PACKAGE="embodied_offline_agent"
    EXECUTABLE="offline_agent"
    NODE_NAME="offline_agent"
    ;;
  *)
    echo "usage: $0 {online|offline}" >&2
    exit 2
    ;;
esac

TMP_DIR="$(mktemp -d)"
PID=""
cleanup() {
  if [[ -n "$PID" ]]; then
    kill -- "-$PID" 2>/dev/null || true
    wait "$PID" 2>/dev/null || true
  fi
  rm -rf "$TMP_DIR"
}
trap cleanup EXIT

setsid ros2 run "$PACKAGE" "$EXECUTABLE" --ros-args \
  -p mode:=mock \
  -p agent_lifecycle_autostart:=false \
  -p microphone_enabled:=false \
  -p wake_word_enabled:=false \
  -p mock_token_delay_s:=0.01 \
  -p memory_path:="$TMP_DIR/memory.json" \
  -p user_memory_dir:="$TMP_DIR/users" \
  >"$TMP_DIR/agent.log" 2>&1 &
PID=$!

if ! bash "$WORKSPACE/tools/acceptance/run_probe.sh" \
  "$WORKSPACE/tests/integration/control/test_agent_lifecycle.py" --agent-name "$NODE_NAME"; then
  cat "$TMP_DIR/agent.log" >&2
  exit 1
fi
