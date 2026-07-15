#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/ros_dds_env.sh"
cd "$WORKSPACE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 18))}"

# 自动门禁用 mock Agent + 文本注入代替真人发声，但 mapping/map_saver、进程切换、
# AMCL/Nav2、Gazebo 物理和 typed Action 都是真实运行时。
export NAV2_PROVIDER_MODE=mock
export NAV2_MICROPHONE_ENABLED=false
export NAV2_CAPTURE_ENABLED=false
export WAKE_WORD_ENABLED=false
export CONTINUOUS_PREFLIGHT_ENABLED=false
export CONTINUOUS_MONITOR_ENABLED=false
export VAD_PROVIDER=energy
export HEADLESS=true
export USE_RVIZ=false
export SYSTEM_READINESS_TIMEOUT=90

SESSION_DIR="${SHOWCASE_SESSION_DIR:-$WORKSPACE/logs/showcase/orchestrated_runtime}"
MAP_PREFIX="$SESSION_DIR/voice_built_map"
SESSION_REPORT="$SESSION_DIR/session_report.json"
NAV_REPORT="$SESSION_DIR/navigation_report.json"
NODE_LOG="$(mktemp)"
mkdir -p "$SESSION_DIR"
rm -f "$MAP_PREFIX.yaml" "$MAP_PREFIX.pgm" "$SESSION_REPORT" "$NAV_REPORT"

setsid ros2 run embodied_slam_tools voice_slam_session_orchestrator --ros-args \
  -p "workspace:=$WORKSPACE" \
  -p mode:=offline \
  -p "map_prefix:=$MAP_PREFIX" \
  -p startup_timeout_s:=120.0 >"$NODE_LOG" 2>&1 &
NODE_PID=$!

cleanup() {
  kill -TERM "$NODE_PID" 2>/dev/null || true
  for _ in $(seq 1 80); do
    kill -0 "$NODE_PID" 2>/dev/null || break
    sleep 0.1
  done
  kill -TERM -- "-$NODE_PID" 2>/dev/null || true
  wait "$NODE_PID" 2>/dev/null || true
  rm -f "$NODE_LOG"
}
trap cleanup EXIT INT TERM

if ! timeout 210 python3 tests/integration/test_voice_slam_session_orchestrator.py \
  --output "$SESSION_REPORT" \
  --transition-timeout 150 \
  --evidence-kind gazebo_slam_map_saver_amcl_nav2 \
  --explore-before-save; then
  echo "---- orchestrator log (last 300 lines) ----" >&2
  tail -n 300 "$NODE_LOG" >&2
  exit 1
fi

test -s "$MAP_PREFIX.yaml"
test -s "$MAP_PREFIX.pgm"

if ! timeout 180 python3 tests/integration/test_nav2_turtlebot3_voice.py \
  --initial-x 0.0 --initial-y 0.0 --initial-yaw 0.0 \
  --navigate-text "去客厅" \
  --navigate-timeout 120 \
  --skip-patrol \
  --output "$NAV_REPORT"; then
  echo "---- orchestrator log (last 300 lines) ----" >&2
  tail -n 300 "$NODE_LOG" >&2
  exit 1
fi

python3 - "$SESSION_REPORT" "$NAV_REPORT" <<'PY'
import json
from pathlib import Path
import sys

session = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
navigation = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
assert session["passed"]
assert session["exploration_succeeded"] is True
assert session["final_phase"] == 7
assert navigation["status"] == "PASS"
assert navigation["distance_m"] >= 0.05
print(json.dumps({
    "passed": True,
    "session_phase": session["final_phase"],
    "map_saved": session["map_saved"],
    "navigation_target": navigation["navigate_to"],
    "navigation_distance_m": navigation["distance_m"],
    "evidence_kind": session["evidence_kind"],
}, ensure_ascii=False, indent=2))
PY

echo "PASS: one terminal text-ASR -> mapping -> save -> restart -> AMCL/Nav2 -> motion"
echo "Evidence: $SESSION_REPORT $NAV_REPORT"
