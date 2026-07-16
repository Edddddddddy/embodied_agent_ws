#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/ros_dds_env.sh"
cd "$WORKSPACE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 18))}"

# 自动门禁用 mock Agent 的确定性 NLU，探索、SLAM、Gazebo、map_saver、AMCL 和
# Nav2 均为真实运行时。这样可以把语音云服务波动与机器人自治能力分开验收。
export NAV2_PROVIDER_MODE=mock
export NAV2_MICROPHONE_ENABLED=false
export NAV2_CAPTURE_ENABLED=false
export WAKE_WORD_ENABLED=false
export CONTINUOUS_PREFLIGHT_ENABLED=false
export CONTINUOUS_MONITOR_ENABLED=false
export CONTINUOUS_READINESS_ENABLED=false
export VAD_PROVIDER=energy
export HEADLESS="${HEADLESS:-true}"
export USE_RVIZ="${USE_RVIZ:-false}"
export SYSTEM_READINESS_TIMEOUT=120

SESSION_DIR="${SHOWCASE_SESSION_DIR:-$WORKSPACE/logs/showcase/autonomous_runtime}"
MAP_PREFIX="$SESSION_DIR/voice_built_map"
REPORT="$SESSION_DIR/automatic_mission_report.json"
MISSION_PLAN="$WORKSPACE/src/embodied_simulation/config/showcase_workplace_mission.yaml"
NODE_LOG="$(mktemp)"
mkdir -p "$SESSION_DIR"
rm -f "$MAP_PREFIX.yaml" "$MAP_PREFIX.pgm" "$REPORT"

if ! ros2 pkg prefix explore_lite >/dev/null 2>&1; then
  echo "MISSING: Explore Lite. Run: bash scripts/setup_frontier_exploration.sh" >&2
  exit 2
fi

setsid ros2 run embodied_slam_tools voice_slam_session_orchestrator --ros-args \
  -p "workspace:=$WORKSPACE" \
  -p mode:=offline \
  -p "map_prefix:=$MAP_PREFIX" \
  -p "mission_plan:=$MISSION_PLAN" \
  -p startup_timeout_s:=150.0 >"$NODE_LOG" 2>&1 &
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

if ! timeout 780 python3 tests/integration/test_voice_slam_session_orchestrator.py \
  --output "$REPORT" \
  --transition-timeout 750 \
  --evidence-kind gazebo_frontier_slam_map_saver_amcl_nav2 \
  --automatic-mission; then
  echo "---- automatic orchestrator log (last 400 lines) ----" >&2
  tail -n 400 "$NODE_LOG" >&2
  exit 1
fi

test -s "$MAP_PREFIX.yaml"
test -s "$MAP_PREFIX.pgm"
python3 - "$REPORT" <<'PY'
import json
from pathlib import Path
import sys

report = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
assert report["passed"] is True
assert report["automatic_mission"] is True
assert report["map_saved"] is True
assert report["final_phase"] == 12
assert report["map"] and report["map"]["known_cells"] > 0
assert abs(report["final_cmd_vel"]["linear_x"]) < 1e-6
assert abs(report["final_cmd_vel"]["angular_z"]) < 1e-6
print(json.dumps(report, ensure_ascii=False, indent=2))
PY

echo "PASS: one intent -> autonomous frontier SLAM -> saved map -> AMCL/Nav2 patrol"
echo "Evidence: $REPORT"
