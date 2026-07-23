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
MISSION_REPORT="$SESSION_DIR/workplace_mission_report.json"
MISSION_PLAN="$WORKSPACE/src/embodied_simulation/config/showcase_workplace_mission.yaml"
NODE_LOG="$(mktemp)"
mkdir -p "$SESSION_DIR"
rm -f "$MAP_PREFIX.yaml" "$MAP_PREFIX.pgm" "$SESSION_REPORT" "$NAV_REPORT" "$MISSION_REPORT"

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

if ! timeout 360 bash tools/acceptance/run_probe.sh tools/acceptance/probes/slam_nav/session_orchestrator.py \
  --output "$SESSION_REPORT" \
  --transition-timeout 180 \
  --evidence-kind gazebo_slam_map_saver_amcl_nav2 \
  --survey-plan "$MISSION_PLAN"; then
  echo "---- orchestrator log (last 300 lines) ----" >&2
  tail -n 300 "$NODE_LOG" >&2
  exit 1
fi

test -s "$MAP_PREFIX.yaml"
test -s "$MAP_PREFIX.pgm"

if ! timeout 420 bash tools/acceptance/run_probe.sh tools/acceptance/probes/slam_nav/nav2_turtlebot3_voice.py \
  --initial-x 0.0 --initial-y 0.0 --initial-yaw 0.0 \
  --navigate-text "去入口" \
  --patrol-text "依次去厨房、办公室" \
  --navigate-timeout 150 \
  --patrol-timeout 300 \
  --output "$NAV_REPORT"; then
  echo "---- orchestrator log (last 300 lines) ----" >&2
  tail -n 300 "$NODE_LOG" >&2
  exit 1
fi

python3 - "$SESSION_REPORT" "$NAV_REPORT" "$MISSION_PLAN" "$MISSION_REPORT" <<'PY'
import json
from pathlib import Path
import sys
import yaml

session = json.loads(Path(sys.argv[1]).read_text(encoding="utf-8"))
navigation = json.loads(Path(sys.argv[2]).read_text(encoding="utf-8"))
plan = yaml.safe_load(Path(sys.argv[3]).read_text(encoding="utf-8"))
assert session["passed"]
assert session["exploration_succeeded"] is True
assert session["final_phase"] == 7
assert session["survey_step_count"] == len(plan["mapping_route"])
assert session["survey_distance_m"] >= plan["acceptance"]["min_mapping_path_m"]
assert session["survey_map"]["known_cells"] >= plan["acceptance"]["min_known_map_cells"]
assert session["survey_map"]["occupied_cells"] >= plan["acceptance"]["min_occupied_map_cells"]
assert navigation["status"] == "PASS"
assert navigation["distance_m"] >= 0.05
visited_targets = [navigation["navigate_to"]["target"]]
visited_targets.extend(navigation["follow_waypoints"]["waypoints"])
assert visited_targets == plan["navigation_mission"]["expected_targets"]
assert len(visited_targets) >= plan["acceptance"]["min_navigation_targets"]
if plan["acceptance"]["require_map_to_odom"]:
    assert navigation["localization"]["parent_frame"] == "map"
if plan["acceptance"]["require_final_zero_velocity"]:
    assert abs(navigation["final_cmd_vel"]["linear_x"]) < 1e-6
    assert abs(navigation["final_cmd_vel"]["angular_z"]) < 1e-6
report = {
    "passed": True,
    "scenario": plan["name"],
    "session_phase": session["final_phase"],
    "map_saved": session["map_saved"],
    "mapping_route_steps": session["survey_step_count"],
    "mapping_path_m": session["survey_distance_m"],
    "mapping_map": session["survey_map"],
    "navigation_target": navigation["navigate_to"],
    "navigation_patrol": navigation["follow_waypoints"],
    "navigation_distance_m": navigation["distance_m"],
    "visited_targets": visited_targets,
    "final_cmd_vel": navigation["final_cmd_vel"],
    "localization": navigation["localization"],
    "evidence_kind": session["evidence_kind"],
}
Path(sys.argv[4]).write_text(
    json.dumps(report, ensure_ascii=False, indent=2) + "\n",
    encoding="utf-8",
)
print(json.dumps(report, ensure_ascii=False, indent=2))
PY

echo "PASS: workplace survey -> SLAM map -> AMCL -> entrance -> kitchen/office patrol"
echo "Evidence: $SESSION_REPORT $NAV_REPORT $MISSION_REPORT"
