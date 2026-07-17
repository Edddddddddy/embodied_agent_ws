#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
source "$WORKSPACE/scripts/ros_dds_env.sh"
cd "$WORKSPACE"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((210 + $$ % 18))}"
export GZ_PARTITION="${GZ_PARTITION:-embodied_agent_$ROS_DOMAIN_ID}"
export IGN_PARTITION="${IGN_PARTITION:-$GZ_PARTITION}"

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
export SHOWCASE_DYNAMIC_OBSTACLE_ENABLED=true

SESSION_ID="${SLAM_NAV_SESSION_ID:-$(date -u +%Y%m%dT%H%M%SZ)-$$}"
SESSION_START_NS="$(date +%s%N)"
SESSION_DIR="${SHOWCASE_SESSION_DIR:-$WORKSPACE/logs/acceptance/slam_nav/$SESSION_ID}"
MAP_PREFIX="$SESSION_DIR/voice_built_map"
REPORT="$SESSION_DIR/slam_nav_e2e_report.json"
MISSION_PLAN="$WORKSPACE/src/embodied_simulation/config/showcase_workplace_mission.yaml"
WORLD_FILE="$WORKSPACE/src/embodied_simulation/worlds/showcase_apartment.sdf.xacro"
DYNAMIC_SCENARIO="$WORKSPACE/src/embodied_navigation/config/showcase_dynamic_obstacle_scenario.json"
NODE_LOG="$SESSION_DIR/runtime.log"
GATE_TIMEOUT_S="${SLAM_NAV_GATE_TIMEOUT_S:-900}"
TRANSITION_TIMEOUT_S="${SLAM_NAV_TRANSITION_TIMEOUT_S:-860}"
PROGRESS_HEARTBEAT_S="${SLAM_NAV_PROGRESS_HEARTBEAT_S:-15}"
mkdir -p "$SESSION_DIR"
rm -f "$MAP_PREFIX.yaml" "$MAP_PREFIX.pgm" "$REPORT"

echo "SLAM/Nav2 end-to-end acceptance"
echo "  session: $SESSION_ID"
echo "  scene: showcase_apartment"
echo "  stages: frontier SLAM -> fresh map -> AMCL/Nav2 -> semantic patrol -> dynamic replan"
echo "  heartbeat: ${PROGRESS_HEARTBEAT_S}s"
echo "  evidence: $REPORT"
echo "  runtime log: $NODE_LOG"

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
}
trap cleanup EXIT INT TERM

if ! timeout "$GATE_TIMEOUT_S" bash tests/integration/run_probe.sh tools/acceptance/probes/slam_nav/session_orchestrator.py \
  --output "$REPORT" \
  --transition-timeout "$TRANSITION_TIMEOUT_S" \
  --gate-timeout-s "$GATE_TIMEOUT_S" \
  --progress-heartbeat-s "$PROGRESS_HEARTBEAT_S" \
  --runtime-log "$NODE_LOG" \
  --summary-only \
  --evidence-kind gazebo_frontier_slam_map_saver_amcl_nav2_dynamic_replan \
  --session-id "$SESSION_ID" \
  --session-start-ns "$SESSION_START_NS" \
  --world-file "$WORLD_FILE" \
  --mission-plan "$MISSION_PLAN" \
  --dynamic-scenario "$DYNAMIC_SCENARIO" \
  --automatic-mission; then
  FAILURE_LOG_LINES="${SLAM_NAV_FAILURE_LOG_LINES:-120}"
  echo "---- automatic orchestrator log (last ${FAILURE_LOG_LINES} lines) ----" >&2
  echo "Full runtime log: $NODE_LOG" >&2
  tail -n "$FAILURE_LOG_LINES" "$NODE_LOG" >&2
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
assert report["schema_version"] == 3
assert report["session_id"]
assert report["automatic_mission"] is True
assert report["map_saved"] is True
assert report["final_phase"] == 12
assert report["map"]["known_cells"] >= 6000
assert report["map"]["occupied_cells"] >= 150
assert report["mapping_path_m"] >= 10.0
assert report["frontier_goal_count"] >= 1
assert report["exploration_completion_reason"] in {
    "no_frontiers", "coverage_plateau", "time_budget_coverage"
}
assert report["map_provenance"]["yaml_mtime_ns"] >= report["session_start_ns"]
assert report["map_provenance"]["image_mtime_ns"] >= report["session_start_ns"]
assert report["dynamic_navigation"]["passed"] is True
assert all(report["checks"].values()), report["checks"]
follow_results = [
    item for item in report["action_results"]
    if str(item.get("message", "")).startswith("nav2:follow_waypoints:")
]
assert follow_results
assert all("missed_waypoints=0" in item["message"] for item in follow_results)
assert abs(report["final_cmd_vel"]["linear_x"]) < 1e-6
assert abs(report["final_cmd_vel"]["angular_z"]) < 1e-6
print(
    "Verified evidence: "
    f"map={report['map']['known_cells']}/{report['map']['occupied_cells']} cells, "
    f"mapping_path={report['mapping_path_m']:.3f}m, "
    f"frontiers={report['frontier_goal_count']}, "
    f"unique_dynamic_plans={report['dynamic_navigation']['unique_plan_count']}"
)
PY

echo "PASS: one intent -> autonomous frontier SLAM -> saved map -> AMCL/Nav2 patrol"
echo "Evidence: $REPORT"
echo "Runtime log: $NODE_LOG"
