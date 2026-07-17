#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-$((180 + $$ % 40))}"

LAUNCH_LOG="$(mktemp)"
PIDS=()
cleanup() {
  for pid in "${PIDS[@]}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  sleep 0.5
  for pid in "${PIDS[@]}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
    wait "$pid" 2>/dev/null || true
  done
  rm -f "$LAUNCH_LOG"
}
trap cleanup EXIT

for package in nav2_bringup nav2_msgs nav2_lifecycle_manager nav2_minimal_tb3_sim embodied_simulation; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "MISSING: ROS package $package" >&2
    exit 1
  fi
done

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "启动 Nav2 TurtleBot3 + 语音语义导航链路（重型仿真验收，可能需要数分钟）"

setsid ros2 launch embodied_simulation voice_nav2_turtlebot3.launch.py \
  launch_agent:=true \
  agent_type:="${AGENT_TYPE:-online}" \
  provider_mode:=mock \
  microphone_enabled:=false \
  wake_word_enabled:=false \
  continuous_control_enabled:=false \
  lifecycle_autostart:=true \
  use_rviz:="${USE_RVIZ:-false}" \
  headless:="${HEADLESS:-true}" \
  use_composition:="${USE_COMPOSITION:-true}" \
  >"$LAUNCH_LOG" 2>&1 &
PIDS+=("$!")

PROBE_ARGS=()
if [[ -n "${SKIP_PATROL:-}" ]]; then
  PROBE_ARGS+=(--skip-patrol)
fi
if [[ "${NAV2_RESILIENCE:-false}" == "true" ]]; then
  PROBE_ARGS+=(--resilience)
fi

if ! timeout "${NAV2_TURTLEBOT3_TIMEOUT:-420}" \
  bash "$WORKSPACE/tools/acceptance/run_probe.sh" "$WORKSPACE/tests/integration/slam_nav/test_nav2_turtlebot3_voice.py" \
    "${PROBE_ARGS[@]}"; then
  cat "$LAUNCH_LOG" >&2
  exit 1
fi

if [[ "${NAV2_RESILIENCE:-false}" == "true" ]]; then
  echo "PASS: Nav2 dynamic obstacle replan + unreachable goal failure feedback"
  echo "Evidence: $WORKSPACE/logs/nav2_resilience_report.json"
else
  echo "PASS: voice text command -> Nav2 TurtleBot3 target navigation (optional patrol)"
  echo "Evidence: $WORKSPACE/logs/nav2_turtlebot3_voice_report.json"
fi
