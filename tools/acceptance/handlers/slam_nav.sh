#!/usr/bin/env bash
# Registered slam_nav acceptance handlers; sourced by the Python runner.

accept_navigation_demo() {
  bash scripts/smoke_test_navigation_sequence.sh online; bash scripts/smoke_test_navigation_sequence.sh offline
}

accept_nav2_bridge() {
  bash scripts/smoke_test_nav2_bridge.sh
}

accept_nav2_preflight() {
  bash scripts/smoke_test_nav2_preflight.sh
}

accept_nav2_assets() {
  python3 scripts/audit_nav2_demo_assets.py
}

accept_nav2_stage() {
  run_isolated_ros_smoke 181 bash scripts/smoke_test_navigation_sequence.sh online
  run_isolated_ros_smoke 182 bash scripts/smoke_test_navigation_sequence.sh offline
  run_isolated_ros_smoke 183 bash scripts/smoke_test_continuous_navigation_queue.sh online
  run_isolated_ros_smoke 184 bash scripts/smoke_test_continuous_navigation_queue.sh offline
  run_isolated_ros_smoke 185 bash scripts/smoke_test_continuous_navigation_natural.sh online
  run_isolated_ros_smoke 186 bash scripts/smoke_test_continuous_navigation_natural.sh offline
  run_isolated_ros_smoke 187 bash scripts/smoke_test_nav2_bridge.sh
  bash scripts/smoke_test_nav2_preflight.sh
}

accept_nav2_turtlebot3() {
  bash scripts/smoke_test_nav2_turtlebot3_voice.sh
}

accept_nav2_resilience() {
  NAV2_RESILIENCE=true SKIP_PATROL=1 \
    bash scripts/smoke_test_nav2_turtlebot3_voice.sh
}

accept_mapping_stage() {
  python3 scripts/audit_slam_mapping_assets.py
  colcon build --packages-select embodied_slam --symlink-install
  colcon test --packages-select embodied_slam --event-handlers console_direct+
  colcon test-result --test-result-base build/embodied_slam --verbose
}

accept_slam_nav_showcase_stage() {
  embodied_workspace_doctor true
  python3 scripts/generate_showcase_scene.py --check
  python3 -m pytest -q tests/repository/test_showcase_scene.py
  PYTHONPATH="$WORKSPACE/src/embodied_agent_core${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -m pytest -q src/embodied_agent_core/test/test_command_nlu.py
  WORKSPACE="$WORKSPACE" bash scripts/voice_slam_nav_showcase.sh audit
}

accept_slam_nav_showcase() {
  bash scripts/smoke_test_slam_nav_showcase.sh
}

accept_slam_nav_showcase_mapping() {
  bash scripts/smoke_test_slam_nav_showcase_mapping.sh
}

accept_slam_session_orchestrator_stage() {
  PYTHONPATH="$WORKSPACE/src/embodied_slam_tools${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -m pytest -q src/embodied_slam_tools/test/test_showcase_session.py
  colcon build --symlink-install --packages-up-to embodied_slam_tools \
    --allow-overriding embodied_agent_interfaces embodied_slam_tools
  WORKSPACE="$WORKSPACE" bash scripts/smoke_test_voice_slam_session_orchestrator.sh
}

accept_slam_autonomous_mission_stage() {
  PYTHONPATH="$WORKSPACE/src/embodied_slam_tools${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -m pytest -q src/embodied_slam_tools/test/test_showcase_session.py
  colcon build --symlink-install --packages-up-to embodied_slam_tools \
    --allow-overriding embodied_agent_interfaces embodied_slam_tools
  PYTHONPATH="$WORKSPACE/src/embodied_slam_tools${PYTHONPATH:+:$PYTHONPATH}" \
    python3 -m pytest -q \
      src/embodied_slam_tools/test/test_showcase_session_node.py \
      tests/integration/slam_nav/test_trigger_automatic_slam_mission.py
  WORKSPACE="$WORKSPACE" bash scripts/smoke_test_voice_slam_automatic_mission.sh
  WORKSPACE="$WORKSPACE" bash scripts/smoke_test_voice_slam_automatic_cancel.sh
}

accept_slam_autonomous_mission() {
  embodied_workspace_doctor true
  WORKSPACE="$WORKSPACE" bash scripts/smoke_test_voice_slam_automatic_mission_gazebo.sh
}

accept_slam_nav_e2e() {
  embodied_workspace_doctor true
  WORKSPACE="$WORKSPACE" bash scripts/smoke_test_voice_slam_automatic_mission_gazebo.sh
}

accept_slam_session_orchestrator() {
  bash scripts/smoke_test_voice_slam_session_orchestrator_gazebo.sh
}

accept_slam_navigation() {
  bash scripts/smoke_test_slam_localization_navigation.sh
}

accept_lidar_loop_runtime() {
  colcon build --packages-up-to embodied_slam --symlink-install
  bash scripts/smoke_test_lidar_loop_runtime.sh
}

accept_dynamic_obstacle_stage() {
  python3 scripts/build_slam_nav2_params.py --output logs/slam_nav2_params.yaml
  colcon build --packages-up-to embodied_navigation --symlink-install --allow-overriding \
    embodied_agent_interfaces embodied_navigation
  colcon test --packages-select embodied_navigation --event-handlers console_direct+
  colcon test-result --test-result-base build/embodied_navigation --verbose
  set +u
  source install/setup.bash
  set -u
  bash scripts/smoke_test_dynamic_obstacle_tracker.sh
  run_dynamic_obstacle_ablation
}

accept_dynamic_obstacle_navigation() {
  bash scripts/smoke_test_predicted_dynamic_obstacle_navigation.sh
}

accept_continuous_navigation() {
  bash scripts/smoke_test_continuous_navigation_queue.sh online; bash scripts/smoke_test_continuous_navigation_queue.sh offline
}

accept_continuous_navigation_natural() {
  bash scripts/smoke_test_continuous_navigation_natural.sh online; bash scripts/smoke_test_continuous_navigation_natural.sh offline
}

accept_continuous_nav2_offline() {
  bash scripts/continuous_nav2_voice_control.sh offline
}

accept_continuous_nav2_online() {
  bash scripts/continuous_nav2_voice_control.sh online
}

accept_voice_slam_workplace_demo() {
  CHECK_MODE="${2:-offline}"
  if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
    echo "Usage: ${ACCEPTANCE_PROGRAM:-acceptance_test.sh} voice-slam-workplace-demo {offline|online}" >&2
    exit 2
  fi
  WORKSPACE="$WORKSPACE" bash scripts/voice_slam_nav_showcase.sh auto "$CHECK_MODE"
}

accept_continuous_nav2_evidence() {
  CHECK_MODE="${2:-offline}"
  if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
    echo "Usage: ${ACCEPTANCE_PROGRAM:-acceptance_test.sh} continuous-nav2-evidence {offline|online}" >&2
    exit 2
  fi
  bash scripts/continuous_nav2_voice_evidence.sh "$CHECK_MODE"
}

accept_continuous_nav2_live_check() {
  CHECK_MODE="${2:-offline}"
  if [[ "$CHECK_MODE" != "offline" && "$CHECK_MODE" != "online" ]]; then
    echo "Usage: ${ACCEPTANCE_PROGRAM:-acceptance_test.sh} continuous-nav2-live-check {offline|online}" >&2
    exit 2
  fi
  echo "continuous-nav2-live-check=$CHECK_MODE：请先在另一个终端启动 acceptance_test.sh continuous-nav2-$CHECK_MODE"
  LIVE_CHECK_ARGS=(
    --scenario nav2 \
    --agent-mode "$CHECK_MODE" \
    --duration "${CONTINUOUS_LIVE_CHECK_DURATION:-240}" \
    --min-asr "${CONTINUOUS_NAV2_LIVE_MIN_ASR:-4}" \
    --min-candidates "${CONTINUOUS_NAV2_LIVE_MIN_CANDIDATES:-2}" \
    --min-success "${CONTINUOUS_NAV2_LIVE_MIN_SUCCESS:-2}" \
    --require-candidate navigate_to \
    --require-candidate follow_waypoints \
    --require-navigation-details
  )
  if [[ -n "${CONTINUOUS_LIVE_CHECK_REPORT:-}" ]]; then
    LIVE_CHECK_ARGS+=(--output "$CONTINUOUS_LIVE_CHECK_REPORT")
  fi
  python3 scripts/continuous_live_check.py "${LIVE_CHECK_ARGS[@]}"
}

accept_continuous_nav2_live_report() {
  REPORT_PATH="${2:-}"
  if [[ -z "$REPORT_PATH" ]]; then
    echo "Usage: ${ACCEPTANCE_PROGRAM:-acceptance_test.sh} continuous-nav2-live-report REPORT_FILE" >&2
    exit 2
  fi
  python3 scripts/continuous_live_check.py \
    --scenario nav2 \
    --input-report "$REPORT_PATH" \
    --min-asr "${CONTINUOUS_NAV2_LIVE_MIN_ASR:-4}" \
    --min-candidates "${CONTINUOUS_NAV2_LIVE_MIN_CANDIDATES:-2}" \
    --min-success "${CONTINUOUS_NAV2_LIVE_MIN_SUCCESS:-2}" \
    --require-candidate navigate_to \
    --require-candidate follow_waypoints \
    --require-navigation-details
}
