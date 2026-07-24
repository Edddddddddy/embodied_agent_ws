#!/usr/bin/env bash
set -euo pipefail

# Clean stale Gazebo/ROS simulation processes from previous demos.
#
# This is intentionally a separate script instead of being hidden in every launch:
# killing Gazebo globally is useful for this single-user WSL demo workspace, but it
# should still be an explicit operation so we do not surprise someone running a
# different simulation.

PATTERNS=(
  "ros2 launch embodied_simulation voice_turtlebot3.launch.py"
  "ros2 launch embodied_simulation voice_nav2_turtlebot3.launch.py"
  "ros2 launch embodied_simulation persistent_voice_nav_base.launch.py"
  "ros2 launch embodied_simulation persistent_mapping_stage.launch.py"
  "ros2 launch embodied_simulation persistent_navigation_stage.launch.py"
  "gz sim"
  "/ros_gz_bridge/parameter_bridge([[:space:]]|$)"
  "/embodied_simulation/simulation_control_node([[:space:]]|$)"
  "/embodied_agent_cpp/typed_action_bridge([[:space:]]|$)"
  "/embodied_agent_cpp/action_guard([[:space:]]|$)"
  "/embodied_agent_cpp/control_authority([[:space:]]|$)"
  "/embodied_agent_cpp/velocity_authority_gate([[:space:]]|$)"
  "/embodied_agent_cpp/keyboard_teleop([[:space:]]|$)"
  "/embodied_navigation/dynamic_obstacle_tracker_node([[:space:]]|$)"
  "/twist_mux/twist_mux([[:space:]]|$)"
  "/nav2_controller/controller_server([[:space:]]|$)"
  "/nav2_bt_navigator/bt_navigator([[:space:]]|$)"
  "/nav2_behaviors/behavior_server([[:space:]]|$)"
  "/nav2_collision_monitor/collision_monitor([[:space:]]|$)"
  "/nav2_map_server/map_server([[:space:]]|$)"
  "/nav2_amcl/amcl([[:space:]]|$)"
  "/nav2_velocity_smoother/velocity_smoother([[:space:]]|$)"
  "/robot_state_publisher/robot_state_publisher([[:space:]]|$)"
  "/rviz2/rviz2([[:space:]]|$)"
  "/rclcpp_components/component_container(_isolated)?([[:space:]]|$)"
  "ros2 run embodied_slam_tools voice_slam_session_orchestrator"
  "/embodied_slam_tools/voice_slam_session_orchestrator([[:space:]]|$)"
  "tools.acceptance.scenarios.unknown_world_slam_e2e"
  "tools.acceptance.scenarios.showcase_gazebo_e2e"
  "/tools/acceptance/probes/slam_nav/session_orchestrator.py"
  "nav2_lifecycle_manager/lifecycle_manager"
  "slam_toolbox.*slam_toolbox_node"
  "explore_lite/explore"
)

declare -A TARGET_COMMANDS=()

collect_targets() {
  local pattern pid command
  for pattern in "${PATTERNS[@]}"; do
    while read -r pid; do
      [[ -n "$pid" ]] || continue
      [[ "$pid" != "$$" && "$pid" != "$PPID" ]] || continue
      command="$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)"
      [[ -n "$command" ]] || continue
      # 调用方可能在同一条 bash -c 中写了匹配词；不能把清理器自己当成 Gazebo。
      [[ "$command" != *"cleanup_simulation_processes.sh"* ]] || continue
      TARGET_COMMANDS["$pid"]="$command"
    done < <(pgrep -f -- "$pattern" 2>/dev/null || true)
  done
}

collect_targets

if [[ "${#TARGET_COMMANDS[@]}" -eq 0 ]]; then
  echo "PASS: no stale Gazebo/ROS simulation processes found"
  exit 0
fi

echo "FOUND: ${#TARGET_COMMANDS[@]} stale Gazebo/ROS process(es)"
for pid in "${!TARGET_COMMANDS[@]}"; do
  printf '  %s %s\n' "$pid" "${TARGET_COMMANDS[$pid]}"
done

if [[ "${CLEANUP_CONFIRM:-false}" != "true" ]]; then
  cat <<'EOF'

Stale simulation processes were found.
To terminate them before a fresh demo, run:

  CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh

EOF
  exit 2
fi

for pid in "${!TARGET_COMMANDS[@]}"; do
  # 先结束外层验收器，避免 ROS 子进程已清理后它仍等待 40 分钟并写回旧报告。
  kill -TERM "$pid" 2>/dev/null || true
done
for _ in $(seq 1 20); do
  alive=0
  for pid in "${!TARGET_COMMANDS[@]}"; do
    kill -0 "$pid" 2>/dev/null && alive=1
  done
  [[ "$alive" -eq 0 ]] && break
  sleep 0.1
done
for pid in "${!TARGET_COMMANDS[@]}"; do
  kill -KILL "$pid" 2>/dev/null || true
done

echo "PASS: stale Gazebo/ROS simulation processes terminated"
