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
  "gz sim"
  "parameter_bridge"
  "simulation_control_node"
  "typed_action_bridge"
)

found=0
for pattern in "${PATTERNS[@]}"; do
  if pgrep -af "$pattern" >/dev/null 2>&1; then
    found=1
    echo "FOUND: $pattern"
    pgrep -af "$pattern" || true
  fi
done

if [[ "$found" -eq 0 ]]; then
  echo "PASS: no stale Gazebo/ROS simulation processes found"
  exit 0
fi

if [[ "${CLEANUP_CONFIRM:-false}" != "true" ]]; then
  cat <<'EOF'

Stale simulation processes were found.
To terminate them before a fresh demo, run:

  CLEANUP_CONFIRM=true bash scripts/cleanup_simulation_processes.sh

EOF
  exit 2
fi

for pattern in "${PATTERNS[@]}"; do
  pkill -TERM -f "$pattern" 2>/dev/null || true
done
sleep 1
for pattern in "${PATTERNS[@]}"; do
  pkill -KILL -f "$pattern" 2>/dev/null || true
done

echo "PASS: stale Gazebo/ROS simulation processes terminated"
