#!/usr/bin/env bash
set -euo pipefail
WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source "$WORKSPACE/scripts/activate.sh"

required_packages=(
  nav2_bringup
  nav2_msgs
  nav2_lifecycle_manager
  nav2_minimal_tb3_sim
  embodied_simulation
)

for package in "${required_packages[@]}"; do
  if ! ros2 pkg prefix "$package" >/dev/null 2>&1; then
    echo "MISSING: ROS package $package" >&2
    exit 1
  fi
  echo "READY: $package"
done

OUTPUT="$(ros2 launch embodied_simulation voice_nav2_turtlebot3.launch.py --show-args)"
grep -q "launch_agent" <<<"$OUTPUT"
grep -q "agent_type" <<<"$OUTPUT"
grep -q "map" <<<"$OUTPUT"
grep -q "params_file" <<<"$OUTPUT"
grep -q "use_rviz" <<<"$OUTPUT"
grep -q "headless" <<<"$OUTPUT"
grep -q "nav_action_timeout_s" <<<"$OUTPUT"
grep -q "action_timeout_s" <<<"$OUTPUT"

test -s "$WORKSPACE/src/embodied_simulation/config/places.yaml"

echo "PASS: Nav2 TurtleBot3 voice launch dependencies and arguments are ready"
