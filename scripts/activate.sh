#!/usr/bin/env bash
set -e

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"
source /opt/ros/jazzy/setup.bash
if [[ -f "$WORKSPACE/.venv/bin/activate" ]]; then
  source "$WORKSPACE/.venv/bin/activate"
fi
source "$WORKSPACE/install/setup.bash"

