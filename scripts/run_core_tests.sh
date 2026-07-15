#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"

source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

echo "[core] acceptance CLI contract"
bash tests/integration/test_acceptance_cli.sh
bash tests/integration/test_voice_benchmark_cli.sh

echo "[core] repository structure guards"
python3 -m pytest -q tests/repository

echo "[core] Python agent unit tests"
python3 -m pytest -q \
  src/embodied_agent_core/test \
  src/embodied_voice_frontend/test \
  src/embodied_offline_agent/test \
  src/embodied_slam_tools/test

echo "[core] C++/simulation unit tests"
colcon build --symlink-install --allow-overriding \
  embodied_agent_interfaces embodied_agent_cpp embodied_simulation embodied_navigation
colcon test --packages-select \
  embodied_agent_cpp embodied_simulation embodied_navigation \
  --event-handlers console_direct+
colcon test-result --verbose

echo "PASS: core unit and structure tests"
