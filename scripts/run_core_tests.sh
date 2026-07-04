#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"

source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

echo "[core] acceptance CLI contract"
bash tests/integration/test_acceptance_cli.sh

echo "[core] repository structure guards"
pytest -q tests/repository

echo "[core] Python agent unit tests"
pytest -q src/embodied_online_agent/test src/embodied_offline_agent/test

echo "[core] C++/simulation unit tests"
colcon build --symlink-install --allow-overriding \
  embodied_agent_interfaces embodied_agent_cpp embodied_simulation
colcon test --packages-select \
  embodied_agent_cpp embodied_simulation \
  --event-handlers console_direct+
colcon test-result --verbose

echo "PASS: core unit and structure tests"
