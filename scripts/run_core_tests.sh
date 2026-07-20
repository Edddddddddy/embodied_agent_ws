#!/usr/bin/env bash
set -euo pipefail

WORKSPACE="${WORKSPACE:-/home/ubuntu/embodied_agent_ws}"

source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

echo "[core] acceptance CLI contract"
bash tests/integration/control/test_acceptance_cli.sh
bash tests/integration/voice/test_voice_benchmark_cli.sh

echo "[core] repository and deterministic evaluation tests"
python3 -m pytest -q tests/repository tests/evaluation

echo "[core] Python agent unit tests"
python3 -m pytest -q \
  src/embodied_agent_core/test \
  src/embodied_voice_frontend/test \
  src/embodied_offline_agent/test \
  src/embodied_slam_tools/test

echo "[core] C++/simulation unit tests"
CORE_CPP_PACKAGES=(
  embodied_agent_interfaces
  embodied_agent_cpp
  embodied_simulation
  embodied_navigation
)
CORE_TEST_PACKAGES=(
  embodied_agent_cpp
  embodied_simulation
  embodied_navigation
)

# --allow-overriding 只声明 overlay 规则，并不会筛选构建包；必须显式
# --packages-select，否则 core 会误编译整个工作区并把第三方仓库的 lint 债务混入结果。
colcon build --symlink-install \
  --packages-select "${CORE_CPP_PACKAGES[@]}" \
  --allow-overriding "${CORE_CPP_PACKAGES[@]}"
colcon test --packages-select "${CORE_TEST_PACKAGES[@]}" \
  --event-handlers console_direct+

# 每个自有包单独收集结果，避免 build/ 中历史或 vendor 的 xUnit 文件污染本次门禁。
for package in "${CORE_TEST_PACKAGES[@]}"; do
  colcon test-result --test-result-base "build/$package" --verbose
done

echo "PASS: core unit and structure tests"
