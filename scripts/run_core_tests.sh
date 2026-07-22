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

# Docker build 阶段已经完成普通 install 的全工作区构建；测试阶段不能在同一
# build 目录切换为 symlink-install。宿主机默认仍增量重建，容器显式关闭此步骤。
if [[ "${EMBODIED_CORE_REBUILD:-true}" == "true" ]]; then
  COLCON_OVERRIDE_ARGS=()
  # --allow-overriding 来自可选的 colcon-override-check 插件；开发机有该插件时消除
  # overlay 警告，最小环境没有插件时则安全省略。参数必须放在 packages-select 前。
  if colcon build --help 2>&1 | grep -q -- '--allow-overriding'; then
    COLCON_OVERRIDE_ARGS=(--allow-overriding "${CORE_CPP_PACKAGES[@]}")
  fi
  colcon build --symlink-install \
    "${COLCON_OVERRIDE_ARGS[@]}" \
    --packages-select "${CORE_CPP_PACKAGES[@]}" \
    --executor sequential
fi
colcon test --packages-select "${CORE_TEST_PACKAGES[@]}" \
  --event-handlers console_direct+

# 每个自有包单独收集结果，避免 build/ 中历史或 vendor 的 xUnit 文件污染本次门禁。
for package in "${CORE_TEST_PACKAGES[@]}"; do
  colcon test-result --test-result-base "build/$package" --verbose
done

echo "PASS: core unit and structure tests"
