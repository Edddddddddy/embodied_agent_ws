#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$SCRIPT_DIR/lifecycle_utils.sh"
embodied_resolve_workspace "${BASH_SOURCE[0]}"
embodied_resolve_runtime_root

source "$WORKSPACE/scripts/activate.sh"
cd "$WORKSPACE"

# linked worktree 只隔离代码与 install，不复制数 GB 的模型和 Python 环境。
# core 的解释器因此优先取当前工作区，其次复用 Git 主工作区 runtime root；
# 仍显式调用选定解释器，避免未激活 venv 时悄悄退回缺依赖的系统 Python。
PYTHON_BIN="${CORE_PYTHON_BIN:-}"
if [[ -z "$PYTHON_BIN" && -x "$WORKSPACE/.venv/bin/python" ]]; then
  PYTHON_BIN="$WORKSPACE/.venv/bin/python"
fi
if [[ -z "$PYTHON_BIN" && -x "$EMBODIED_RUNTIME_ROOT/.venv/bin/python" ]]; then
  PYTHON_BIN="$EMBODIED_RUNTIME_ROOT/.venv/bin/python"
fi
if [[ -z "$PYTHON_BIN" ]]; then
  PYTHON_BIN="$(command -v python3)"
fi

echo "[core] acceptance CLI contract"
bash tests/integration/control/test_acceptance_cli.sh
bash tests/integration/voice/test_voice_benchmark_cli.sh

echo "[core] repository and deterministic evaluation tests"
"$PYTHON_BIN" -m pytest -q tests/repository tests/evaluation

echo "[core] Python agent unit tests"
"$PYTHON_BIN" -m pytest -q \
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
