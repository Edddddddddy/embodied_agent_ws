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

# 部分 Python 评测会直接导入自定义 ROS 消息。linked worktree 不能复用主工作区
# 的旧接口产物，因此必须先构建接口，再启动 pytest；否则会出现“源码已更新，
# Python 却从旧 install 导入消息”的隐蔽环境污染。
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

  # build 完成后重新加载当前 worktree 的 overlay，确保后续 pytest 优先使用刚生成
  # 的消息模块，而不是 activate.sh 初次加载时记录的其它工作区 underlay。
  # colcon 的 setup 脚本会读取可选环境变量，加载期间需暂时关闭 nounset。
  set +u
  source "$WORKSPACE/install/setup.bash"
  set -u
fi

# 直接运行 pytest 时，ament 不会像 colcon test 一样自动选择当前包的源码。
# 显式把本 worktree 的 Python 包放在最前面，防止同机其它 worktree 的旧
# install 覆盖当前实现；generated ROS interfaces 仍由上面的 overlay 提供。
CORE_PYTHON_SOURCE_PACKAGES=(
  embodied_agent_core
  embodied_agent_middleware
  embodied_voice_frontend
  embodied_offline_agent
  embodied_online_agent
  embodied_slam_tools
)
for package in "${CORE_PYTHON_SOURCE_PACKAGES[@]}"; do
  export PYTHONPATH="$WORKSPACE/src/$package${PYTHONPATH:+:$PYTHONPATH}"
done

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
  src/embodied_online_agent/test \
  src/embodied_slam_tools/test

echo "[core] C++/simulation unit tests"
colcon test --packages-select "${CORE_TEST_PACKAGES[@]}" \
  --event-handlers console_direct+

# 每个自有包单独收集结果，避免 build/ 中历史或 vendor 的 xUnit 文件污染本次门禁。
for package in "${CORE_TEST_PACKAGES[@]}"; do
  colcon test-result --test-result-base "build/$package" --verbose
done

echo "PASS: core unit and structure tests"
