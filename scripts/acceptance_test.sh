#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WORKSPACE="$(cd -- "$SCRIPT_DIR/.." && pwd -P)"

# 入口只负责定位仓库；模式、帮助和路由均由可单测的 Python 模块统一管理。
cd "$WORKSPACE"
exec python3 -m tools.acceptance.cli "$@"
