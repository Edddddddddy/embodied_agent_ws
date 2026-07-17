#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Usage: $0 <probe.py> [args...]" >&2
  exit 2
fi

PROBE="$1"
shift
ACCEPTANCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
WORKSPACE="$(cd -- "$ACCEPTANCE_DIR/../.." && pwd -P)"

# 相对 probe 始终相对仓库根解析，避免调用者 cwd 不同导致同一命令时好时坏。
if [[ "$PROBE" != /* ]]; then
  PROBE="$WORKSPACE/$PROBE"
fi

if [[ ! -f "$PROBE" ]]; then
  echo "ERROR: acceptance probe not found: $PROBE" >&2
  exit 2
fi

# Probe 可能依赖 ROS executor，也可能位于 tests/integration；runner 只拥有一致的
# Python 启动语义，不拥有环境激活或 PASS 判定。-u 确保重型门禁输出实时可见，
# 避免管道/重定向下因 stdout 缓冲看起来“卡住”。
export PYTHONPATH="$WORKSPACE${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -u "$PROBE" "$@"
