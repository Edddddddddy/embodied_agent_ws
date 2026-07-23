"""验收工具共享的仓库路径定位 Interface。"""

from __future__ import annotations

from pathlib import Path


_REPOSITORY_MARKERS = (
    Path("scripts/acceptance_test.sh"),
    Path("tools/acceptance"),
    Path("src"),
)


def repository_root(start: Path | None = None) -> Path:
    """从任意仓库内路径向上查找根目录，不依赖固定 ``parents[n]``。

    Probe 会按领域迁移目录；若用父目录下标定位 logs/config，移动文件就会静默
    改变运行结果。通过稳定的仓库标记查找，可同时支持主 worktree 和 Git worktree。
    """

    current = (start or Path(__file__)).resolve()
    if current.is_file():
        current = current.parent
    for candidate in (current, *current.parents):
        if all((candidate / marker).exists() for marker in _REPOSITORY_MARKERS):
            return candidate
    raise FileNotFoundError(f"repository root not found from: {current}")
