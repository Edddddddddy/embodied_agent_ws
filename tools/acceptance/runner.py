"""Execute one registered acceptance mode without exposing shell routing logic."""

from __future__ import annotations

import os
import subprocess
from pathlib import Path
from typing import Protocol, Sequence

from .catalog import AcceptanceMode


class ModeRunner(Protocol):
    """Small test seam used by the CLI and repository tests."""

    def run(self, mode: AcceptanceMode, arguments: Sequence[str]) -> int: ...


class BashModeRunner:
    """ROS environment adapter for existing Bash-oriented test implementations.

    The Python catalog owns mode selection. Bash only supplies domain commands that
    still need ROS setup semantics such as ``source install/setup.bash``.
    """

    def __init__(self, workspace: Path) -> None:
        self._workspace = workspace.resolve()

    def run(self, mode: AcceptanceMode, arguments: Sequence[str]) -> int:
        common_library = (
            self._workspace / "tools" / "acceptance" / "handlers" / "common.sh"
        )
        handler_library = self._workspace / mode.handler_library
        if not common_library.is_file():
            raise FileNotFoundError(
                f"acceptance common library not found: {common_library}"
            )
        if not handler_library.is_file():
            raise FileNotFoundError(
                f"acceptance handler library not found: {handler_library}"
            )

        activation = (
            'source "$WORKSPACE/scripts/activate.sh"'
            if mode.requires_ros_environment
            else ":"
        )
        # mode.name 只属于 Python catalog，不伪装成 Bash 的隐藏 `$1`。
        # 用户参数通过 argv 原样传入，避免空格或特殊字符改变路由或被再次执行。
        script = f"""set -euo pipefail
export WORKSPACE="$1"
shift
cd "$WORKSPACE"
{activation}
source "$WORKSPACE/tools/acceptance/handlers/common.sh"
source "$WORKSPACE/{mode.handler_library}"
"{mode.handler}" "$@"
"""
        environment = os.environ.copy()
        environment["ACCEPTANCE_PROGRAM"] = "bash scripts/acceptance_test.sh"
        completed = subprocess.run(
            [
                "bash",
                "--noprofile",
                "--norc",
                "-c",
                script,
                "acceptance-runner",
                str(self._workspace),
                *arguments,
            ],
            cwd=self._workspace,
            env=environment,
            check=False,
        )
        return completed.returncode
