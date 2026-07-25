"""从运行日志快速识别“任务还在等，但 Nav2 已被关闭”的状态。"""

from __future__ import annotations

from pathlib import Path
import re
from typing import Callable


_NAV2_CRITICAL = re.compile(
    r"CRITICAL FAILURE:\s*SERVER\s+([A-Za-z0-9_/-]+)\s+IS DOWN",
    re.IGNORECASE,
)


class RuntimeLogHealthMonitor:
    """增量读取日志；不在内存中复制整份十几分钟 runtime.log。"""

    def __init__(self, path: Path) -> None:
        self.path = path
        self._offset = 0
        self._pending = ""
        self._failure: str | None = None

    def check(self) -> str | None:
        if self._failure is not None:
            return self._failure
        try:
            size = self.path.stat().st_size
        except FileNotFoundError:
            return None
        if size < self._offset:
            # 日志被重新创建时从头读取，禁止沿用旧 inode 的 offset。
            self._offset = 0
            self._pending = ""
        with self.path.open("r", encoding="utf-8", errors="replace") as stream:
            stream.seek(self._offset)
            chunk = stream.read()
            self._offset = stream.tell()
        if not chunk:
            return None
        text = self._pending + chunk
        lines = text.splitlines(keepends=True)
        self._pending = ""
        if lines and not lines[-1].endswith(("\n", "\r")):
            self._pending = lines.pop()
        for line in lines:
            match = _NAV2_CRITICAL.search(line)
            if match is None:
                continue
            server = match.group(1)
            self._failure = (
                "nav2_runtime_unhealthy: lifecycle manager shut down "
                f"server={server}; inspect resource_samples.jsonl and "
                "runtime.log"
            )
            return self._failure
        # 进程可能在写完 CRITICAL 后立即退出，最后一行没有换行。pending
        # 仍要参与致命模式匹配，否则探针会一直等到任务总 deadline。
        match = _NAV2_CRITICAL.search(self._pending)
        if match is not None:
            server = match.group(1)
            self._failure = (
                "nav2_runtime_unhealthy: lifecycle manager shut down "
                f"server={server}; inspect resource_samples.jsonl and "
                "runtime.log"
            )
            return self._failure
        return None

    def wait_until(
        self,
        *,
        predicate: Callable[[], bool],
        waiter: Callable[[Callable[[], bool], float, str], None],
        timeout_s: float,
        description: str,
    ) -> None:
        """把日志失败并入已有 ROS wait seam，避免入口复制轮询状态机。"""

        runtime_failure: list[str] = []

        def guarded() -> bool:
            failure = self.check()
            if failure is not None:
                runtime_failure[:] = [failure]
                return True
            return predicate()

        waiter(guarded, timeout_s, description)
        if runtime_failure:
            raise RuntimeError(runtime_failure[0])
