"""采集可跨阶段比较的 Linux 进程身份.

只比较 PID 会遭遇 PID 复用；持久演示同时绑定内核启动 ID 和 `/proc/<pid>/stat`
中的 starttime，才能证明 Gazebo/Agent 在建图切换导航时没有被悄悄重启。
"""

from __future__ import annotations

import hashlib
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping


class ProcessIdentityUnavailable(RuntimeError):
    """目标进程不存在，或其 `/proc` 身份无法完整读取."""


@dataclass(frozen=True)
class ProcessIdentity:
    """一个 Linux 进程在本次系统启动周期内的稳定身份."""

    pid: int
    start_ticks: int
    boot_id: str
    executable: str
    argv_sha256: str

    def as_evidence(self) -> dict[str, int | str]:
        """生成安全证据；命令行只保留摘要，避免泄露凭据."""
        return {
            'pid': self.pid,
            'start_ticks': self.start_ticks,
            'boot_id': self.boot_id,
            'executable': self.executable,
            'argv_sha256': self.argv_sha256,
        }

    def is_current(self) -> bool:
        """确认 PID 仍指向采集时的同一活进程，而非复用后的新进程."""
        try:
            return self.capture(self.pid) == self
        except ProcessIdentityUnavailable:
            return False

    @classmethod
    def capture(cls, pid: int) -> 'ProcessIdentity':
        """从 procfs 读取并绑定进程身份."""
        if pid <= 0:
            raise ProcessIdentityUnavailable(f'invalid pid: {pid}')
        previous: ProcessIdentity | None = None
        last_error: ProcessIdentityUnavailable | None = None
        # Popen 返回时子进程可能仍处于 fork→exec 窗口。要求两次连续样本一致，
        # 避免把临时的父解释器 argv/exe 误写成常驻 Gazebo 身份。
        for _ in range(10):
            try:
                current = cls._capture_once(pid)
            except ProcessIdentityUnavailable as error:
                last_error = error
                time.sleep(0.005)
                continue
            if current == previous:
                return current
            previous = current
            time.sleep(0.005)
        if previous is not None:
            raise ProcessIdentityUnavailable(
                f'process identity did not stabilize for pid {pid}'
            )
        raise last_error or ProcessIdentityUnavailable(
            f'cannot capture process identity for pid {pid}'
        )

    @classmethod
    def _capture_once(cls, pid: int) -> 'ProcessIdentity':
        proc = Path('/proc') / str(pid)
        try:
            stat = (proc / 'stat').read_text(encoding='utf-8')
            # comm 字段允许包含空格和括号，必须从最后一个右括号之后解析。
            fields = stat[stat.rfind(')') + 2:].split()
            if len(fields) <= 19 or fields[0] == 'Z':
                raise ProcessIdentityUnavailable(f'process is not live: {pid}')
            start_ticks = int(fields[19])
            boot_id = Path('/proc/sys/kernel/random/boot_id').read_text(
                encoding='utf-8'
            ).strip()
            executable = str((proc / 'exe').resolve(strict=True))
            argv = (proc / 'cmdline').read_bytes()
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
            ValueError,
        ) as exc:
            raise ProcessIdentityUnavailable(
                f'cannot capture process identity for pid {pid}'
            ) from exc
        if not boot_id or not argv:
            raise ProcessIdentityUnavailable(
                f'incomplete process identity for pid {pid}'
            )
        return cls(
            pid=pid,
            start_ticks=start_ticks,
            boot_id=boot_id,
            executable=executable,
            argv_sha256=hashlib.sha256(argv).hexdigest(),
        )


@dataclass(frozen=True)
class RuntimeCheckpoint:
    """一次阶段边界的进程身份快照."""

    label: str
    wall_ns: int
    roles: Mapping[str, ProcessIdentity]

    @classmethod
    def capture(
        cls,
        label: str,
        role_pids: Mapping[str, int],
        *,
        wall_ns: int | None = None,
    ) -> 'RuntimeCheckpoint':
        """采集一组具名运行时角色."""
        rendered_label = label.strip()
        if not rendered_label:
            raise ValueError('runtime checkpoint label must not be empty')
        if not role_pids:
            raise ValueError('runtime checkpoint requires at least one role')
        roles = {
            role: ProcessIdentity.capture(pid)
            for role, pid in sorted(role_pids.items())
        }
        return cls(
            label=rendered_label,
            wall_ns=time.time_ns() if wall_ns is None else wall_ns,
            roles=roles,
        )

    def as_evidence(self) -> dict[str, object]:
        """生成可直接序列化的阶段证据."""
        return {
            'label': self.label,
            'wall_ns': self.wall_ns,
            'roles': {
                role: identity.as_evidence()
                for role, identity in sorted(self.roles.items())
            },
        }
