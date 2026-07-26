"""Linux/WSL 验收进程监管 Adapter。

本模块只负责 Popen、进程组和 subreaper；ROS domain、证据目录与场景状态由上层拥有。
"""

from __future__ import annotations

import ctypes
import os
import signal
import subprocess
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence, TextIO

from tools.acceptance.errors import AcceptanceSessionError


@dataclass(frozen=True, slots=True)
class RunResult:
    returncode: int
    timed_out: bool = False


@dataclass(frozen=True, slots=True)
class StopResult:
    returncode: int | None
    graceful: bool
    killed: bool


@dataclass(frozen=True, slots=True)
class SessionCleanupResult:
    observed_pids: tuple[int, ...] = ()
    killed: bool = False
    remaining_pids: tuple[int, ...] = ()


class ProcessAdapter(Protocol):
    """进程实现 seam；单元测试使用 fake，WSL 使用 subprocess Adapter。"""

    def spawn(
        self,
        argv: Sequence[str],
        *,
        cwd: Path,
        env: Mapping[str, str],
        log_path: Path,
    ) -> Any: ...

    def run(
        self,
        argv: Sequence[str],
        *,
        cwd: Path,
        env: Mapping[str, str],
        timeout_s: float,
    ) -> RunResult: ...

    def stop(self, handle: Any, *, grace_s: float) -> StopResult: ...

    def begin_session(self) -> None: ...

    def finish_session(self, *, grace_s: float) -> SessionCleanupResult: ...


@dataclass(slots=True)
class _SubprocessHandle:
    process: subprocess.Popen[bytes]
    stream: TextIO | None
    process_group_id: int


@dataclass(frozen=True, slots=True)
class _ProcessIdentity:
    pid: int
    process_group_id: int = field(compare=False)
    start_ticks: int
    state: str = field(compare=False)


class SubprocessProcessAdapter:
    """以独立 POSIX 进程组运行命令，退出时回收整棵子进程树。"""

    _PR_SET_CHILD_SUBREAPER = 36
    _PR_GET_CHILD_SUBREAPER = 37

    def __init__(self) -> None:
        self._session_active = False
        self._previous_subreaper = False
        self._baseline_descendants: set[_ProcessIdentity] = set()

    @staticmethod
    def _start(
        argv: Sequence[str],
        *,
        cwd: Path,
        env: Mapping[str, str],
        stream: TextIO | None,
    ) -> _SubprocessHandle:
        process = subprocess.Popen(
            list(argv),
            cwd=cwd,
            env=dict(env),
            stdout=stream,
            stderr=subprocess.STDOUT if stream is not None else None,
            start_new_session=True,
        )
        return _SubprocessHandle(process, stream, process.pid)

    def spawn(
        self,
        argv: Sequence[str],
        *,
        cwd: Path,
        env: Mapping[str, str],
        log_path: Path,
    ) -> _SubprocessHandle:
        log_path.parent.mkdir(parents=True, exist_ok=True)
        stream = log_path.open("a", encoding="utf-8", buffering=1)
        try:
            return self._start(argv, cwd=cwd, env=env, stream=stream)
        except BaseException:
            stream.close()
            raise

    def run(
        self,
        argv: Sequence[str],
        *,
        cwd: Path,
        env: Mapping[str, str],
        timeout_s: float,
    ) -> RunResult:
        handle = self._start(argv, cwd=cwd, env=env, stream=None)
        try:
            return RunResult(handle.process.wait(timeout=timeout_s))
        except subprocess.TimeoutExpired:
            stopped = self.stop(handle, grace_s=2.0)
            return RunResult(
                stopped.returncode
                if stopped.returncode is not None
                else -signal.SIGKILL,
                timed_out=True,
            )
        except BaseException:
            # Ctrl-C 也必须回收探针进程组，否则下一轮仍会看到旧 ROS 节点。
            self.stop(handle, grace_s=2.0)
            raise

    @staticmethod
    def _group_exists(process_group_id: int) -> bool:
        try:
            os.killpg(process_group_id, 0)
            return True
        except ProcessLookupError:
            return False

    @staticmethod
    def _child_process_ids(process_id: int) -> set[int]:
        """汇总所有线程创建的子进程，而不只读取主线程 children。"""

        children: set[int] = set()
        task_root = Path(f"/proc/{process_id}/task")
        try:
            task_directories = tuple(task_root.iterdir())
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            return children
        for task_directory in task_directories:
            try:
                children.update(
                    int(value)
                    for value in (task_directory / "children").read_text().split()
                )
            except (FileNotFoundError, ProcessLookupError, PermissionError):
                continue
        return children

    @classmethod
    def _descendant_process_ids(cls, root_pid: int) -> set[int]:
        descendants: set[int] = set()
        pending = [root_pid]
        while pending:
            process_id = pending.pop()
            if process_id in descendants:
                continue
            descendants.add(process_id)
            pending.extend(cls._child_process_ids(process_id))
        return descendants

    @staticmethod
    def _process_identity(process_id: int) -> _ProcessIdentity | None:
        try:
            raw = Path(f"/proc/{process_id}/stat").read_text()
            fields = raw[raw.rfind(")") + 2 :].split()
            return _ProcessIdentity(
                pid=process_id,
                process_group_id=int(fields[2]),
                start_ticks=int(fields[19]),
                state=fields[0],
            )
        except (FileNotFoundError, ProcessLookupError, PermissionError, ValueError):
            return None

    @classmethod
    def _process_groups(cls, root_pid: int) -> set[int]:
        """读取 Linux `/proc` 进程树，包含子进程自行 setsid 后的新组。"""

        groups: set[int] = set()
        for process_id in cls._descendant_process_ids(root_pid):
            identity = cls._process_identity(process_id)
            if identity is not None:
                groups.add(identity.process_group_id)
        return groups

    @staticmethod
    def _signal_groups(process_group_ids: set[int], signal_number: int) -> None:
        for process_group_id in process_group_ids:
            try:
                os.killpg(process_group_id, signal_number)
            except ProcessLookupError:
                continue

    @classmethod
    def _get_subreaper(cls) -> bool:
        value = ctypes.c_int()
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(
            cls._PR_GET_CHILD_SUBREAPER, ctypes.byref(value), 0, 0, 0
        ) != 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number))
        return bool(value.value)

    @classmethod
    def _set_subreaper(cls, enabled: bool) -> None:
        libc = ctypes.CDLL(None, use_errno=True)
        if libc.prctl(cls._PR_SET_CHILD_SUBREAPER, int(enabled), 0, 0, 0) != 0:
            error_number = ctypes.get_errno()
            raise OSError(error_number, os.strerror(error_number))

    def _current_session_descendants(self) -> set[_ProcessIdentity]:
        identities: set[_ProcessIdentity] = set()
        for process_id in self._descendant_process_ids(os.getpid()):
            if process_id == os.getpid():
                continue
            identity = self._process_identity(process_id)
            if identity is not None and identity not in self._baseline_descendants:
                identities.add(identity)
        return identities

    @staticmethod
    def _reap_adopted(identities: set[_ProcessIdentity]) -> None:
        for identity in identities:
            try:
                os.waitpid(identity.pid, os.WNOHANG)
            except (ChildProcessError, ProcessLookupError):
                continue

    def begin_session(self) -> None:
        if self._session_active:
            raise AcceptanceSessionError("process adapter session is already active")
        self._previous_subreaper = self._get_subreaper()
        try:
            if not self._previous_subreaper:
                self._set_subreaper(True)
            self._baseline_descendants = {
                identity
                for process_id in self._descendant_process_ids(os.getpid())
                if process_id != os.getpid()
                if (identity := self._process_identity(process_id)) is not None
            }
            self._session_active = True
        except BaseException:
            if not self._previous_subreaper:
                self._set_subreaper(False)
            raise

    def finish_session(self, *, grace_s: float) -> SessionCleanupResult:
        if not self._session_active:
            return SessionCleanupResult()
        observed: set[_ProcessIdentity] = set()
        killed = False
        own_group = os.getpgrp()
        try:
            current = self._current_session_descendants()
            observed.update(current)
            self._signal_groups(
                {
                    identity.process_group_id
                    for identity in current
                    if identity.process_group_id != own_group
                },
                signal.SIGTERM,
            )
            deadline = time.monotonic() + max(0.0, grace_s)
            while time.monotonic() < deadline:
                self._reap_adopted(current)
                refreshed = self._current_session_descendants()
                unseen = refreshed - observed
                if unseen:
                    self._signal_groups(
                        {
                            identity.process_group_id
                            for identity in unseen
                            if identity.process_group_id != own_group
                        },
                        signal.SIGTERM,
                    )
                    observed.update(unseen)
                current = refreshed
                if not current:
                    break
                time.sleep(0.05)

            self._reap_adopted(current)
            current = self._current_session_descendants()
            if current:
                killed = True
                self._signal_groups(
                    {
                        identity.process_group_id
                        for identity in current
                        if identity.process_group_id != own_group
                    },
                    signal.SIGKILL,
                )
                kill_deadline = time.monotonic() + 2.0
                while current and time.monotonic() < kill_deadline:
                    self._reap_adopted(current)
                    time.sleep(0.05)
                    current = self._current_session_descendants()
                    observed.update(current)
            self._reap_adopted(current)
            remaining = self._current_session_descendants()
            return SessionCleanupResult(
                observed_pids=tuple(sorted(identity.pid for identity in observed)),
                killed=killed,
                remaining_pids=tuple(
                    sorted(identity.pid for identity in remaining)
                ),
            )
        finally:
            try:
                if not self._previous_subreaper:
                    self._set_subreaper(False)
            finally:
                self._baseline_descendants.clear()
                self._session_active = False

    def stop(self, handle: _SubprocessHandle, *, grace_s: float) -> StopResult:
        killed = False
        try:
            process_groups = self._process_groups(handle.process.pid)
            process_groups.add(handle.process_group_id)
            if any(self._group_exists(group_id) for group_id in process_groups):
                # StageProcessManager 可能给 Gazebo/Nav2 再做 setsid；只杀最外层
                # process group 会留下孤儿 server，因此先记录整棵 /proc 子树的组。
                self._signal_groups(process_groups, signal.SIGTERM)
                deadline = time.monotonic() + max(0.0, grace_s)
                while time.monotonic() < deadline:
                    # poll() 会回收已退出的组长；否则 zombie 仍会让 killpg(0)
                    # 看起来“进程组存活”，每次清理都会无谓等待完整 grace period。
                    handle.process.poll()
                    new_groups = self._process_groups(handle.process.pid)
                    unseen_groups = new_groups - process_groups
                    if unseen_groups:
                        self._signal_groups(unseen_groups, signal.SIGTERM)
                        process_groups.update(unseen_groups)
                    if not any(
                        self._group_exists(group_id)
                        for group_id in process_groups
                    ):
                        break
                    time.sleep(0.05)
                remaining_groups = {
                    group_id
                    for group_id in process_groups
                    if self._group_exists(group_id)
                }
                if remaining_groups:
                    killed = True
                    self._signal_groups(remaining_groups, signal.SIGKILL)
            try:
                returncode = handle.process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                returncode = handle.process.poll()
            return StopResult(returncode, graceful=not killed, killed=killed)
        except ProcessLookupError:
            return StopResult(handle.process.poll(), graceful=True, killed=False)
        finally:
            if handle.stream is not None and not handle.stream.closed:
                handle.stream.close()
