"""重型验收共用的会话生命周期 Module。

公开验收模式只需要声明要启动的进程与探针；ROS domain、Gazebo partition、
进程组清理、超时和失败日志统一由本模块实现。这样卡住或 Ctrl-C 时不会把一套
残留 ROS graph 留给下一次验收。
"""

from __future__ import annotations

import ctypes
import fcntl
import hashlib
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Protocol, Sequence, TextIO


class AcceptanceSessionError(RuntimeError):
    """验收会话无法安全继续。"""


class DomainLeaseUnavailable(AcceptanceSessionError):
    """候选 ROS domain 均被其他验收会话占用。"""


class ArtifactLeaseUnavailable(AcceptanceSessionError):
    """证据目录正在被另一验收会话写入。"""


class AcceptanceCommandError(AcceptanceSessionError):
    """会话内命令失败或超过截止时间。"""


class AcceptanceSignalError(AcceptanceSessionError):
    """SIGTERM/SIGHUP 被转换成可穿过 context manager 的异常。"""

    def __init__(self, signal_number: int) -> None:
        self.signal_number = signal_number
        super().__init__(f"acceptance interrupted by {signal.Signals(signal_number).name}")


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


@dataclass(slots=True)
class RosDomainLease:
    domain_id: int
    path: Path
    _descriptor: int
    _released: bool = False

    def release(self) -> None:
        if self._released:
            return
        fcntl.flock(self._descriptor, fcntl.LOCK_UN)
        os.close(self._descriptor)
        self._released = True


class RosDomainLeasePool:
    """用 ``flock`` 分配 ROS domain，避免 PID 取模导致并行碰撞。"""

    def __init__(self, root: Path, first: int = 210, last: int = 227) -> None:
        if not (0 <= first <= last <= 232):
            raise ValueError("ROS domain range must be within 0..232")
        self._root = root
        self._first = first
        self._last = last

    def acquire(
        self, *, session_id: str, preferred: int | None = None
    ) -> RosDomainLease:
        if preferred is not None and not 0 <= preferred <= 232:
            raise ValueError("preferred ROS_DOMAIN_ID must be within 0..232")
        self._root.mkdir(parents=True, exist_ok=True)
        candidates = (
            (preferred,) if preferred is not None else range(self._first, self._last + 1)
        )
        for domain_id in candidates:
            path = self._root / f"ros-domain-{domain_id}.lock"
            descriptor = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
            try:
                fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
            except BlockingIOError:
                os.close(descriptor)
                continue
            metadata = json.dumps(
                {"pid": os.getpid(), "session_id": session_id},
                ensure_ascii=False,
            ).encode("utf-8")
            os.ftruncate(descriptor, 0)
            os.write(descriptor, metadata)
            os.fsync(descriptor)
            return RosDomainLease(domain_id, path, descriptor)
        requested = str(preferred) if preferred is not None else f"{self._first}..{self._last}"
        raise DomainLeaseUnavailable(
            f"no free ROS domain lease in {requested}; wait for another acceptance "
            "session or unset ROS_DOMAIN_ID"
        )


@dataclass(slots=True)
class ArtifactDirectoryLease:
    target: Path
    path: Path
    _descriptor: int
    _released: bool = False

    def release(self) -> None:
        if self._released:
            return
        fcntl.flock(self._descriptor, fcntl.LOCK_UN)
        os.close(self._descriptor)
        self._released = True


class ArtifactDirectoryLeasePool:
    """串行化同一证据目录，防止并发会话互删 map/report。"""

    def __init__(self, root: Path) -> None:
        self._root = root

    def acquire(self, *, target: Path, session_id: str) -> ArtifactDirectoryLease:
        resolved = target.resolve()
        digest = hashlib.sha256(os.fsencode(str(resolved))).hexdigest()[:24]
        self._root.mkdir(parents=True, exist_ok=True)
        path = self._root / f"artifact-{digest}.lock"
        descriptor = os.open(path, os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            os.close(descriptor)
            raise ArtifactLeaseUnavailable(
                f"acceptance artifact directory is already in use: {resolved}"
            ) from error
        metadata = json.dumps(
            {
                "pid": os.getpid(),
                "session_id": session_id,
                "artifact_dir": str(resolved),
            },
            ensure_ascii=False,
        ).encode("utf-8")
        os.ftruncate(descriptor, 0)
        os.write(descriptor, metadata)
        os.fsync(descriptor)
        return ArtifactDirectoryLease(resolved, path, descriptor)


@dataclass(frozen=True, slots=True)
class AcceptanceSessionConfig:
    name: str
    workspace: Path
    artifact_root: Path
    timeout_s: float
    session_id: str | None = None
    artifact_dir: Path | None = None
    domain_first: int = 210
    domain_last: int = 227
    preferred_domain: int | None = None
    inherit_ros_domain_id: bool = True
    termination_grace_s: float = 8.0
    failure_log_lines: int = 120
    environment: Mapping[str, str] = field(default_factory=dict)
    lock_root: Path = Path("/tmp/embodied-agent-acceptance")


@dataclass(slots=True)
class _ChildRecord:
    role: str
    argv: tuple[str, ...]
    log_path: Path
    handle: Any
    stop_result: StopResult | None = None


@dataclass(slots=True)
class _RunRecord:
    argv: tuple[str, ...]
    timeout_s: float
    returncode: int | None = None
    timed_out: bool = False
    error: str | None = None


def _safe_session_id(value: str) -> str:
    rendered = re.sub(r"[^A-Za-z0-9_.-]+", "-", value).strip("-.")
    if not rendered:
        raise ValueError("acceptance session_id is empty after normalization")
    return rendered[:96]


class AcceptanceSession:
    """一次验收的深 Module：调用者只声明进程，不重复实现生命周期。"""

    def __init__(
        self,
        config: AcceptanceSessionConfig,
        *,
        process_adapter: ProcessAdapter | None = None,
        base_environment: Mapping[str, str] | None = None,
    ) -> None:
        if config.timeout_s <= 0:
            raise ValueError("acceptance timeout_s must be positive")
        self.config = config
        self._adapter = process_adapter or SubprocessProcessAdapter()
        self._base_environment = dict(
            os.environ if base_environment is None else base_environment
        )
        generated = (
            f"{time.strftime('%Y%m%dT%H%M%SZ', time.gmtime())}-"
            f"{os.getpid()}-{uuid.uuid4().hex[:8]}"
        )
        self.session_id = _safe_session_id(config.session_id or generated)
        self.artifact_dir = (
            config.artifact_dir.resolve()
            if config.artifact_dir is not None
            else (config.artifact_root / self.session_id).resolve()
        )
        self.manifest_path = self.artifact_dir / "acceptance_session.json"
        self.environment: dict[str, str] = {}
        self._lease: RosDomainLease | None = None
        self._artifact_lease: ArtifactDirectoryLease | None = None
        self._children: list[_ChildRecord] = []
        self._runs: list[_RunRecord] = []
        self._process_session_started = False
        self._session_cleanup: SessionCleanupResult | None = None
        self._previous_signal_handlers: dict[int, Any] = {}
        self._started_monotonic = 0.0
        self._started_ns = 0
        self._closed = False
        self._outcome = "running"
        self._error: str | None = None

    @property
    def started_ns(self) -> int:
        return self._started_ns

    @property
    def domain_id(self) -> int:
        if self._lease is None:
            raise AcceptanceSessionError("acceptance session has not been opened")
        return self._lease.domain_id

    def __enter__(self) -> "AcceptanceSession":
        artifact_pool = ArtifactDirectoryLeasePool(
            self.config.lock_root / "artifacts"
        )
        self._artifact_lease = artifact_pool.acquire(
            target=self.artifact_dir, session_id=self.session_id
        )
        try:
            self.artifact_dir.mkdir(parents=True, exist_ok=True)
            preferred = self.config.preferred_domain
            if (
                preferred is None
                and self.config.inherit_ros_domain_id
                and self._base_environment.get("ROS_DOMAIN_ID")
            ):
                preferred = int(self._base_environment["ROS_DOMAIN_ID"])
            pool = RosDomainLeasePool(
                self.config.lock_root / "domains",
                self.config.domain_first,
                self.config.domain_last,
            )
            self._lease = pool.acquire(
                session_id=self.session_id, preferred=preferred
            )
            # 尽早把 TERM/HUP 转成异常，确保后续任何 setup 失败都经过统一 finally。
            self._install_signal_handlers()
            self._started_monotonic = time.monotonic()
            self._started_ns = time.time_ns()
            self.environment = self._base_environment.copy()
            self.environment.update(self.config.environment)
            self.environment.update(
                {
                    "WORKSPACE": str(self.config.workspace.resolve()),
                    "ROS_DOMAIN_ID": str(self.domain_id),
                    "ACCEPTANCE_SESSION_ID": self.session_id,
                    "ACCEPTANCE_SESSION_DIR": str(self.artifact_dir),
                }
            )
            partition = self.config.environment.get(
                "GZ_PARTITION",
                f"embodied_agent_{self.domain_id}_{self.session_id}",
            )
            self.environment["GZ_PARTITION"] = partition
            self.environment["IGN_PARTITION"] = partition
            # WSL 下禁用 FastDDS SHM，避免遗留 fastrtps_port 锁让验收假性卡住。
            if self.environment.get("EMBODIED_ALLOW_FASTDDS_SHM", "false") != "true":
                self.environment["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
            # Linux subreaper 必须早于任何 spawn 启用，父编排器先崩溃时才能接管
            # 已 setsid 的 Gazebo/Nav2 孙进程，并在会话结束统一回收。
            self._adapter.begin_session()
            self._process_session_started = True
            self._write_manifest()
        except BaseException:
            self._restore_signal_handlers()
            if self._process_session_started:
                try:
                    self._adapter.finish_session(
                        grace_s=self.config.termination_grace_s
                    )
                except BaseException:
                    # 尚未 spawn 任何进程；保留原始 setup 异常，并优先释放两类 lease。
                    pass
                finally:
                    self._process_session_started = False
            if self._lease is not None:
                self._lease.release()
                self._lease = None
            self._artifact_lease.release()
            self._artifact_lease = None
            raise
        print(
            f"[acceptance-session] id={self.session_id} "
            f"domain={self.domain_id} partition={partition}"
        )
        return self

    def _install_signal_handlers(self) -> None:
        if threading.current_thread() is not threading.main_thread():
            return
        for signal_number in (signal.SIGTERM, signal.SIGHUP):
            self._previous_signal_handlers[signal_number] = signal.getsignal(
                signal_number
            )
            signal.signal(signal_number, self._raise_signal)

    @staticmethod
    def _raise_signal(signal_number: int, _frame) -> None:
        # signal handler 只改变控制流；真正的 TERM→KILL 清理由 __exit__ 统一执行。
        raise AcceptanceSignalError(signal_number)

    def _ignore_managed_signals_during_cleanup(self) -> None:
        for signal_number in self._previous_signal_handlers:
            signal.signal(signal_number, signal.SIG_IGN)

    def _restore_signal_handlers(self) -> None:
        for signal_number, previous in self._previous_signal_handlers.items():
            signal.signal(signal_number, previous)
        self._previous_signal_handlers.clear()

    def remaining_s(self) -> float:
        return max(
            0.0,
            self.config.timeout_s - (time.monotonic() - self._started_monotonic),
        )

    def log_path(self, role: str) -> Path:
        safe_role = _safe_session_id(role)
        return self.artifact_dir / f"{safe_role}.log"

    def spawn(
        self,
        role: str,
        argv: Sequence[str],
        *,
        log_path: Path | None = None,
    ) -> Any:
        if self._lease is None or self._closed:
            raise AcceptanceSessionError("acceptance session is not active")
        resolved_log = (log_path or self.log_path(role)).resolve()
        handle = self._adapter.spawn(
            argv,
            cwd=self.config.workspace,
            env=self.environment,
            log_path=resolved_log,
        )
        self._children.append(
            _ChildRecord(role, tuple(argv), resolved_log, handle)
        )
        self._write_manifest()
        return handle

    def run(
        self,
        argv: Sequence[str],
        *,
        timeout_s: float | None = None,
        check: bool = True,
    ) -> int:
        if self._lease is None or self._closed:
            raise AcceptanceSessionError("acceptance session is not active")
        remaining = self.remaining_s()
        requested = remaining if timeout_s is None else min(timeout_s, remaining)
        if requested <= 0:
            raise AcceptanceCommandError(
                f"acceptance session {self.session_id} exceeded its {self.config.timeout_s}s deadline"
            )
        record = _RunRecord(tuple(argv), requested)
        self._runs.append(record)
        try:
            result = self._adapter.run(
                argv,
                cwd=self.config.workspace,
                env=self.environment,
                timeout_s=requested,
            )
        except BaseException as error:
            record.error = str(error).strip() or type(error).__name__
            self._write_manifest()
            raise
        record.returncode = result.returncode
        record.timed_out = result.timed_out
        command = " ".join(argv)
        if result.timed_out:
            record.error = f"command timed out after {requested:.1f}s: {command}"
        elif check and result.returncode != 0:
            record.error = (
                f"command failed with exit {result.returncode}: {command}"
            )
        self._write_manifest()
        if result.timed_out:
            raise AcceptanceCommandError(record.error)
        if check and result.returncode != 0:
            raise AcceptanceCommandError(record.error)
        return result.returncode

    def _tail_failure_logs(self) -> None:
        for child in self._children:
            if not child.log_path.is_file():
                continue
            lines = child.log_path.read_text(
                encoding="utf-8", errors="replace"
            ).splitlines()
            tail = lines[-self.config.failure_log_lines :]
            print(
                f"---- {child.role} log (last {len(tail)} lines) ----",
                file=sys.stderr,
            )
            print(f"Full runtime log: {child.log_path}", file=sys.stderr)
            if tail:
                print("\n".join(tail), file=sys.stderr)

    def _write_manifest(self) -> None:
        self.artifact_dir.mkdir(parents=True, exist_ok=True)
        payload = {
            "schema_version": 1,
            "name": self.config.name,
            "session_id": self.session_id,
            "outcome": self._outcome,
            "error": self._error,
            "started_ns": self._started_ns,
            "finished_ns": time.time_ns() if self._closed else None,
            "domain_id": self._lease.domain_id if self._lease is not None else None,
            "gz_partition": self.environment.get("GZ_PARTITION"),
            "cleanup_complete": self._closed
            and self._session_cleanup is not None
            and not self._session_cleanup.remaining_pids
            and all(
                child.stop_result is not None
                and child.stop_result.returncode is not None
                for child in self._children
            ),
            "processes": [
                {
                    "role": child.role,
                    "argv": list(child.argv),
                    "log_path": str(child.log_path),
                    "stop": (
                        {
                            "returncode": child.stop_result.returncode,
                            "graceful": child.stop_result.graceful,
                            "killed": child.stop_result.killed,
                        }
                        if child.stop_result is not None
                        else None
                    ),
                }
                for child in self._children
            ],
            "orphan_cleanup": (
                {
                    "observed_pids": list(self._session_cleanup.observed_pids),
                    "killed": self._session_cleanup.killed,
                    "remaining_pids": list(
                        self._session_cleanup.remaining_pids
                    ),
                }
                if self._session_cleanup is not None
                else None
            ),
            "commands": [
                {
                    "argv": list(command.argv),
                    "timeout_s": command.timeout_s,
                    "returncode": command.returncode,
                    "timed_out": command.timed_out,
                    "error": command.error,
                }
                for command in self._runs
            ],
        }
        temporary = self.manifest_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        os.replace(temporary, self.manifest_path)

    def close(self, *, error: BaseException | None = None) -> None:
        if self._closed:
            return
        self._ignore_managed_signals_during_cleanup()
        cleanup_errors: list[str] = []
        for child in reversed(self._children):
            try:
                child.stop_result = self._adapter.stop(
                    child.handle, grace_s=self.config.termination_grace_s
                )
                if child.stop_result.returncode is None:
                    cleanup_errors.append(
                        f"{child.role}: process could not be reaped"
                    )
            except BaseException as stop_error:  # pragma: no cover - OS failure
                cleanup_errors.append(f"{child.role}: {stop_error}")
        if self._process_session_started:
            try:
                self._session_cleanup = self._adapter.finish_session(
                    grace_s=self.config.termination_grace_s
                )
                if self._session_cleanup.remaining_pids:
                    cleanup_errors.append(
                        "adopted processes still alive: "
                        + ",".join(
                            str(value)
                            for value in self._session_cleanup.remaining_pids
                        )
                    )
            except BaseException as cleanup_error:  # pragma: no cover - OS failure
                cleanup_errors.append(f"orphan cleanup: {cleanup_error}")
            finally:
                self._process_session_started = False
        self._error = (
            (str(error).strip() or type(error).__name__)
            if error is not None
            else None
        )
        if cleanup_errors:
            joined = "; ".join(cleanup_errors)
            self._error = f"{self._error}; cleanup: {joined}" if self._error else joined
        self._outcome = "failed" if error is not None or cleanup_errors else "passed"
        self._closed = True
        try:
            self._write_manifest()
        finally:
            # 即便磁盘已满导致 manifest 写入失败，也不能把 domain 永久锁住。
            if self._lease is not None:
                self._lease.release()
            if self._artifact_lease is not None:
                self._artifact_lease.release()
            self._restore_signal_handlers()
        if self._error:
            self._tail_failure_logs()
        if cleanup_errors and error is None:
            raise AcceptanceSessionError(f"acceptance cleanup failed: {joined}")

    def __exit__(self, exc_type, exc, traceback) -> bool:
        self.close(error=exc)
        return False
