"""重型验收共用的稳定会话 Interface。

场景只声明 spawn/run；lease、环境、进程监管、证据和失败清理由本 facade 编排。
"""

from __future__ import annotations

import json
import os
import re
import signal
import sys
import threading
import time
import uuid
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Mapping, Sequence

from tools.acceptance.errors import (
    AcceptanceCommandError,
    AcceptanceResourceExhaustion,
    AcceptanceSessionError,
    AcceptanceSignalError,
    ArtifactLeaseUnavailable,
    DomainLeaseUnavailable,
)
from tools.acceptance.leases import (
    ArtifactDirectoryLease,
    ArtifactDirectoryLeasePool,
    RosDomainLease,
    RosDomainLeasePool,
)
from tools.acceptance.process_supervisor import (
    ProcessAdapter,
    RunResult,
    SessionCleanupResult,
    StopResult,
    SubprocessProcessAdapter,
)
from tools.acceptance.resource_watchdog import (
    ResourceWatchdog,
    ResourceWatchdogConfig,
)


__all__ = [
    "AcceptanceCommandError",
    "AcceptanceResourceExhaustion",
    "AcceptanceSession",
    "AcceptanceSessionConfig",
    "AcceptanceSessionError",
    "AcceptanceSignalError",
    "ArtifactDirectoryLease",
    "ArtifactDirectoryLeasePool",
    "ArtifactLeaseUnavailable",
    "DomainLeaseUnavailable",
    "ProcessAdapter",
    "RosDomainLease",
    "RosDomainLeasePool",
    "RosEnvironmentIsolation",
    "RunResult",
    "SessionCleanupResult",
    "StopResult",
    "SubprocessProcessAdapter",
]


@dataclass(frozen=True, slots=True)
class RosEnvironmentIsolation:
    """重型公开验收可复现的 ROS overlay 来源策略。"""

    system_prefix: Path = Path("/opt/ros/jazzy")
    allowed_external_packages: tuple[str, ...] = (
        "explore_lite",
        "explore_lite_msgs",
    )
    required_packages: tuple[str, ...] = (
        "nav2_bringup",
        "nav2_lifecycle_manager",
        "nav2_util",
    )
    audited_packages: tuple[str, ...] = (
        "explore_lite",
        "explore_lite_msgs",
        "nav2_bringup",
        "nav2_lifecycle_manager",
        "nav2_util",
    )


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
    unset_environment_keys: tuple[str, ...] = ()
    manifest_environment_keys: tuple[str, ...] = ()
    ros_environment_isolation: RosEnvironmentIsolation | None = None
    resource_watchdog: ResourceWatchdogConfig | None = None
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


_ROS_PATH_ENVIRONMENT_KEYS = (
    "AMENT_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "COLCON_PREFIX_PATH",
    "LD_LIBRARY_PATH",
    "PATH",
    "PKG_CONFIG_PATH",
    "PYTHONPATH",
)


def _path_entries(value: str | None) -> list[str]:
    return [entry for entry in str(value or "").split(os.pathsep) if entry]


def _normalized_path(value: str | Path) -> Path:
    return Path(value).expanduser().resolve(strict=False)


def _path_is_within(path: Path, root: Path) -> bool:
    return path == root or root in path.parents


def _packages_in_prefix(prefix: Path) -> frozenset[str]:
    package_index = (
        prefix / "share/ament_index/resource_index/packages"
    )
    try:
        return frozenset(entry.name for entry in package_index.iterdir())
    except (FileNotFoundError, NotADirectoryError, PermissionError):
        return frozenset()


def _external_install_root(prefix: Path) -> Path:
    """从 isolated/merged colcon prefix 推导用于剔除残余路径的 install 根。"""

    if prefix.name == "install":
        return prefix
    if prefix.parent.name == "install":
        return prefix.parent
    return prefix


def _isolate_ros_environment(
    environment: Mapping[str, str],
    *,
    workspace: Path,
    policy: RosEnvironmentIsolation,
) -> tuple[dict[str, str], dict[str, Any]]:
    """移除宿主 Nav2 overlay，同时保留系统、当前工作区和 frontier。"""

    isolated = dict(environment)
    workspace_install = _normalized_path(workspace / "install")
    system_prefix = _normalized_path(policy.system_prefix)
    trusted_roots = (workspace_install, system_prefix)
    allowed_package_names = frozenset(policy.allowed_external_packages)
    original_ament = _path_entries(isolated.get("AMENT_PREFIX_PATH"))
    retained_ament: list[str] = []
    allowed_external_prefixes: list[Path] = []
    allowed_external_build_roots: list[Path] = []
    removed_prefixes: list[str] = []
    excluded_overlay_roots: set[Path] = set()

    for rendered in original_ament:
        prefix = _normalized_path(rendered)
        if any(_path_is_within(prefix, root) for root in trusted_roots):
            retained_ament.append(rendered)
            continue
        packages = _packages_in_prefix(prefix)
        # frontier 可以位于单独的源码工作区；只有该 prefix 不同时携带其它
        # ROS 包时才放行，避免 merged install 借白名单重新带入 Nav2。
        if packages and packages.issubset(allowed_package_names):
            retained_ament.append(rendered)
            allowed_external_prefixes.append(prefix)
            install_root = _external_install_root(prefix)
            if install_root.name == "install":
                allowed_external_build_roots.extend(
                    install_root.parent / "build" / package
                    for package in packages
                )
            continue
        if packages & allowed_package_names:
            unsafe = ", ".join(sorted(packages - allowed_package_names))
            raise AcceptanceSessionError(
                "external frontier prefix also provides non-allowlisted "
                f"packages: {rendered} ({unsafe})"
            )
        removed_prefixes.append(rendered)
        install_root = _external_install_root(prefix)
        excluded_overlay_roots.add(install_root)
        if install_root.name == "install":
            # ament 索引只能定位 install；ament_python 的 editable/symlink
            # 产物还可能把同一外部工作区的 build/* 写进 PYTHONPATH。两者必须
            # 作为一个 overlay 一起剔除，否则 C++ ABI 已隔离、Python 仍会串包。
            excluded_overlay_roots.add(install_root.parent / "build")

    isolated["AMENT_PREFIX_PATH"] = os.pathsep.join(retained_ament)

    def keep_path(rendered: str) -> bool:
        candidate = _normalized_path(rendered)
        if any(_path_is_within(candidate, root) for root in trusted_roots):
            return True
        if any(
            _path_is_within(candidate, prefix)
            for prefix in allowed_external_prefixes
        ):
            return True
        if any(
            _path_is_within(candidate, root)
            for root in allowed_external_build_roots
        ):
            return True
        return not any(
            _path_is_within(candidate, root) or candidate == root
            for root in excluded_overlay_roots
        )

    for key in _ROS_PATH_ENVIRONMENT_KEYS:
        if key == "AMENT_PREFIX_PATH" or key not in isolated:
            continue
        isolated[key] = os.pathsep.join(
            entry
            for entry in _path_entries(isolated[key])
            if keep_path(entry)
        )
    for key in ("AMENT_CURRENT_PREFIX", "COLCON_CURRENT_PREFIX"):
        value = isolated.get(key)
        if value and not keep_path(value):
            isolated.pop(key, None)

    retained_prefixes = [
        (rendered, _normalized_path(rendered))
        for rendered in _path_entries(isolated.get("AMENT_PREFIX_PATH"))
    ]
    package_prefixes: dict[str, str | None] = {}
    for package in policy.audited_packages:
        marker = Path("share/ament_index/resource_index/packages") / package
        package_prefixes[package] = next(
            (
                rendered
                for rendered, prefix in retained_prefixes
                if (prefix / marker).is_file()
            ),
            None,
        )
    missing = [
        package
        for package in policy.required_packages
        if package_prefixes.get(package) is None
    ]
    if missing:
        raise AcceptanceSessionError(
            "isolated ROS environment is missing required packages: "
            + ", ".join(missing)
        )

    # 只落路径与包来源，不把可能含凭据的完整宿主环境写入 manifest。
    provenance = {
        "isolated": True,
        "system_prefix": str(system_prefix),
        "workspace_install_prefix": str(workspace_install),
        "allowed_external_packages": sorted(allowed_package_names),
        "removed_prefixes": removed_prefixes,
        "paths": {
            key: _path_entries(isolated.get(key))
            for key in _ROS_PATH_ENVIRONMENT_KEYS
        },
        "package_prefixes": package_prefixes,
    }
    return isolated, provenance


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
        self._ros_environment_provenance: dict[str, Any] | None = None
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
        self._closing = False
        self._outcome = "running"
        self._error: str | None = None
        self._resource_watchdog: ResourceWatchdog | None = None
        self._resource_failure: str | None = None

    @property
    def started_ns(self) -> int:
        return self._started_ns

    @property
    def domain_id(self) -> int:
        if self._lease is None:
            raise AcceptanceSessionError("acceptance session has not been opened")
        return self._lease.domain_id

    def __enter__(self) -> "AcceptanceSession":
        if (
            self.config.resource_watchdog is not None
            and threading.current_thread() is not threading.main_thread()
        ):
            # watchdog 通过 SIGTERM 把后台失败交给 Python 主线程。非主线程
            # 无法安装 signal handler，继续启动会让默认 SIGTERM 直接杀进程。
            raise AcceptanceSessionError(
                "resource watchdog requires AcceptanceSession on main thread"
            )
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
            # 场景可能继承用户上一次演示的坐标、地图或地点表。必须先删除再覆盖，
            # 否则“空字符串默认值”也可能被下游 `${VAR:-default}` 重新解释为旧策略。
            for key in self.config.unset_environment_keys:
                self.environment.pop(key, None)
            self.environment.update(self.config.environment)
            if self.config.ros_environment_isolation is not None:
                self.environment, self._ros_environment_provenance = (
                    _isolate_ros_environment(
                        self.environment,
                        workspace=self.config.workspace,
                        policy=self.config.ros_environment_isolation,
                    )
                )
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
            if self.config.resource_watchdog is not None:
                self._resource_watchdog = ResourceWatchdog(
                    config=self.config.resource_watchdog,
                    session_id=self.session_id,
                    output_path=self.artifact_dir
                    / "resource_samples.jsonl",
                    on_failure=self._on_resource_failure,
                )
                self._resource_watchdog.start()
        except BaseException:
            if self._resource_watchdog is not None:
                self._resource_watchdog.stop()
                self._resource_watchdog = None
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

    def _raise_signal(self, signal_number: int, _frame) -> None:
        # signal handler 只改变控制流；真正的 TERM→KILL 清理由 __exit__ 统一执行。
        if (
            signal_number == signal.SIGTERM
            and self._resource_failure is not None
        ):
            raise AcceptanceResourceExhaustion(self._resource_failure)
        raise AcceptanceSignalError(signal_number)

    def _on_resource_failure(self, detail: str) -> None:
        """从 watchdog 线程把失败交还主线程，复用统一停车/清理事务。"""

        self._resource_failure = detail
        if not self._closed and not self._closing:
            os.kill(os.getpid(), signal.SIGTERM)

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
            # 只记录场景显式声明的白名单；不能为了可复现性把 API key 等整份
            # 宿主环境写入证据目录。被清除的 key 以 null 留下可审计痕迹。
            "environment": {
                key: self.environment.get(key)
                for key in sorted(
                    set(self.config.manifest_environment_keys)
                    | set(self.config.unset_environment_keys)
                )
            },
            "ros_environment": self._ros_environment_provenance,
            "resources": self._resource_manifest(),
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

    def _resource_manifest(self) -> dict[str, Any] | None:
        watchdog = self._resource_watchdog
        if watchdog is None:
            return None
        summary = watchdog.summary
        latest = summary.latest_memory
        return {
            "samples_path": str(watchdog.output_path),
            "sample_count": summary.sample_count,
            "latest": (
                {
                    "mem_total_mib": round(latest.total_kib / 1024, 1),
                    "mem_available_mib": round(
                        latest.available_kib / 1024, 1
                    ),
                    "swap_total_mib": round(
                        latest.swap_total_kib / 1024, 1
                    ),
                    "swap_free_mib": round(
                        latest.swap_free_kib / 1024, 1
                    ),
                    "swap_used_ratio": round(
                        latest.swap_used_ratio, 4
                    ),
                }
                if latest is not None
                else None
            ),
            "peak_session_rss_mib": round(
                summary.peak_session_rss_kib / 1024, 1
            ),
            "latest_session_process_count": (
                summary.latest_session_process_count
            ),
            "failure": summary.failure,
        }

    def close(self, *, error: BaseException | None = None) -> None:
        if self._closed:
            return
        self._closing = True
        # 先阻止 watchdog 的迟到 SIGTERM，再 join 后台线程；否则信号可能在
        # close() 等待线程时打断清理，反而留下 Gazebo/Nav2 进程和 domain lease。
        self._ignore_managed_signals_during_cleanup()
        late_resource_error: AcceptanceResourceExhaustion | None = None
        if self._resource_watchdog is not None:
            self._resource_watchdog.stop()
            resource_failure = self._resource_watchdog.summary.failure
            if resource_failure is not None and error is None:
                late_resource_error = AcceptanceResourceExhaustion(
                    resource_failure
                )
                error = late_resource_error
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
        if late_resource_error is not None:
            raise late_resource_error

    def __exit__(self, exc_type, exc, traceback) -> bool:
        self.close(error=exc)
        return False
