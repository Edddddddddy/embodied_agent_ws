"""重型验收的有界资源采样与低内存保护。

采样以 JSONL 流式落盘，内存中只保留 latest/peak；因此监控器本身不会随
十几分钟的 SLAM 会话增长。所有实现只依赖 ``/proc``，CI 与 WSL 都无需
额外安装 psutil。
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
from pathlib import Path
import threading
import time
from typing import Callable

from tools.acceptance.visual_runtime import (
    LinuxMemorySnapshot,
    read_linux_memory,
)


@dataclass(frozen=True, slots=True)
class ResourcePressurePolicy:
    """连续多次越界才失败，过滤瞬时 map-save/渲染峰值。"""

    minimum_available_kib: int = 768 * 1024
    maximum_swap_used_ratio: float = 0.90
    consecutive_samples: int = 3

    def __post_init__(self) -> None:
        if self.minimum_available_kib <= 0:
            raise ValueError("minimum available memory must be positive")
        if not 0.0 < self.maximum_swap_used_ratio <= 1.0:
            raise ValueError("maximum swap used ratio must be in (0, 1]")
        if self.consecutive_samples <= 0:
            raise ValueError("consecutive samples must be positive")


@dataclass(frozen=True, slots=True)
class ResourceWatchdogConfig:
    interval_s: float = 5.0
    policy: ResourcePressurePolicy = field(
        default_factory=ResourcePressurePolicy
    )

    def __post_init__(self) -> None:
        if self.interval_s <= 0.0:
            raise ValueError("resource watchdog interval must be positive")


class ResourcePressureTracker:
    """把单点指标转换成带滞回的确定性失败信号。"""

    def __init__(self, policy: ResourcePressurePolicy) -> None:
        self.policy = policy
        self._consecutive = 0

    def observe(self, memory: LinuxMemorySnapshot) -> str | None:
        low_available = (
            memory.available_kib < self.policy.minimum_available_kib
        )
        # Swap 很高但内存已经回收时不应误杀；只有与低可用内存共同出现才
        # 表示正在进入 WSL thrash。
        swap_pressure = (
            memory.swap_total_kib > 0
            and memory.swap_used_ratio
            >= self.policy.maximum_swap_used_ratio
            and memory.available_kib
            < self.policy.minimum_available_kib * 2
        )
        if low_available or swap_pressure:
            self._consecutive += 1
        else:
            self._consecutive = 0
        if self._consecutive < self.policy.consecutive_samples:
            return None
        return (
            "resource_exhaustion: "
            f"available_mib={memory.available_kib / 1024:.1f} "
            f"swap_used_ratio={memory.swap_used_ratio:.3f} "
            f"consecutive_samples={self._consecutive}"
        )


@dataclass(frozen=True, slots=True)
class ResourceWatchdogSummary:
    sample_count: int
    latest_memory: LinuxMemorySnapshot | None
    peak_session_rss_kib: int
    latest_session_process_count: int
    failure: str | None


def session_process_memory(
    session_id: str,
    *,
    proc_root: Path = Path("/proc"),
) -> tuple[int, int]:
    """按会话环境变量聚合进程 RSS，跨越嵌套 ``setsid`` 进程组。"""

    marker = f"ACCEPTANCE_SESSION_ID={session_id}".encode("utf-8")
    count = 0
    rss_kib = 0
    for process_dir in proc_root.iterdir():
        if not process_dir.name.isdigit():
            continue
        try:
            environment = (process_dir / "environ").read_bytes().split(b"\0")
            if marker not in environment:
                continue
            status = (process_dir / "status").read_text(
                encoding="utf-8", errors="replace"
            )
        except (
            FileNotFoundError,
            PermissionError,
            ProcessLookupError,
        ):
            continue
        count += 1
        for line in status.splitlines():
            if line.startswith("VmRSS:"):
                fields = line.split()
                if len(fields) >= 2:
                    rss_kib += int(fields[1])
                break
    return count, rss_kib


class ResourceWatchdog:
    """周期采样系统资源，并在持续低内存时调用一次失败回调。"""

    def __init__(
        self,
        *,
        config: ResourceWatchdogConfig,
        session_id: str,
        output_path: Path,
        on_failure: Callable[[str], None],
        memory_sampler: Callable[[], LinuxMemorySnapshot] = read_linux_memory,
        process_sampler: Callable[[str], tuple[int, int]] = (
            session_process_memory
        ),
    ) -> None:
        self.config = config
        self.session_id = session_id
        self.output_path = output_path
        self._on_failure = on_failure
        self._memory_sampler = memory_sampler
        self._process_sampler = process_sampler
        self._tracker = ResourcePressureTracker(config.policy)
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        self._sample_count = 0
        self._latest_memory: LinuxMemorySnapshot | None = None
        self._peak_session_rss_kib = 0
        self._latest_session_process_count = 0
        self._failure: str | None = None

    @property
    def summary(self) -> ResourceWatchdogSummary:
        with self._lock:
            return ResourceWatchdogSummary(
                sample_count=self._sample_count,
                latest_memory=self._latest_memory,
                peak_session_rss_kib=self._peak_session_rss_kib,
                latest_session_process_count=(
                    self._latest_session_process_count
                ),
                failure=self._failure,
            )

    def sample_once(self) -> None:
        """执行一次采样；公开该 seam 便于无 sleep 的确定性单测。"""

        memory = self._memory_sampler()
        process_count, session_rss_kib = self._process_sampler(
            self.session_id
        )
        failure = self._tracker.observe(memory)
        record = {
            "timestamp_ns": time.time_ns(),
            "mem_total_mib": round(memory.total_kib / 1024, 1),
            "mem_available_mib": round(memory.available_kib / 1024, 1),
            "swap_total_mib": round(memory.swap_total_kib / 1024, 1),
            "swap_free_mib": round(memory.swap_free_kib / 1024, 1),
            "swap_used_ratio": round(memory.swap_used_ratio, 4),
            "session_process_count": process_count,
            "session_rss_mib": round(session_rss_kib / 1024, 1),
            "pressure": failure is not None,
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

        notify = False
        with self._lock:
            self._sample_count += 1
            self._latest_memory = memory
            self._latest_session_process_count = process_count
            self._peak_session_rss_kib = max(
                self._peak_session_rss_kib,
                session_rss_kib,
            )
            if failure is not None and self._failure is None:
                self._failure = (
                    f"{failure} session_rss_mib={session_rss_kib / 1024:.1f}"
                )
                notify = True
            reported_failure = self._failure
        if notify and reported_failure is not None:
            self._on_failure(reported_failure)

    def start(self) -> None:
        if self._thread is not None:
            raise RuntimeError("resource watchdog is already started")

        def monitor() -> None:
            # 进入会话后立即建立基线；之后用 Event.wait 使 stop 不必等待完整周期。
            while not self._stop.is_set():
                try:
                    self.sample_once()
                except (OSError, ValueError) as error:
                    # 资源采样失败不能伪装成资源充足；保留诊断记录，但不因
                    # 单次 /proc 竞争终止真实机器人任务。
                    self.output_path.parent.mkdir(parents=True, exist_ok=True)
                    with self.output_path.open("a", encoding="utf-8") as stream:
                        stream.write(
                            json.dumps(
                                {
                                    "timestamp_ns": time.time_ns(),
                                    "sampling_error": str(error),
                                },
                                ensure_ascii=False,
                            )
                            + "\n"
                        )
                if self._stop.wait(self.config.interval_s):
                    break

        self._thread = threading.Thread(
            target=monitor,
            name=f"acceptance-resource-{self.session_id[:24]}",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=max(1.0, self.config.interval_s + 0.5))
        self._thread = None
