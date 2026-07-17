"""Observable progress for long-running acceptance gates.

Heavy ROS/Gazebo gates intentionally redirect verbose node output to an artifact
log.  This reporter keeps the operator terminal alive with concise milestones
and heartbeats without duplicating the full ROS log stream.
"""

from __future__ import annotations

import sys
import threading
import time
from collections.abc import Callable
from typing import TextIO


def _single_line(value: object) -> str:
    """Keep heartbeat output grep-friendly even when a provider returns newlines."""

    return " ".join(str(value).split()) or "-"


class AcceptanceProgress:
    """Thread-safe stage and heartbeat reporter for a single acceptance run."""

    def __init__(
        self,
        *,
        label: str,
        total_stages: int,
        heartbeat_s: float = 15.0,
        stream: TextIO = sys.stdout,
    ) -> None:
        if total_stages < 1:
            raise ValueError("total_stages must be positive")
        if heartbeat_s <= 0:
            raise ValueError("heartbeat_s must be positive")
        self._label = _single_line(label)
        self._total_stages = total_stages
        self._heartbeat_s = heartbeat_s
        self._stream = stream
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None
        self._started_at = 0.0
        self._current_stage = 0
        self._detail_supplier: Callable[[], str] = lambda: "-"

    def _emit(self, message: str) -> None:
        # flush=True 是现场可观测性的关键：管道/PTY 下不能依赖 Python 行缓冲。
        with self._lock:
            print(message, file=self._stream, flush=True)

    def start(
        self,
        *,
        session_id: str,
        timeout_s: float,
        log_path: str,
        detail_supplier: Callable[[], str] | None = None,
    ) -> None:
        if self._thread is not None:
            raise RuntimeError("acceptance progress already started")
        self._started_at = time.monotonic()
        self._detail_supplier = detail_supplier or (lambda: "-")
        self._emit(
            f"[{self._label}] START session={_single_line(session_id)} "
            f"timeout={timeout_s:g}s log={_single_line(log_path)}"
        )
        self._thread = threading.Thread(
            target=self._heartbeat_loop,
            name=f"{self._label}-progress",
            daemon=True,
        )
        self._thread.start()

    def stage(self, index: int, name: str, detail: str = "") -> None:
        if not 1 <= index <= self._total_stages:
            raise ValueError(
                f"stage index {index} outside 1..{self._total_stages}"
            )
        # ROS transient-local 状态可能重复投递；只公布向前推进的里程碑。
        with self._lock:
            if index <= self._current_stage:
                return
            self._current_stage = index
            suffix = f" - {_single_line(detail)}" if detail else ""
            print(
                f"[{self._label}][{index}/{self._total_stages}] "
                f"{_single_line(name)}{suffix}",
                file=self._stream,
                flush=True,
            )

    def _heartbeat_loop(self) -> None:
        while not self._stop_event.wait(self._heartbeat_s):
            elapsed = int(time.monotonic() - self._started_at)
            try:
                detail = _single_line(self._detail_supplier())
            except Exception as error:  # pragma: no cover - defensive telemetry
                detail = f"detail_unavailable={type(error).__name__}"
            self._emit(
                f"[{self._label}] RUNNING elapsed={elapsed}s "
                f"stage={self._current_stage}/{self._total_stages} {detail}"
            )

    def stop(self, outcome: str | None = None) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=max(1.0, self._heartbeat_s * 2.0))
        if outcome:
            elapsed = int(time.monotonic() - self._started_at)
            self._emit(
                f"[{self._label}] {_single_line(outcome)} elapsed={elapsed}s"
            )
