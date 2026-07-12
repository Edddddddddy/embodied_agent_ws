"""ASR utterance endpoint 的去重、延迟提交与关闭协调。"""

from __future__ import annotations

import threading
import time
from typing import Callable


class AsrEndpointRuntime:
    """把 VAD endpoint 变成至多一次、可延迟且可安全关闭的 ASR commit。

    online/offline 的 commit 传输方式不同（直接 WebSocket commit 与离线队列事件），
    但端点去重、busy gate 和 timer 生命周期完全相同，因此由此模块统一持有。
    """

    def __init__(
        self,
        *,
        delay_ms: int,
        blocked: Callable[[], bool],
        commit: Callable[[], None],
        on_endpoint: Callable[[str, int], None],
        on_commit: Callable[[str], None],
        on_duplicate: Callable[[str], None] | None = None,
        on_error: Callable[[Exception], None] | None = None,
        duplicate_window_s: float = 0.05,
        clock: Callable[[], float] = time.monotonic,
        timer_factory=threading.Timer,
    ):
        self._delay_ms = max(0, int(delay_ms))
        self._blocked = blocked
        self._commit = commit
        self._on_endpoint = on_endpoint
        self._on_commit = on_commit
        self._on_duplicate = on_duplicate or (lambda _source: None)
        self._on_error = on_error or (lambda _error: None)
        self._duplicate_window_s = max(0.0, float(duplicate_window_s))
        self._clock = clock
        self._timer_factory = timer_factory
        self._lock = threading.Lock()
        self._last_request_at = float("-inf")
        self._closed = False
        self._timers: set[object] = set()

    def request(self, source: str) -> bool:
        """接受一次 endpoint；返回 False 表示 busy、重复或已经关闭。"""

        if self._blocked():
            return False
        with self._lock:
            if self._closed:
                return False
            now = self._clock()
            if now - self._last_request_at < self._duplicate_window_s:
                self._on_duplicate(source)
                return False
            self._last_request_at = now

        self._on_endpoint(source, self._delay_ms)
        if self._delay_ms <= 0:
            return self._perform_commit(source)

        timer = self._timer_factory(
            self._delay_ms / 1000.0,
            self._timer_fired,
            args=(source,),
        )
        timer.daemon = True
        with self._lock:
            if self._closed:
                return False
            self._timers.add(timer)
        timer.start()
        return True

    def _timer_fired(self, source: str) -> None:
        try:
            self._perform_commit(source)
        finally:
            current = threading.current_thread()
            with self._lock:
                self._timers.discard(current)

    def _perform_commit(self, source: str) -> bool:
        with self._lock:
            if self._closed:
                return False
        if self._blocked():
            return False
        try:
            self._on_commit(source)
            self._commit()
            return True
        except Exception as exc:
            # timer 线程中的异常不能静默消失，也不能以未捕获 traceback 污染现场日志。
            self._on_error(exc)
            return False

    def close(self) -> None:
        """取消尚未触发的 timer，防止节点销毁后后台线程继续访问 provider。"""

        with self._lock:
            self._closed = True
            timers = tuple(self._timers)
            self._timers.clear()
        for timer in timers:
            timer.cancel()
