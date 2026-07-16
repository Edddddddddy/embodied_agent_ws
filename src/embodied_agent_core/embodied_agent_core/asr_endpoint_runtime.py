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
        duplicate_window_s: float = 0.25,
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
        self._last_request_source: str | None = None
        self._closed = False
        self._generation = 0
        self._timers: set[object] = set()

    def request(self, source: str) -> bool:
        """接受一次 endpoint；返回 False 表示 busy、重复或已经关闭。"""

        if self._blocked():
            return False
        with self._lock:
            if self._closed:
                return False
            now = self._clock()
            # WebRTC 兼容层会为同一端点依次发布 speech_ended 与
            # silence_timeout；只压掉这种跨 topic 镜像。相同 source 的下一条
            # endpoint 代表新 utterance，即使用户说得很快也不能吞掉。
            if (
                source != self._last_request_source
                and now - self._last_request_at < self._duplicate_window_s
            ):
                self._on_duplicate(source)
                return False
            self._last_request_at = now
            self._last_request_source = source
            generation = self._generation

        self._on_endpoint(source, self._delay_ms)
        if self._delay_ms <= 0:
            return self._perform_commit(source, generation)

        timer = self._timer_factory(
            self._delay_ms / 1000.0,
            self._timer_fired,
            args=(source, generation),
        )
        timer.daemon = True
        with self._lock:
            if self._closed:
                return False
            self._timers.add(timer)
        timer.start()
        return True

    def _timer_fired(self, source: str, generation: int) -> None:
        try:
            self._perform_commit(source, generation)
        finally:
            current = threading.current_thread()
            with self._lock:
                self._timers.discard(current)

    def _perform_commit(self, source: str, generation: int) -> bool:
        with self._lock:
            if self._closed or generation != self._generation:
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
            self._generation += 1
            timers = tuple(self._timers)
            self._timers.clear()
        for timer in timers:
            timer.cancel()

    def cancel_pending(self) -> None:
        """Lifecycle 停用时取消本代 endpoint，但允许后续重新 activate。"""

        with self._lock:
            if self._closed:
                return
            self._generation += 1
            self._last_request_at = float("-inf")
            self._last_request_source = None
            timers = tuple(self._timers)
            self._timers.clear()
        for timer in timers:
            timer.cancel()

    def resume_utterance(self) -> None:
        """VAD 在提交延迟内恢复说话时，把两段继续视为同一 utterance。

        本方法只取消 endpoint timer，不触碰 ASR provider 的音频流；因此前半句
        已经送入 provider 的 PCM 会保留，下一次 speech_ended 再一次性提交完整句。
        """

        self.cancel_pending()
