import queue
import threading
from dataclasses import dataclass
from typing import Generic, Optional, TypeVar

T = TypeVar("T")


@dataclass(frozen=True)
class BufferStats:
    accepted: int
    dropped: int
    high_watermark: int


class DoubleBuffer(Generic[T]):
    """A bounded two-slot hand-off queue with explicit overflow policy."""

    def __init__(self, *, drop_oldest: bool):
        self._queue: queue.Queue[object] = queue.Queue(maxsize=2)
        self._drop_oldest = drop_oldest
        self._lock = threading.Lock()
        self._accepted = 0
        self._dropped = 0
        self._high_watermark = 0
        self._closed = False
        self._sentinel = object()

    def put(self, item: T, timeout: Optional[float] = None) -> bool:
        with self._lock:
            if self._closed:
                return False
        if self._drop_oldest:
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                try:
                    self._queue.get_nowait()
                    self._queue.task_done()
                except queue.Empty:
                    pass
                with self._lock:
                    self._dropped += 1
                self._queue.put_nowait(item)
        else:
            try:
                self._queue.put(item, timeout=timeout)
            except queue.Full:
                with self._lock:
                    self._dropped += 1
                return False
        with self._lock:
            self._accepted += 1
            self._high_watermark = max(self._high_watermark, self._queue.qsize())
        return True

    def get(self, timeout: Optional[float] = None) -> T:
        item = self._queue.get(timeout=timeout)
        self._queue.task_done()
        if item is self._sentinel:
            raise StopIteration
        return item  # type: ignore[return-value]

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
        if not self._drop_oldest:
            self._queue.put(self._sentinel)
            return
        while True:
            try:
                self._queue.put_nowait(self._sentinel)
                return
            except queue.Full:
                try:
                    self._queue.get_nowait()
                    self._queue.task_done()
                except queue.Empty:
                    return

    def abort(self) -> None:
        """Discard queued work and wake a consumer during error shutdown."""
        with self._lock:
            self._closed = True
        while True:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
            except queue.Empty:
                break
        self._queue.put_nowait(self._sentinel)

    @property
    def stats(self) -> BufferStats:
        with self._lock:
            return BufferStats(self._accepted, self._dropped, self._high_watermark)
