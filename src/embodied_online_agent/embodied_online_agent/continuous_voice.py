import queue
import threading
import time
from dataclasses import dataclass
from typing import Iterable, Optional

from .wakeword import WakeWordGate


DEFAULT_SLEEP_WORDS = ("退出控制", "结束控制", "休眠", "睡眠", "先这样")
DEFAULT_PRIORITY_STOP_WORDS = ("停下", "停止", "急停", "刹车", "别动")


@dataclass(frozen=True)
class SessionDecision:
    """连续语音会话对一句识别结果的判定。

    command 为 None 表示这句话不应进入 LLM/动作链路，例如未唤醒、空唤醒词、
    或“退出控制”。priority_stop=True 表示这句话应该抢占普通队列。
    """

    command: Optional[str]
    accepted: bool
    reason: str
    session_active: bool
    priority_stop: bool = False


@dataclass(frozen=True)
class QueuedCommand:
    text: str
    priority_stop: bool = False
    created_at: float = 0.0
    context: object | None = None


@dataclass(frozen=True)
class QueueSnapshot:
    accepted: bool
    size: int
    dropped: int = 0
    reason: str = ""


class ContinuousVoiceSession:
    """把文本级唤醒词升级为“一次唤醒，多轮控制”的会话状态机。

    第一阶段仍复用现有 ASR 文本唤醒词，避免引入声学 KWS 依赖；后续接
    sherpa-onnx KWS/openWakeWord 时，只需要把 wake event 映射到这里。
    """

    def __init__(
        self,
        wake_gate: WakeWordGate,
        *,
        enabled: bool,
        sleep_words: Iterable[str] = DEFAULT_SLEEP_WORDS,
        priority_stop_words: Iterable[str] = DEFAULT_PRIORITY_STOP_WORDS,
    ):
        self._wake_gate = wake_gate
        self.enabled = enabled
        self._sleep_words = tuple(word for word in sleep_words if word)
        self._priority_stop_words = tuple(word for word in priority_stop_words if word)

    def accept(self, transcript: str) -> SessionDecision:
        text = transcript.strip()
        if not text:
            return SessionDecision(None, False, "empty", self._wake_gate.active)

        if self.enabled and self._contains_any(text, self._sleep_words):
            self._wake_gate.sleep()
            return SessionDecision(None, False, "session_sleep", False)

        command = self._wake_gate.process(text)
        if command is None:
            reason = "session_awake" if self._wake_gate.active else "wake_word_not_detected"
            return SessionDecision(None, False, reason, self._wake_gate.active)

        priority_stop = self._contains_any(command, self._priority_stop_words)
        return SessionDecision(
            command,
            True,
            "accepted",
            self._wake_gate.active,
            priority_stop=priority_stop,
        )

    @staticmethod
    def _contains_any(text: str, words: Iterable[str]) -> bool:
        return any(word in text for word in words)


class ContinuousCommandQueue:
    """串行命令队列。

    普通命令 FIFO 执行；急停/停下是高优先级命令，会清空尚未执行的普通命令。
    已经交给执行器的动作由 Agent 额外发布 stop/cancel 处理。
    """

    def __init__(self, max_size: int = 8, clock=time.monotonic):
        self._queue: queue.Queue[QueuedCommand] = queue.Queue(maxsize=max(1, max_size))
        self._clock = clock
        self._lock = threading.Lock()

    def put(
        self,
        text: str,
        *,
        priority_stop: bool = False,
        context: object | None = None,
    ) -> QueueSnapshot:
        item = QueuedCommand(
            text=text,
            priority_stop=priority_stop,
            created_at=self._clock(),
            context=context,
        )
        with self._lock:
            dropped = self.clear() if priority_stop else 0
            try:
                self._queue.put_nowait(item)
            except queue.Full:
                return QueueSnapshot(False, self._queue.qsize(), dropped, "queue_full")
            return QueueSnapshot(True, self._queue.qsize(), dropped)

    def get(self, timeout: float = 0.1) -> QueuedCommand:
        return self._queue.get(timeout=timeout)

    def task_done(self) -> None:
        self._queue.task_done()

    def clear(self) -> int:
        dropped = 0
        while True:
            try:
                self._queue.get_nowait()
                self._queue.task_done()
                dropped += 1
            except queue.Empty:
                return dropped

    def size(self) -> int:
        return self._queue.qsize()
