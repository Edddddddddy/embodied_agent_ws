import queue
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Optional

from .wakeword import WakeWordGate
from .wake_provider import TextWakeProvider, WakeDecision, WakeEvent, WakeEventKind


DEFAULT_SLEEP_WORDS = ("退出控制", "结束控制", "休眠", "睡眠", "先这样")
DEFAULT_PRIORITY_STOP_WORDS = ("停下", "停止", "急停", "刹车", "别动")


class SessionEventKind(str, Enum):
    WAKE = "wake"
    COMMAND = "command"
    REJECTED = "rejected"
    SLEEP = "sleep"


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
    event: "SessionEvent" = None


@dataclass(frozen=True)
class SessionEvent:
    kind: SessionEventKind
    session_state: str
    wake_event: WakeEvent
    transcript: str = ""
    command: Optional[str] = None

    def as_dict(self) -> dict:
        return {
            "kind": self.kind.value,
            "session_state": self.session_state,
            "wake_event": self.wake_event.as_dict(),
            "transcript": self.transcript,
            "command": self.command,
        }


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


@dataclass(frozen=True)
class CommandQueueEvent:
    event: str
    source: str
    text: str
    size: int
    dropped: int = 0
    reason: str = ""
    priority_stop: bool = False

    def as_dict(self) -> dict:
        return {
            "event": self.event,
            "source": self.source,
            "text": self.text,
            "size": self.size,
            "dropped": self.dropped,
            "reason": self.reason,
            "priority_stop": self.priority_stop,
        }


@dataclass(frozen=True)
class CommandExecutionEvent:
    event: str
    source: str
    text: str
    success: Optional[bool] = None
    reason: str = ""

    def as_dict(self) -> dict:
        return {
            "event": self.event,
            "source": self.source,
            "text": self.text,
            "success": self.success,
            "reason": self.reason,
        }


class CommandExecutionTracker:
    """连续控制的可观测事件生成器。

    队列本身只负责线程安全地存取命令；tracker 负责把 enqueue/clear/started/finished
    变成稳定 JSON 契约，供 monitor、测试或 UI 观察。
    """

    def __init__(self, source: str):
        self.source = source

    def queue_event(
        self,
        event: str,
        text: str,
        snapshot: QueueSnapshot,
        *,
        priority_stop: bool = False,
    ) -> CommandQueueEvent:
        normalized = event
        if event == "enqueue" and not snapshot.accepted:
            normalized = "rejected"
        return CommandQueueEvent(
            normalized,
            self.source,
            text,
            snapshot.size,
            snapshot.dropped,
            snapshot.reason,
            priority_stop,
        )

    def execution_started(self, item: "QueuedCommand") -> CommandExecutionEvent:
        return CommandExecutionEvent("started", self.source, item.text)

    def execution_finished(
        self,
        item: "QueuedCommand",
        *,
        success: bool,
        reason: str,
    ) -> CommandExecutionEvent:
        return CommandExecutionEvent("finished", self.source, item.text, success, reason)


class ContinuousVoiceSession:
    """把文本级唤醒词升级为“一次唤醒，多轮控制”的会话状态机。

    第一阶段仍复用现有 ASR 文本唤醒词，避免引入声学 KWS 依赖；后续接
    sherpa-onnx KWS/openWakeWord 时，只需要把 wake event 映射到这里。
    """

    def __init__(
        self,
        wake_gate: WakeWordGate | TextWakeProvider,
        *,
        enabled: bool,
        sleep_words: Iterable[str] = DEFAULT_SLEEP_WORDS,
        priority_stop_words: Iterable[str] = DEFAULT_PRIORITY_STOP_WORDS,
    ):
        self._wake_provider = (
            wake_gate
            if isinstance(wake_gate, TextWakeProvider)
            else TextWakeProvider.from_gate(wake_gate)
        )
        self.enabled = enabled
        self._sleep_words = tuple(word for word in sleep_words if word)
        self._priority_stop_words = tuple(word for word in priority_stop_words if word)

    def accept(self, transcript: str) -> SessionDecision:
        text = transcript.strip()
        if not text:
            return self._decision_from_wake(
                WakeDecision(
                    None,
                    self._wake_provider.active,
                    WakeEvent(WakeEventKind.REJECTED, "text", transcript, None),
                ),
                accepted=False,
                reason="empty",
                command=None,
                priority_stop=False,
            )

        if self.enabled and self._contains_any(text, self._sleep_words):
            wake_event = self._wake_provider.sleep()
            return SessionDecision(
                None,
                False,
                "session_sleep",
                False,
                False,
                SessionEvent(
                    SessionEventKind.SLEEP,
                    "sleeping",
                    wake_event,
                    transcript,
                    None,
                ),
            )

        wake_decision = self._wake_provider.accept(text)
        command = wake_decision.command
        if command is None:
            reason = "session_awake" if wake_decision.active else "wake_word_not_detected"
            return self._decision_from_wake(
                wake_decision,
                accepted=False,
                reason=reason,
                command=None,
                priority_stop=False,
            )

        priority_stop = self._contains_any(command, self._priority_stop_words)
        return self._decision_from_wake(
            wake_decision,
            accepted=True,
            reason="accepted",
            command=command,
            priority_stop=priority_stop,
        )

    def external_wake(self, provider: str, transcript: str = "") -> SessionEvent:
        wake_event = self._wake_provider.external_wake(provider, transcript)
        return SessionEvent(
            SessionEventKind.WAKE,
            "awake",
            wake_event,
            transcript,
            None,
        )

    def external_sleep(self, provider: str) -> SessionEvent:
        wake_event = self._wake_provider.external_sleep(provider)
        return SessionEvent(
            SessionEventKind.SLEEP,
            "sleeping",
            wake_event,
            "",
            None,
        )

    @staticmethod
    def _contains_any(text: str, words: Iterable[str]) -> bool:
        return any(word in text for word in words)

    @staticmethod
    def _session_kind(wake_event: WakeEvent, accepted: bool) -> SessionEventKind:
        if wake_event.kind == WakeEventKind.SLEEP:
            return SessionEventKind.SLEEP
        if accepted:
            return SessionEventKind.COMMAND
        if wake_event.kind == WakeEventKind.WAKE:
            return SessionEventKind.WAKE
        return SessionEventKind.REJECTED

    def _decision_from_wake(
        self,
        wake_decision: WakeDecision,
        *,
        accepted: bool,
        reason: str,
        command: Optional[str],
        priority_stop: bool,
    ) -> SessionDecision:
        kind = self._session_kind(wake_decision.event, accepted)
        state = "awake" if wake_decision.active else "sleeping"
        return SessionDecision(
            command,
            accepted,
            reason,
            wake_decision.active,
            priority_stop,
            SessionEvent(kind, state, wake_decision.event, wake_decision.event.transcript, command),
        )


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
