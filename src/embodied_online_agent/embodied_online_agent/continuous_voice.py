import queue
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Iterable, Optional

from .wakeword import WakeWordGate
from .wake_provider import TextWakeProvider, WakeDecision, WakeEvent, WakeEventKind


DEFAULT_SLEEP_WORDS = ("退出控制", "结束控制", "休眠", "睡眠", "先这样")
DEFAULT_PRIORITY_STOP_WORDS = ("停下", "停止", "急停", "刹车", "别动")
DEFAULT_FILLER_WORDS = ("嗯", "嗯嗯", "啊", "哦", "噢", "呃", "额", "唔")
_PUNCTUATION_CHARS = " \t\r\n，。！？!?、,.；;：:"


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

    def queue_expired(self, item: "QueuedCommand", *, size: int) -> CommandQueueEvent:
        return CommandQueueEvent(
            "expired",
            self.source,
            item.text,
            size,
            dropped=1,
            reason="stale_command",
            priority_stop=item.priority_stop,
        )

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
        filler_words: Iterable[str] = DEFAULT_FILLER_WORDS,
        duplicate_window_s: float = 1.2,
        clock=time.monotonic,
    ):
        self._wake_provider = (
            wake_gate
            if isinstance(wake_gate, TextWakeProvider)
            else TextWakeProvider.from_gate(wake_gate)
        )
        self.enabled = enabled
        self._sleep_words = tuple(word for word in sleep_words if word)
        self._priority_stop_words = tuple(word for word in priority_stop_words if word)
        self._filler_words = tuple(self._normalize_short_text(word) for word in filler_words if word)
        self._duplicate_window_s = max(0.0, float(duplicate_window_s))
        self._clock = clock
        self._last_command_text = ""
        self._last_command_at = 0.0
        self._had_active_session = False

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

        # 真实 ASR 长时间开麦时经常输出“嗯。”、“啊。”这类语气词 final。
        # 在会话层提前过滤，避免无意义文本进入 LLM 或动作队列。
        if self._is_filler(text):
            return self._decision_from_wake(
                WakeDecision(
                    None,
                    self._wake_provider.active,
                    WakeEvent(WakeEventKind.REJECTED, "text", transcript, None),
                ),
                accepted=False,
                reason="filler",
                command=None,
                priority_stop=False,
            )

        if self.enabled and self._contains_any(text, self._sleep_words):
            wake_event = self._wake_provider.sleep()
            self._had_active_session = False
            self._forget_command()
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
        if wake_decision.event.kind == WakeEventKind.WAKE:
            self._had_active_session = True
            self._forget_command()
        if command is None:
            reason = "session_awake" if wake_decision.active else "wake_word_not_detected"
            if not wake_decision.active and self._had_active_session:
                reason = "session_timeout"
                self._had_active_session = False
                self._forget_command()
            return self._decision_from_wake(
                wake_decision,
                accepted=False,
                reason=reason,
                command=None,
                priority_stop=False,
            )

        priority_stop = self._contains_any(command, self._priority_stop_words)
        if not priority_stop and self._is_duplicate_command(command):
            return self._decision_from_wake(
                wake_decision,
                accepted=False,
                reason="duplicate_command",
                command=None,
                priority_stop=False,
            )
        if not priority_stop:
            self._remember_command(command)
        self._had_active_session = wake_decision.active
        return self._decision_from_wake(
            wake_decision,
            accepted=True,
            reason="accepted",
            command=command,
            priority_stop=priority_stop,
        )

    def external_wake(self, provider: str, transcript: str = "") -> SessionEvent:
        wake_event = self._wake_provider.external_wake(provider, transcript)
        self._had_active_session = True
        self._forget_command()
        return SessionEvent(
            SessionEventKind.WAKE,
            "awake",
            wake_event,
            transcript,
            None,
        )

    def external_sleep(self, provider: str) -> SessionEvent:
        wake_event = self._wake_provider.external_sleep(provider)
        self._had_active_session = False
        self._forget_command()
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
    def _normalize_short_text(text: str) -> str:
        return "".join(ch for ch in text if ch not in _PUNCTUATION_CHARS).lower()

    def _is_filler(self, text: str) -> bool:
        normalized = self._normalize_short_text(text)
        return bool(normalized) and normalized in self._filler_words

    def _is_duplicate_command(self, command: str) -> bool:
        if self._duplicate_window_s <= 0.0:
            return False
        normalized = self._normalize_short_text(command)
        if not normalized or normalized != self._last_command_text:
            return False
        return (self._clock() - self._last_command_at) <= self._duplicate_window_s

    def _remember_command(self, command: str) -> None:
        self._last_command_text = self._normalize_short_text(command)
        self._last_command_at = self._clock()

    def _forget_command(self) -> None:
        self._last_command_text = ""
        self._last_command_at = 0.0

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

    def __init__(self, max_size: int = 8, max_age_s: float = 30.0, clock=time.monotonic):
        self._queue: queue.Queue[QueuedCommand] = queue.Queue(maxsize=max(1, max_size))
        self._max_age_s = max(0.0, float(max_age_s))
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

    def get(
        self,
        timeout: float = 0.1,
        on_stale: Callable[[QueuedCommand], None] | None = None,
    ) -> QueuedCommand:
        while True:
            item = self._queue.get(timeout=timeout)
            if not self._is_stale(item):
                return item
            if on_stale is not None:
                on_stale(item)
            self._queue.task_done()

    def _is_stale(self, item: QueuedCommand) -> bool:
        if item.priority_stop or self._max_age_s <= 0.0:
            return False
        return (self._clock() - item.created_at) > self._max_age_s

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
