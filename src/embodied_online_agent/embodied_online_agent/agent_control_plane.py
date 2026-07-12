"""在线/离线 Agent 共用的语音控制面。

ASR、LLM、TTS 属于可替换的数据面；会话、队列、NLU 前处理和 ROS 事件契约则必须
保持一致。本模块把后者收敛成一个组合式深模块，避免 online/offline 节点各自维护一份
状态机。领域状态与 ROS 发布 Adapter 分开，便于不启动 ROS graph 就测试核心行为。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Iterable

from .command_completion import CommandCompleter
from .command_nlu import CommandNLU
from .command_normalizer import CommandNormalizer
from .continuous_voice import (
    CommandExecutionTracker,
    ContinuousCommandQueue,
    ContinuousVoiceSession,
    QueueSnapshot,
)
from .navigation_phrases import is_navigation_cancel
from .recognition_retry import RecognitionRetryTracker
from .transcript_stabilizer import TranscriptStabilizer
from .wakeword import WakeWordGate


@dataclass(frozen=True)
class AgentControlPlaneConfig:
    source: str
    continuous_enabled: bool
    queue_size: int
    command_max_age_s: float
    wake_words: Iterable[str]
    wake_word_aliases: Iterable[str]
    wake_word_enabled: bool
    wake_timeout_s: float
    duplicate_window_s: float
    recognition_max_retries: int
    command_normalization_enabled: bool
    normalization_feedback_enabled: bool
    normalization_fuzzy_threshold: float
    normalization_rules_path: Path | str
    command_completion_enabled: bool
    command_nlu_enabled: bool
    command_nlu_min_confidence: float
    partial_merge_enabled: bool
    partial_max_age_s: float

    @classmethod
    def from_parameters(
        cls,
        source: str,
        param: Callable[[str], Any],
        normalization_rules_path: Path | str,
    ) -> "AgentControlPlaneConfig":
        """从两类 Agent 的同名参数构造唯一控制面配置。"""
        continuous_enabled = bool(param("continuous_control_enabled"))
        return cls(
            source=source,
            continuous_enabled=continuous_enabled,
            queue_size=int(param("continuous_command_queue_size")),
            command_max_age_s=float(param("continuous_command_max_age_s")),
            wake_words=param("wake_words"),
            wake_word_aliases=param("wake_word_aliases"),
            wake_word_enabled=bool(param("wake_word_enabled")),
            wake_timeout_s=float(
                param("voice_session_timeout_s")
                if continuous_enabled
                else param("wake_active_timeout_s")
            ),
            duplicate_window_s=float(param("continuous_duplicate_window_s")),
            recognition_max_retries=int(param("recognition_max_retries")),
            command_normalization_enabled=bool(
                param("command_normalization_enabled")
            ),
            normalization_feedback_enabled=bool(
                param("command_normalization_feedback_enabled")
            ),
            normalization_fuzzy_threshold=float(
                param("command_normalization_fuzzy_threshold")
            ),
            normalization_rules_path=normalization_rules_path,
            command_completion_enabled=bool(param("command_completion_enabled")),
            command_nlu_enabled=bool(param("command_nlu_enabled")),
            command_nlu_min_confidence=float(param("command_nlu_min_confidence")),
            partial_merge_enabled=bool(param("asr_partial_merge_enabled")),
            partial_max_age_s=float(param("asr_partial_max_age_s")),
        )


@dataclass(frozen=True)
class TranscriptControlDecision:
    """一次 ASR final 穿过控制面后的完整、可测试决策。"""

    directive: str
    transcript: str
    command: str = ""
    state: str = ""
    session_event: object | None = None
    recognition_feedback: tuple[dict, ...] = field(default_factory=tuple)
    queue_event: object | None = None
    cancel_reason: str = ""
    priority_action: str = ""
    dropped: int = 0


class AgentControlPlane:
    """拥有与 provider 无关的状态，并生成统一的命令生命周期事件。"""

    def __init__(self, config: AgentControlPlaneConfig):
        self.source = config.source
        self.continuous_enabled = config.continuous_enabled
        self.command_queue = ContinuousCommandQueue(
            config.queue_size, max_age_s=config.command_max_age_s
        )
        self._tracker = CommandExecutionTracker(config.source)
        self.wake_gate = WakeWordGate(
            config.wake_words,
            aliases=config.wake_word_aliases,
            enabled=config.wake_word_enabled,
            active_timeout_s=config.wake_timeout_s,
        )
        self.voice_session = ContinuousVoiceSession(
            self.wake_gate,
            enabled=config.continuous_enabled,
            duplicate_window_s=config.duplicate_window_s,
        )
        self.retry_tracker = RecognitionRetryTracker(
            config.recognition_max_retries
        )
        self.command_normalizer = CommandNormalizer(
            fuzzy_threshold=config.normalization_fuzzy_threshold,
            rules_path=config.normalization_rules_path,
        )
        self.command_normalization_enabled = config.command_normalization_enabled
        self.normalization_feedback_enabled = config.normalization_feedback_enabled
        self.command_completer = CommandCompleter(
            enabled=config.command_completion_enabled
        )
        self.command_nlu = CommandNLU(
            enabled=config.command_nlu_enabled,
            min_confidence=config.command_nlu_min_confidence,
        )
        self.transcript_stabilizer = TranscriptStabilizer(
            enabled=config.partial_merge_enabled,
            max_age_s=config.partial_max_age_s,
        )
        self._nlu_batch_sequence = 0

    def next_nlu_batch_id(self) -> str:
        self._nlu_batch_sequence += 1
        return f"{self.source}-nlu-{self._nlu_batch_sequence}"

    def accept_transcript(
        self, transcript: str, *, wake_word_required: bool
    ) -> TranscriptControlDecision:
        """统一执行归一化、会话门控、补全和优先控制决策。

        这里可以修改队列状态，但不会发布 ROS 消息或调用动作执行器；调用方根据返回值
        处理 provider 特有的 turn/latency，ROS Adapter 负责发布可观测事件。
        """
        feedback: list[dict] = []
        normalized = (
            self.command_normalizer.normalize(transcript)
            if self.command_normalization_enabled
            else None
        )
        if (
            normalized is not None
            and normalized.changed
            and self.normalization_feedback_enabled
        ):
            feedback.append(normalized.feedback_dict())
        clean_transcript = normalized.text if normalized is not None else transcript
        session = self.voice_session.accept(clean_transcript)

        if not session.accepted or session.command is None:
            if session.reason == "session_sleep":
                dropped = self.command_queue.clear()
                snapshot = QueueSnapshot(
                    True, self.command_queue.size(), dropped
                )
                return TranscriptControlDecision(
                    "sleep",
                    clean_transcript,
                    state="sleeping",
                    session_event=session.event,
                    recognition_feedback=tuple(feedback),
                    queue_event=self.queue_event("clear", "", snapshot),
                    cancel_reason="session_sleep",
                    priority_action="stop",
                    dropped=dropped,
                )
            if session.reason == "session_awake":
                return TranscriptControlDecision(
                    "awake",
                    clean_transcript,
                    state="session_awake",
                    session_event=session.event,
                    recognition_feedback=tuple(feedback),
                )
            if session.reason in {"filler", "duplicate_command"}:
                feedback.append(
                    {
                        "status": "ignored",
                        "reason": session.reason,
                        "transcript": clean_transcript,
                    }
                )
                return TranscriptControlDecision(
                    "ignored",
                    clean_transcript,
                    state="listening",
                    session_event=session.event,
                    recognition_feedback=tuple(feedback),
                )
            if session.reason == "session_timeout":
                feedback.append(self.retry_tracker.failed(clean_transcript).as_dict())
                feedback.append(
                    {
                        "status": "session_timeout",
                        "reason": "voice_session_timeout",
                        "transcript": clean_transcript,
                        "prompt": "会话已超时，请先说小智",
                    }
                )
                return TranscriptControlDecision(
                    "retry",
                    clean_transcript,
                    state="retry_listening",
                    session_event=session.event,
                    recognition_feedback=tuple(feedback),
                )
            if wake_word_required and not self.wake_gate.active:
                feedback.append(self.retry_tracker.failed(clean_transcript).as_dict())
                return TranscriptControlDecision(
                    "retry",
                    clean_transcript,
                    state="retry_listening",
                    session_event=session.event,
                    recognition_feedback=tuple(feedback),
                )
            return TranscriptControlDecision(
                "noop",
                clean_transcript,
                session_event=session.event,
                recognition_feedback=tuple(feedback),
            )

        self.retry_tracker.succeeded()
        command = session.command
        if not session.priority_stop:
            completed = self.command_completer.complete(command)
            command = completed.text
            if completed.changed:
                feedback.append(completed.feedback_dict())

        navigation_cancel = is_navigation_cancel(command)
        if self.continuous_enabled and (session.priority_stop or navigation_cancel):
            cancel_reason = (
                "priority_stop"
                if session.priority_stop
                else "priority_navigation_cancel"
            )
            dropped = self.command_queue.clear()
            snapshot = QueueSnapshot(
                True, self.command_queue.size(), dropped, cancel_reason
            )
            return TranscriptControlDecision(
                "priority",
                clean_transcript,
                command=command,
                state="listening",
                session_event=session.event,
                recognition_feedback=tuple(feedback),
                queue_event=self.queue_event(
                    "clear",
                    command,
                    snapshot,
                    priority_stop=session.priority_stop,
                ),
                cancel_reason=cancel_reason,
                priority_action=(
                    "stop" if session.priority_stop else "cancel_navigation"
                ),
                dropped=dropped,
            )

        return TranscriptControlDecision(
            "command",
            clean_transcript,
            command=command,
            session_event=session.event,
            recognition_feedback=tuple(feedback),
        )

    def queue_event(
        self,
        event: str,
        text: str,
        snapshot: QueueSnapshot,
        *,
        priority_stop: bool = False,
    ):
        return self._tracker.queue_event(
            event, text, snapshot, priority_stop=priority_stop
        )

    def queue_expired(self, item):
        return self._tracker.queue_expired(
            item, size=self.command_queue.size()
        )

    def execution_started(self, item):
        return self._tracker.execution_started(item)

    def execution_finished(self, item, *, success, reason: str = ""):
        return self._tracker.execution_finished(
            item, success=success, reason=reason
        )
