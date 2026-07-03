from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Optional

from .wakeword import WakeWordGate


class WakeEventKind(str, Enum):
    WAKE = "wake"
    CONTINUE = "continue"
    REJECTED = "rejected"
    SLEEP = "sleep"


@dataclass(frozen=True)
class WakeEvent:
    """唤醒事件是连续会话与具体 KWS 实现之间的稳定契约。"""

    kind: WakeEventKind
    provider: str
    transcript: str = ""
    command: Optional[str] = None

    def as_dict(self) -> dict:
        return {
            "kind": self.kind.value,
            "provider": self.provider,
            "transcript": self.transcript,
            "command": self.command,
        }


@dataclass(frozen=True)
class WakeDecision:
    command: Optional[str]
    active: bool
    event: WakeEvent


class TextWakeProvider:
    """当前默认的文本级唤醒 provider。

    它复用 ASR 之后的“小智/小志/晓智”文本门控；后续 sherpa-onnx KWS 或 openWakeWord
    只要输出同样的 WakeEvent/WakeDecision，连续会话无需知道具体声学模型。
    """

    provider_name = "text"

    def __init__(
        self,
        words: Iterable[str],
        aliases: Iterable[str] = (),
        enabled: bool = True,
        active_timeout_s: float = 10.0,
        clock=None,
    ):
        kwargs = {
            "aliases": aliases,
            "enabled": enabled,
            "active_timeout_s": active_timeout_s,
        }
        if clock is not None:
            kwargs["clock"] = clock
        self._gate = WakeWordGate(words, **kwargs)

    @classmethod
    def from_gate(cls, gate: WakeWordGate) -> "TextWakeProvider":
        provider = cls([], enabled=False)
        provider._gate = gate
        return provider

    def accept(self, transcript: str) -> WakeDecision:
        was_active = self.active
        command = self._gate.process(transcript)
        active = self.active
        if command is not None:
            kind = WakeEventKind.CONTINUE if was_active else WakeEventKind.WAKE
            return WakeDecision(
                command,
                active,
                WakeEvent(kind, self.provider_name, transcript, command),
            )
        if active and not was_active:
            return WakeDecision(
                None,
                active,
                WakeEvent(WakeEventKind.WAKE, self.provider_name, transcript, None),
            )
        return WakeDecision(
            None,
            active,
            WakeEvent(WakeEventKind.REJECTED, self.provider_name, transcript, None),
        )

    def sleep(self) -> WakeEvent:
        self._gate.sleep()
        return WakeEvent(WakeEventKind.SLEEP, self.provider_name)

    @property
    def active(self) -> bool:
        return True if not self._gate.enabled else self._gate.active
