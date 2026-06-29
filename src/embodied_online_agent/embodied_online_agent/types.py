from dataclasses import dataclass, field
from typing import Any, Dict


@dataclass(frozen=True)
class ActionCommand:
    name: str
    arguments: Dict[str, Any] = field(default_factory=dict)

    def as_dict(self) -> Dict[str, Any]:
        return {"name": self.name, "arguments": self.arguments}


@dataclass(frozen=True)
class LatencySnapshot:
    llm_first_token_ms: float | None
    asr_to_first_token_ms: float | None
    tts_first_audio_ms: float | None

    def as_dict(self) -> Dict[str, float | None]:
        return {
            "llm_first_token_ms": self.llm_first_token_ms,
            "asr_to_first_token_ms": self.asr_to_first_token_ms,
            "tts_first_audio_ms": self.tts_first_audio_ms,
        }

