import time
from typing import Iterable, Optional


class WakeWordGate:
    """Transcript-level wake word gate with a short follow-up window."""

    def __init__(
        self,
        words: Iterable[str],
        enabled: bool = True,
        active_timeout_s: float = 10.0,
        clock=time.monotonic,
    ):
        self.words = tuple(word.strip().lower() for word in words if word.strip())
        self.enabled = enabled
        self.active_timeout_s = active_timeout_s
        self._clock = clock
        self._active_until = 0.0

    def process(self, transcript: str) -> Optional[str]:
        text = transcript.strip()
        if not text:
            return None
        if not self.enabled:
            return text

        lowered = text.lower()
        for word in self.words:
            index = lowered.find(word)
            if index >= 0:
                self._active_until = self._clock() + self.active_timeout_s
                remainder = (text[:index] + text[index + len(word) :]).strip(" ，,。.!！?？")
                return remainder or None

        if self._clock() <= self._active_until:
            self._active_until = self._clock() + self.active_timeout_s
            return text
        return None

    @property
    def active(self) -> bool:
        return self._clock() <= self._active_until

