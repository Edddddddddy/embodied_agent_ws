import time
from typing import Iterable, Optional


class WakeWordGate:
    """Transcript-level wake word gate with a short follow-up window."""

    def __init__(
        self,
        words: Iterable[str],
        aliases: Iterable[str] = (),
        enabled: bool = True,
        active_timeout_s: float = 10.0,
        clock=time.monotonic,
    ):
        triggers = {
            word.strip().lower()
            for word in (*tuple(words), *tuple(aliases))
            if word.strip()
        }
        self.words = tuple(sorted(triggers, key=len, reverse=True))
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

    def wake(self) -> None:
        """由外部声学 KWS 直接打开当前唤醒窗口。"""
        self._active_until = self._clock() + self.active_timeout_s

    def sleep(self) -> None:
        """立即退出当前唤醒会话。"""
        self._active_until = 0.0
