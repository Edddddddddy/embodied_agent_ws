import threading
import time

from .types import LatencySnapshot


class LatencyTracker:
    """Thread-safe timestamps for one conversational turn."""

    def __init__(self, clock=time.perf_counter):
        self._clock = clock
        self._lock = threading.Lock()
        self.reset()

    def reset(self) -> None:
        with getattr(self, "_lock", threading.Lock()):
            self.asr_final_at = None
            self.llm_requested_at = None
            self.llm_first_token_at = None
            self.tts_requested_at = None
            self.tts_first_audio_at = None

    def mark_asr_final(self) -> None:
        with self._lock:
            self.asr_final_at = self._clock()

    def mark_llm_requested(self) -> None:
        with self._lock:
            self.llm_requested_at = self._clock()

    def mark_llm_first_token(self) -> None:
        with self._lock:
            if self.llm_first_token_at is None:
                self.llm_first_token_at = self._clock()

    def mark_tts_requested(self) -> None:
        with self._lock:
            if self.tts_requested_at is None:
                self.tts_requested_at = self._clock()

    def mark_tts_first_audio(self) -> None:
        with self._lock:
            if self.tts_first_audio_at is None:
                self.tts_first_audio_at = self._clock()

    @staticmethod
    def _elapsed(start, end):
        if start is None or end is None:
            return None
        return round((end - start) * 1000.0, 2)

    def snapshot(self) -> LatencySnapshot:
        with self._lock:
            return LatencySnapshot(
                llm_first_token_ms=self._elapsed(
                    self.llm_requested_at, self.llm_first_token_at
                ),
                asr_to_first_token_ms=self._elapsed(
                    self.asr_final_at, self.llm_first_token_at
                ),
                tts_first_audio_ms=self._elapsed(
                    self.tts_requested_at, self.tts_first_audio_at
                ),
            )

