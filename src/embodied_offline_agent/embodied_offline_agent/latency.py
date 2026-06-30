import time
from dataclasses import asdict, dataclass
from typing import Optional


@dataclass
class OfflineLatency:
    asr_finalize_ms: Optional[float] = None
    llm_first_token_ms: Optional[float] = None
    end_to_first_audio_ms: Optional[float] = None
    turn_complete_ms: Optional[float] = None

    def __post_init__(self):
        self._turn_started = time.perf_counter()
        self._silence_at: Optional[float] = None
        self._llm_at: Optional[float] = None

    def mark_silence(self):
        self._silence_at = time.perf_counter()
        self._turn_started = self._silence_at

    def mark_asr_final(self):
        if self._silence_at is not None:
            self.asr_finalize_ms = (time.perf_counter() - self._silence_at) * 1000.0

    def mark_llm_start(self):
        self._llm_at = time.perf_counter()

    def mark_first_token(self):
        if self._llm_at is not None and self.llm_first_token_ms is None:
            self.llm_first_token_ms = (time.perf_counter() - self._llm_at) * 1000.0

    def mark_first_audio(self):
        if self.end_to_first_audio_ms is None:
            self.end_to_first_audio_ms = (time.perf_counter() - self._turn_started) * 1000.0

    def finish(self):
        self.turn_complete_ms = (time.perf_counter() - self._turn_started) * 1000.0

    def report(self, message_dropped: int, audio_dropped: int) -> dict:
        value = asdict(self)
        value.update({
            "asr_target_met": self.asr_finalize_ms is not None and self.asr_finalize_ms < 600.0,
            "e2e_target_met": self.end_to_first_audio_ms is not None and self.end_to_first_audio_ms < 3500.0,
            "message_buffer_dropped": message_dropped,
            "audio_buffer_dropped": audio_dropped,
        })
        return value
