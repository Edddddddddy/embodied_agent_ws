from dataclasses import asdict, dataclass
import json


@dataclass(frozen=True)
class RecognitionRetry:
    status: str
    reason: str
    transcript: str
    attempt: int
    max_attempts: int
    prompt: str = "没有听清唤醒词，请再说一次"

    def to_json(self) -> str:
        return json.dumps(asdict(self), ensure_ascii=False)


class RecognitionRetryTracker:
    """Counts failed utterances without ever locking recognition out."""

    def __init__(self, max_attempts: int = 3):
        self.max_attempts = max(1, int(max_attempts))
        self._attempt = 0

    def failed(self, transcript: str) -> RecognitionRetry:
        self._attempt = self._attempt % self.max_attempts + 1
        return RecognitionRetry(
            status="retry",
            reason="wake_word_not_detected",
            transcript=transcript,
            attempt=self._attempt,
            max_attempts=self.max_attempts,
        )

    def succeeded(self) -> None:
        self._attempt = 0
