from abc import ABC, abstractmethod
from typing import Callable, Dict, Iterable, Iterator, List


class AsrProvider(ABC):
    @abstractmethod
    def start(self, on_partial: Callable[[str], None], on_final: Callable[[str], None]):
        raise NotImplementedError

    @abstractmethod
    def push_audio(self, pcm16: bytes):
        raise NotImplementedError

    @abstractmethod
    def commit(self):
        raise NotImplementedError

    @abstractmethod
    def stop(self):
        raise NotImplementedError


class LlmProvider(ABC):
    @abstractmethod
    def stream(self, messages: List[Dict[str, str]]) -> Iterator[str]:
        raise NotImplementedError


class TtsProvider(ABC):
    @abstractmethod
    def synthesize(
        self,
        text_chunks: Iterable[str],
        on_audio: Callable[[bytes], None],
    ) -> None:
        raise NotImplementedError

    def close(self) -> None:
        """Release an optional persistent TTS session."""
        return None
