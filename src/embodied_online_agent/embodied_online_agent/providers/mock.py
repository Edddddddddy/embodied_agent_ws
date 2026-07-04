import json
import math
import struct
import time
from typing import Callable, Dict, Iterable, Iterator, List

from .base import AsrProvider, LlmProvider, TtsProvider


class MockAsr(AsrProvider):
    def __init__(self, scripted_finals: Iterable[str] | None = None):
        self._scripted_finals = list(scripted_finals or [])
        self.on_partial = None
        self.on_final = None

    def start(self, on_partial, on_final):
        self.on_partial = on_partial
        self.on_final = on_final

    def push_audio(self, pcm16: bytes):
        del pcm16

    def commit(self):
        if self._scripted_finals and self.on_final is not None:
            self.on_final(self._scripted_finals.pop(0))
        return None

    def stop(self):
        return None


class MockLlm(LlmProvider):
    def __init__(self, token_delay_s: float = 0.0):
        self.token_delay_s = token_delay_s

    def stream(self, messages: List[Dict[str, str]]) -> Iterator[str]:
        text = messages[-1]["content"]
        action = None
        if "前进" in text or "向前" in text:
            action = {"name": "move", "arguments": {"linear_x": 0.2, "duration_s": 1.0}}
        elif "后退" in text:
            action = {"name": "move", "arguments": {"linear_x": -0.2, "duration_s": 1.0}}
        elif "停止" in text:
            action = {"name": "stop", "arguments": {}}
        response = f"<speech>收到，正在处理：{text}。</speech>"
        if action:
            response += "<action>" + json.dumps(action, ensure_ascii=False) + "</action>"
        for char in response:
            if self.token_delay_s:
                time.sleep(self.token_delay_s)
            yield char


class MockTts(TtsProvider):
    """Produces a quiet audible tone, so the full pipeline is testable."""

    def __init__(self, sample_rate: int = 24000, chunk_duration_s: float = 0.04):
        self.sample_rate = sample_rate
        self.chunk_duration_s = chunk_duration_s

    def synthesize(self, text_chunks: Iterable[str], on_audio: Callable[[bytes], None]):
        phase = 0
        count = int(self.sample_rate * self.chunk_duration_s)
        for text in text_chunks:
            if not text:
                continue
            samples = []
            for _ in range(count):
                samples.append(int(500 * math.sin(2.0 * math.pi * 440.0 * phase / self.sample_rate)))
                phase += 1
            on_audio(struct.pack("<" + "h" * len(samples), *samples))
