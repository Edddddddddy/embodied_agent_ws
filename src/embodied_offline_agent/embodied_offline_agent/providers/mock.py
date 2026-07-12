import json
import math
import struct
import time


class MockOfflineAsr:
    def __init__(self, scripted_finals=None, scripted_partials=None):
        self._scripted_finals = list(scripted_finals or [])
        self._scripted_partials = list(scripted_partials or [])
        self._partial = None
        self._final = None

    def start(self, on_partial, on_final):
        self._partial, self._final = on_partial, on_final

    def push_audio(self, _pcm16: bytes):
        return None

    def commit(self):
        if self._scripted_partials and self._partial is not None:
            partial = self._scripted_partials.pop(0)
            if partial:
                self._partial(partial)
        if self._scripted_finals and self._final is not None:
            self._final(self._scripted_finals.pop(0))
        return None

    def reset(self):
        return None


class MockOfflineLlm:
    def __init__(self, token_delay_s: float = 0.0):
        self._delay = token_delay_s

    def stream(self, messages):
        text = messages[-1]["content"]
        action = None
        if "前进" in text or "向前" in text:
            action = {"name": "move", "arguments": {"linear_x": 0.2, "duration_s": 1.0}}
        elif "停止" in text:
            action = {"name": "stop", "arguments": {}}
        output = f"<speech>收到，离线执行：{text}。</speech>"
        if action:
            output += "<action>" + json.dumps(action, ensure_ascii=False) + "</action>"
        for char in output:
            if self._delay:
                time.sleep(self._delay)
            yield char


class MockOfflineTts:
    def __init__(self, sample_rate: int = 44100):
        self.sample_rate = sample_rate

    def synthesize(self, text: str) -> bytes:
        count = max(1, int(self.sample_rate * min(0.12, 0.02 + len(text) * 0.003)))
        samples = [int(350 * math.sin(2.0 * math.pi * 440.0 * i / self.sample_rate)) for i in range(count)]
        return struct.pack("<" + "h" * len(samples), *samples)
