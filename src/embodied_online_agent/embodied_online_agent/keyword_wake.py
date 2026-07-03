"""Keyword spotting sidecar adapters.

本模块把“检测到唤醒词”抽象成一个很小的接口：输出标准
`/agent/wake_event_input` JSON。真实模型可以是 sherpa-onnx KWS，测试/演示也可以是
文本 detector；Agent 不需要知道唤醒来自哪个声学模型。
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass
from typing import Iterable, Protocol


@dataclass(frozen=True)
class KeywordWakeMatch:
    keyword: str
    provider: str
    score: float = 1.0


class KeywordWakeDetector(Protocol):
    provider_name: str

    def detect_text(self, text: str) -> KeywordWakeMatch | None:
        ...

    def detect_audio(self, pcm16: bytes) -> KeywordWakeMatch | None:
        ...


class TextKeywordWakeDetector:
    """无模型 mock/text detector，用于 sidecar 链路验收和手工调试。"""

    provider_name = "mock_kws"

    def __init__(
        self,
        keywords: Iterable[str],
        *,
        aliases: Iterable[str] = (),
        provider_name: str = "mock_kws",
    ):
        self.provider_name = provider_name
        triggers = [word.strip() for word in (*tuple(keywords), *tuple(aliases)) if word.strip()]
        self._triggers = tuple(sorted(set(triggers), key=len, reverse=True))

    def detect_text(self, text: str) -> KeywordWakeMatch | None:
        lowered = text.strip().lower()
        if not lowered:
            return None
        for keyword in self._triggers:
            if keyword.lower() in lowered:
                return KeywordWakeMatch(keyword, self.provider_name)
        return None

    def detect_audio(self, _pcm16: bytes) -> KeywordWakeMatch | None:
        return None


class SherpaKeywordWakeDetector:
    """sherpa-onnx open-vocabulary KWS adapter.

    参考 sherpa-onnx 官方 `KeywordSpotter` Python 示例：创建 stream，持续
    `accept_waveform()` / `decode_stream()`，`get_result()` 非空时立刻 `reset_stream()`。
    该依赖是可选依赖，只有 `kws_provider:=sherpa` 时才会导入。
    """

    provider_name = "sherpa_kws"

    def __init__(
        self,
        *,
        tokens: str,
        encoder: str,
        decoder: str,
        joiner: str,
        keywords_file: str,
        sample_rate: int = 16000,
        num_threads: int = 1,
        provider: str = "cpu",
        provider_name: str = "sherpa_kws",
    ):
        import numpy as np
        import sherpa_onnx

        self.provider_name = provider_name
        self._np = np
        self._sample_rate = sample_rate
        self._spotter = sherpa_onnx.KeywordSpotter(
            tokens=tokens,
            encoder=encoder,
            decoder=decoder,
            joiner=joiner,
            num_threads=num_threads,
            keywords_file=keywords_file,
            provider=provider,
        )
        self._stream = self._spotter.create_stream()

    def detect_text(self, _text: str) -> KeywordWakeMatch | None:
        return None

    def detect_audio(self, pcm16: bytes) -> KeywordWakeMatch | None:
        if not pcm16:
            return None
        samples = self._np.frombuffer(pcm16, dtype="<i2").astype(self._np.float32) / 32768.0
        self._stream.accept_waveform(self._sample_rate, samples)
        while self._spotter.is_ready(self._stream):
            self._spotter.decode_stream(self._stream)
        result = self._spotter.get_result(self._stream)
        keyword = result.text if hasattr(result, "text") else str(result)
        keyword = keyword.strip()
        if not keyword:
            return None
        self._spotter.reset_stream(self._stream)
        return KeywordWakeMatch(keyword, self.provider_name)


class KeywordWakeBridge:
    """把 detector match 转成 `/agent/wake_event_input` JSON，并处理冷却时间。"""

    def __init__(self, *, cooldown_s: float = 1.0, clock=time.monotonic):
        self._cooldown_s = max(0.0, float(cooldown_s))
        self._clock = clock
        self._next_allowed_at = 0.0

    def wake_payload(self, match: KeywordWakeMatch | None) -> str | None:
        if match is None:
            return None
        now = self._clock()
        if now < self._next_allowed_at:
            return None
        self._next_allowed_at = now + self._cooldown_s
        return json.dumps(
            {
                "kind": "wake",
                "provider": match.provider,
                "transcript": match.keyword,
                "score": round(match.score, 3),
            },
            ensure_ascii=False,
        )
