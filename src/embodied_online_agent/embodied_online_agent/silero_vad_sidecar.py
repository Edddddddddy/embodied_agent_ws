"""可选 Silero VAD sidecar 的核心逻辑。

这个模块刻意不依赖 ROS：ROS 节点只负责订阅/发布，端点检测和模型调用都通过
StreamingVadEndpoint 这个小接口测试。这样后续从 silero-vad 切换到 sherpa VAD、
WebRTC VAD 或硬件 DSP 时，Agent 侧仍然只消费 speech_started/speech_ended 事件。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Protocol


class VadEventName(str, Enum):
    SPEECH_STARTED = "speech_started"
    SPEECH_ENDED = "speech_ended"


@dataclass(frozen=True)
class VadEvent:
    name: VadEventName
    reason: str = ""
    probability: float = 0.0


class VadProbabilityProvider(Protocol):
    """VAD 模型 adapter 的最小接口。

    输入是单帧 PCM16 little-endian，输出是 [0, 1] 的人声概率。
    """

    def speech_probability(self, pcm_frame: bytes, sample_rate: int) -> float:
        ...


class SileroVadUnavailableError(RuntimeError):
    pass


class StreamingVadEndpoint:
    """把帧级 VAD 概率转换成稳定的端点事件。

    公开接口只有 process_pcm(bytes)，它可以接收任意大小 PCM chunk；内部按固定
    frame_ms 聚合后调用 provider。短噪声只发 speech_started，不发 speech_ended，
    与现有 C++ SpeechEndpointDetector 的行为保持一致。
    """

    def __init__(
        self,
        probability_provider: VadProbabilityProvider,
        *,
        sample_rate: int,
        frame_ms: int,
        threshold: float,
        speech_end_silence_s: float,
        min_utterance_ms: float,
        max_utterance_s: float,
    ):
        if sample_rate <= 0:
            raise ValueError("sample_rate must be positive")
        if frame_ms <= 0:
            raise ValueError("frame_ms must be positive")
        if not 0.0 <= threshold <= 1.0:
            raise ValueError("threshold must be in [0, 1]")
        if speech_end_silence_s <= 0.0:
            raise ValueError("speech_end_silence_s must be positive")
        if min_utterance_ms < 0.0:
            raise ValueError("min_utterance_ms must be non-negative")
        if max_utterance_s <= 0.0:
            raise ValueError("max_utterance_s must be positive")

        self._provider = probability_provider
        self._sample_rate = sample_rate
        self._frame_seconds = frame_ms / 1000.0
        self._frame_bytes = int(sample_rate * self._frame_seconds) * 2
        if self._frame_bytes <= 0:
            raise ValueError("frame_ms is too small for sample_rate")
        self._threshold = threshold
        self._end_silence_s = speech_end_silence_s
        self._min_utterance_s = min_utterance_ms / 1000.0
        self._max_utterance_s = max_utterance_s
        self._buffer = bytearray()
        self._in_utterance = False
        self._speech_s = 0.0
        self._silence_s = 0.0

    def process_pcm(self, pcm_chunk: bytes) -> list[VadEvent]:
        if len(pcm_chunk) % 2 != 0:
            pcm_chunk = pcm_chunk[:-1]
        self._buffer.extend(pcm_chunk)
        events: list[VadEvent] = []
        while len(self._buffer) >= self._frame_bytes:
            frame = bytes(self._buffer[: self._frame_bytes])
            del self._buffer[: self._frame_bytes]
            probability = self._provider.speech_probability(frame, self._sample_rate)
            events.extend(self._update(probability >= self._threshold, probability))
        return events

    def reset(self) -> None:
        self._buffer.clear()
        self._in_utterance = False
        self._speech_s = 0.0
        self._silence_s = 0.0

    def _update(self, speech: bool, probability: float) -> list[VadEvent]:
        events: list[VadEvent] = []
        if speech:
            if not self._in_utterance:
                self._in_utterance = True
                events.append(VadEvent(VadEventName.SPEECH_STARTED, probability=probability))
            self._speech_s += self._frame_seconds
            self._silence_s = 0.0
            if self._speech_s + 1e-9 >= self._max_utterance_s:
                events.extend(self._finish("max_duration", probability))
            return events

        if not self._in_utterance:
            return events
        self._silence_s += self._frame_seconds
        if self._silence_s + 1e-9 >= self._end_silence_s:
            events.extend(self._finish("silence", probability))
        return events

    def _finish(self, reason: str, probability: float) -> list[VadEvent]:
        long_enough = self._speech_s + 1e-9 >= self._min_utterance_s
        self._in_utterance = False
        self._speech_s = 0.0
        self._silence_s = 0.0
        if not long_enough:
            return []
        return [VadEvent(VadEventName.SPEECH_ENDED, reason=reason, probability=probability)]


class SileroVadProvider:
    """silero-vad Python 包 adapter。

    依赖是可选的：默认工程和 CI 不安装 torch/onnxruntime/silero-vad；连续语音脚本的
    VAD_PROVIDER=auto 会在依赖可用时才切到 silero。如果用户显式启动 vad_provider:=silero
    但依赖缺失，节点会给出可执行的安装提示。
    """

    def __init__(self, *, use_onnx: bool = True, model_path: str = ""):
        try:
            import numpy as np
            import torch
            from silero_vad import load_silero_vad
        except Exception as error:  # pragma: no cover - 依赖缺失时只在真实节点启动路径触发
            raise SileroVadUnavailableError(
                "Silero VAD 依赖未安装。可执行：pip install silero-vad onnxruntime "
                "（或安装 torch 后使用 silero-vad 默认模型）。"
            ) from error

        self._np = np
        self._torch = torch
        kwargs = {"onnx": use_onnx}
        if model_path:
            path = Path(model_path).expanduser()
            if not path.exists():
                raise SileroVadUnavailableError(f"Silero VAD model_path 不存在: {path}")
            kwargs["model_file_path"] = str(path)
        try:
            self._model = load_silero_vad(**kwargs)
        except TypeError:
            # 兼容不同 silero-vad 版本：旧版可能不支持 model_file_path。
            kwargs.pop("model_file_path", None)
            self._model = load_silero_vad(**kwargs)

    def speech_probability(self, pcm_frame: bytes, sample_rate: int) -> float:
        audio = self._np.frombuffer(pcm_frame, dtype="<i2").astype("float32") / 32768.0
        tensor = self._torch.from_numpy(audio)
        with self._torch.no_grad():
            probability = self._model(tensor, sample_rate)
        if hasattr(probability, "item"):
            return float(probability.item())
        return float(probability)
