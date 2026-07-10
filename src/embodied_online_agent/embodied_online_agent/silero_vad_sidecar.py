"""可选 Silero VAD sidecar 的核心逻辑。

这个模块刻意不依赖 ROS：ROS 节点只负责订阅/发布，端点检测和模型调用都通过
StreamingVadEndpoint 这个小接口测试。这样后续从 silero-vad 切换到 sherpa VAD、
WebRTC VAD 或硬件 DSP 时，Agent 侧仍然只消费 speech_started/speech_ended 事件。
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Callable, Protocol


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


class WebRtcVadUnavailableError(RuntimeError):
    pass


class SileroOnnxVadProvider:
    """不依赖 PyTorch 的 Silero ONNX 流式推理 adapter。

    Silero v6 的 ONNX 模型不是无状态分类器：每一帧都必须携带上一帧返回的 state，
    同时拼接 64（16kHz）或 32（8kHz）个历史采样点。把状态封装在 provider 内部，
    上层 endpoint 只需要处理人声概率，不会把模型细节泄漏到 ROS 节点。
    """

    _SAMPLES_PER_FRAME = {8000: 256, 16000: 512}
    _CONTEXT_SAMPLES = {8000: 32, 16000: 64}

    def __init__(
        self,
        *,
        model_path: str,
        session_factory: Callable[[str], object] | None = None,
    ):
        path = Path(model_path).expanduser()
        if not path.exists():
            raise SileroVadUnavailableError(f"Silero VAD model_path 不存在: {path}")
        try:
            import numpy as np
        except Exception as error:  # pragma: no cover - 基础运行时异常
            raise SileroVadUnavailableError("Silero ONNX 需要 numpy。") from error

        self._np = np
        self._session = (
            session_factory(str(path))
            if session_factory is not None
            else self._create_session(str(path))
        )
        self._last_sample_rate = 0
        self._reset_state(16000)

    @staticmethod
    def _create_session(model_path: str):
        try:
            import onnxruntime
        except Exception as error:  # pragma: no cover - 真实运行时依赖检查
            raise SileroVadUnavailableError(
                "Silero ONNX 依赖未安装。请执行："
                "bash scripts/setup_voice_vad_runtime.sh silero"
            ) from error

        options = onnxruntime.SessionOptions()
        options.inter_op_num_threads = 1
        options.intra_op_num_threads = 1
        providers = None
        if "CPUExecutionProvider" in onnxruntime.get_available_providers():
            providers = ["CPUExecutionProvider"]
        return onnxruntime.InferenceSession(
            model_path,
            sess_options=options,
            providers=providers,
        )

    def _reset_state(self, sample_rate: int) -> None:
        context_size = self._CONTEXT_SAMPLES[sample_rate]
        self._state = self._np.zeros((2, 1, 128), dtype=self._np.float32)
        self._context = self._np.zeros((1, context_size), dtype=self._np.float32)
        self._last_sample_rate = sample_rate

    def reset(self, sample_rate: int = 16000) -> None:
        """在新音频流开始时清空递归状态，避免上一段语音影响下一段概率。"""
        if sample_rate not in self._SAMPLES_PER_FRAME:
            raise ValueError("Silero VAD sample_rate must be 8000 or 16000")
        self._reset_state(sample_rate)

    def speech_probability(self, pcm_frame: bytes, sample_rate: int) -> float:
        if sample_rate not in self._SAMPLES_PER_FRAME:
            raise ValueError("Silero VAD sample_rate must be 8000 or 16000")
        expected_samples = self._SAMPLES_PER_FRAME[sample_rate]
        audio = self._np.frombuffer(pcm_frame, dtype="<i2")
        if audio.size != expected_samples:
            raise ValueError(
                "Silero VAD requires exactly "
                f"{expected_samples} samples per frame at {sample_rate}Hz"
            )
        if self._last_sample_rate != sample_rate:
            self._reset_state(sample_rate)

        normalized = audio.astype(self._np.float32).reshape(1, -1) / 32768.0
        model_input = self._np.concatenate((self._context, normalized), axis=1)
        outputs = self._session.run(
            None,
            {
                "input": model_input,
                "state": self._state,
                "sr": self._np.array(sample_rate, dtype=self._np.int64),
            },
        )
        if len(outputs) < 2:
            raise RuntimeError("Silero ONNX model must return probability and state")
        probability, self._state = outputs[0], self._np.asarray(
            outputs[1], dtype=self._np.float32
        )
        context_size = self._CONTEXT_SAMPLES[sample_rate]
        self._context = model_input[:, -context_size:].copy()
        return float(self._np.asarray(probability).reshape(-1)[0])


class StreamingVadEndpoint:
    """把帧级 VAD 概率转换成稳定的端点事件。

    公开接口只有 process_pcm(bytes)，它可以接收任意大小 PCM chunk；内部按固定
    frame_ms 聚合后调用 provider。短噪声在起点确认阶段直接丢弃；已经发出的
    speech_started 始终会得到配对的 speech_ended，避免 Agent 状态悬挂。
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
        speech_start_ms: float = 0.0,
        speech_end_threshold: float | None = None,
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
        if speech_start_ms < 0.0:
            raise ValueError("speech_start_ms must be non-negative")
        if (
            speech_end_threshold is not None
            and not 0.0 <= speech_end_threshold <= threshold
        ):
            raise ValueError("speech_end_threshold must be in [0, threshold]")

        self._provider = probability_provider
        self._sample_rate = sample_rate
        self._frame_seconds = frame_ms / 1000.0
        self._frame_bytes = int(sample_rate * self._frame_seconds) * 2
        if self._frame_bytes <= 0:
            raise ValueError("frame_ms is too small for sample_rate")
        self._threshold = threshold
        # 起点需要连续多帧达到阈值，避免键盘声或碰麦克风让 Agent 卡在 speech_detected。
        # 进入语音段后使用更低的结束阈值形成滞回，降低概率在临界值附近抖动造成的断句。
        # 起点确认至少覆盖 min_utterance：一旦向下游发布 speech_started，后续必须保证
        # 会发布配对的 speech_ended，不能在结束阶段再以“太短”为由丢弃事件。
        self._speech_start_s = max(speech_start_ms, min_utterance_ms) / 1000.0
        self._speech_end_threshold = (
            threshold if speech_end_threshold is None else speech_end_threshold
        )
        self._end_silence_s = speech_end_silence_s
        self._max_utterance_s = max_utterance_s
        self._buffer = bytearray()
        self._in_utterance = False
        self._pending_speech_s = 0.0
        self._pending_probability = 0.0
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
            events.extend(self._update(probability))
        return events

    def reset(self) -> None:
        self._buffer.clear()
        self._in_utterance = False
        self._pending_speech_s = 0.0
        self._pending_probability = 0.0
        self._speech_s = 0.0
        self._silence_s = 0.0

    def _update(self, probability: float) -> list[VadEvent]:
        events: list[VadEvent] = []
        if not self._in_utterance:
            if probability < self._threshold:
                self._pending_speech_s = 0.0
                self._pending_probability = 0.0
                return events

            self._pending_speech_s += self._frame_seconds
            self._pending_probability = max(self._pending_probability, probability)
            if self._pending_speech_s + 1e-9 < self._speech_start_s:
                return events

            self._in_utterance = True
            self._speech_s = self._pending_speech_s
            self._pending_speech_s = 0.0
            start_probability = self._pending_probability
            self._pending_probability = 0.0
            events.append(
                VadEvent(VadEventName.SPEECH_STARTED, probability=start_probability)
            )
            if self._speech_s + 1e-9 >= self._max_utterance_s:
                events.extend(self._finish("max_duration", probability))
            return events

        if probability >= self._speech_end_threshold:
            self._speech_s += self._frame_seconds
            self._silence_s = 0.0
            if self._speech_s + 1e-9 >= self._max_utterance_s:
                events.extend(self._finish("max_duration", probability))
            return events

        self._silence_s += self._frame_seconds
        if self._silence_s + 1e-9 >= self._end_silence_s:
            events.extend(self._finish("silence", probability))
        return events

    def _finish(self, reason: str, probability: float) -> list[VadEvent]:
        self._in_utterance = False
        self._pending_speech_s = 0.0
        self._pending_probability = 0.0
        self._speech_s = 0.0
        self._silence_s = 0.0
        return [VadEvent(VadEventName.SPEECH_ENDED, reason=reason, probability=probability)]


class SileroVadProvider:
    """Silero provider facade：默认走轻量 ONNX，JIT 模式才使用官方 Python 包。

    依赖是可选的：默认 CI 不下载模型；连续语音脚本的 VAD_PROVIDER=auto 会在模型和
    onnxruntime 都可用时才切到 Silero。这样端侧部署只增加约 2.2MiB 模型和 ONNX Runtime，
    不需要为了 VAD 安装完整 PyTorch。
    """

    def __init__(
        self,
        *,
        use_onnx: bool = True,
        model_path: str = "",
        session_factory: Callable[[str], object] | None = None,
    ):
        self._onnx_provider = None
        if use_onnx:
            if not model_path:
                raise SileroVadUnavailableError(
                    "Silero ONNX model_path 为空。请执行："
                    "bash scripts/setup_voice_vad_runtime.sh silero"
                )
            self._onnx_provider = SileroOnnxVadProvider(
                model_path=model_path,
                session_factory=session_factory,
            )
            return

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
        kwargs = {"onnx": False}
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
        if self._onnx_provider is not None:
            return self._onnx_provider.speech_probability(pcm_frame, sample_rate)
        audio = self._np.frombuffer(pcm_frame, dtype="<i2").astype("float32") / 32768.0
        tensor = self._torch.from_numpy(audio)
        with self._torch.no_grad():
            probability = self._model(tensor, sample_rate)
        if hasattr(probability, "item"):
            return float(probability.item())
        return float(probability)


class WebRtcVadProvider:
    """py-webrtcvad adapter。

    WebRTC VAD 是传统工程里非常常见的轻量端点检测方案：CPU 开销低、依赖小，
    但只能输出二分类 speech/non-speech，并要求输入帧必须是 10/20/30ms PCM16。
    因此这里把 True/False 映射成 1.0/0.0，复用 StreamingVadEndpoint 的滞回和
    silence timeout 逻辑。
    """

    _SUPPORTED_SAMPLE_RATES = {8000, 16000, 32000, 48000}
    _SUPPORTED_FRAME_MS = {10, 20, 30}

    def __init__(self, *, aggressiveness: int = 2):
        if aggressiveness < 0 or aggressiveness > 3:
            raise ValueError("WebRTC VAD aggressiveness must be in [0, 3]")
        try:
            import webrtcvad
        except Exception as error:  # pragma: no cover - 依赖缺失时只在真实节点启动路径触发
            raise WebRtcVadUnavailableError(
                "WebRTC VAD 依赖未安装。可执行：pip install webrtcvad。"
            ) from error
        self._vad = webrtcvad.Vad(int(aggressiveness))

    def speech_probability(self, pcm_frame: bytes, sample_rate: int) -> float:
        if sample_rate not in self._SUPPORTED_SAMPLE_RATES:
            raise ValueError(
                "WebRTC VAD sample_rate must be one of "
                f"{sorted(self._SUPPORTED_SAMPLE_RATES)}"
            )
        frame_ms = round(len(pcm_frame) / 2 / sample_rate * 1000)
        if frame_ms not in self._SUPPORTED_FRAME_MS:
            raise ValueError("WebRTC VAD frame_ms must be one of [10, 20, 30]")
        return 1.0 if self._vad.is_speech(pcm_frame, sample_rate) else 0.0
