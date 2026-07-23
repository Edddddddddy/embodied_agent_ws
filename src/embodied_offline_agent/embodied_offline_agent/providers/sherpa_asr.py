from pathlib import Path

import numpy as np
import sherpa_onnx


class SherpaZipformerAsr:
    """Streaming ZipFormer transducer. All methods must run on one worker thread."""

    def __init__(
        self, model_dir: str, sample_rate: int, num_threads: int,
        decoding_method: str = "modified_beam_search",
        hotwords_file: str = "", hotwords_score: float = 3.0,
        max_active_paths: int = 16, modeling_unit: str = "cjkchar",
        tail_padding_s: float = 0.66,
    ):
        root = Path(model_dir).expanduser()
        self._sample_rate = sample_rate
        # Streaming ZipFormer 需要少量尾部静音把编码器剩余上下文推出。这里在
        # 构造期预分配，避免每个 utterance 提交时重复创建 numpy 数组。
        self._tail_padding = np.zeros(
            round(sample_rate * tail_padding_s), dtype=np.float32
        )
        self._recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
            tokens=str(root / "tokens.txt"),
            encoder=str(root / "encoder-epoch-99-avg-1.int8.onnx"),
            decoder=str(root / "decoder-epoch-99-avg-1.int8.onnx"),
            joiner=str(root / "joiner-epoch-99-avg-1.int8.onnx"),
            num_threads=num_threads,
            sample_rate=sample_rate,
            feature_dim=80,
            decoding_method=decoding_method,
            max_active_paths=max_active_paths,
            hotwords_file=hotwords_file,
            hotwords_score=hotwords_score,
            modeling_unit=modeling_unit,
            enable_endpoint_detection=False,
            provider="cpu",
        )
        self._stream = self._recognizer.create_stream()
        self._on_partial = None
        self._on_final = None
        self._last_partial = ""

    def start(self, on_partial, on_final):
        self._on_partial, self._on_final = on_partial, on_final

    def push_audio(self, pcm16: bytes):
        samples = np.frombuffer(pcm16, dtype="<i2").astype(np.float32) / 32768.0
        self._stream.accept_waveform(self._sample_rate, samples)
        self._decode_ready()
        text = self._text()
        if text and text != self._last_partial:
            self._last_partial = text
            self._on_partial(text)

    def commit(self):
        if self._tail_padding.size:
            self._stream.accept_waveform(self._sample_rate, self._tail_padding)
        self._stream.input_finished()
        self._decode_ready()
        text = self._text().strip()
        if text:
            self._on_final(text)
        self._stream = self._recognizer.create_stream()
        self._last_partial = ""

    def reset(self):
        """Lifecycle 再激活时丢弃停用前的半句音频，防止跨会话拼接。"""

        self._stream = self._recognizer.create_stream()
        self._last_partial = ""

    def _decode_ready(self):
        while self._recognizer.is_ready(self._stream):
            self._recognizer.decode_stream(self._stream)

    def _text(self) -> str:
        result = self._recognizer.get_result(self._stream)
        return result.text if hasattr(result, "text") else str(result)
