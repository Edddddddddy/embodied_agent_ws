from pathlib import Path

import numpy as np
import sherpa_onnx


class SherpaVitsTts:
    """Offline VITS adapter; sentence-level calls provide pseudo-streaming."""

    def __init__(self, model_dir: str, num_threads: int, speaker_id: int, speed: float):
        root = Path(model_dir).expanduser()
        model_path = self._select_model(root)
        self._require_file(root / "lexicon.txt")
        self._require_file(root / "tokens.txt")
        # Sherpa-ONNX 的 VITS TTS 是句级离线合成：这里优先加载 fp32 模型，
        # 缺省时退回 int8 模型；上层再用双缓冲把“句级输出”包装成伪流式。
        rule_fsts = ",".join(
            str(root / name)
            for name in ("phone.fst", "date.fst", "number.fst")
            if (root / name).exists()
        )
        vits = sherpa_onnx.OfflineTtsVitsModelConfig(
            model=str(model_path),
            lexicon=str(root / "lexicon.txt"),
            tokens=str(root / "tokens.txt"),
        )
        model = sherpa_onnx.OfflineTtsModelConfig(
            vits=vits, num_threads=num_threads, debug=False, provider="cpu"
        )
        self._tts = sherpa_onnx.OfflineTts(
            sherpa_onnx.OfflineTtsConfig(model=model, rule_fsts=rule_fsts)
        )
        self.sample_rate = self._tts.sample_rate
        self._speaker_id = speaker_id
        self._speed = speed

    def synthesize(self, text: str) -> bytes:
        audio = self._tts.generate(text, sid=self._speaker_id, speed=self._speed)
        samples = np.clip(np.asarray(audio.samples) * 32767.0, -32768, 32767).astype("<i2")
        return samples.tobytes()

    @staticmethod
    def _select_model(root: Path) -> Path:
        for name in ("model.onnx", "model.int8.onnx"):
            path = root / name
            if path.is_file():
                return path
        raise FileNotFoundError(
            f"Sherpa TTS model not found under {root}; expected model.onnx or model.int8.onnx"
        )

    @staticmethod
    def _require_file(path: Path) -> None:
        if not path.is_file():
            raise FileNotFoundError(f"Sherpa TTS required file not found: {path}")
