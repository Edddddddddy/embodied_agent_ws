from pathlib import Path

import numpy as np
import sherpa_onnx


class SherpaVitsTts:
    """Offline VITS adapter; sentence-level calls provide pseudo-streaming."""

    def __init__(self, model_dir: str, num_threads: int, speaker_id: int, speed: float):
        root = Path(model_dir).expanduser()
        rule_fsts = ",".join(str(root / name) for name in ("phone.fst", "date.fst", "number.fst") if (root / name).exists())
        vits = sherpa_onnx.OfflineTtsVitsModelConfig(
            model=str(root / "model.onnx"),
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
