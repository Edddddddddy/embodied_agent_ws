from __future__ import annotations

import subprocess
import tempfile
import wave
from pathlib import Path


class SummerTtsError(RuntimeError):
    """Raised when the SummerTTS command-line runtime cannot synthesize audio."""


class SummerTts:
    """SummerTTS command-line adapter.

    SummerTTS 本身是 C++ 离线 TTS 项目，当前最稳定的嵌入方式是调用它编译出的
    tts_test 二进制：输入文本文件 + 模型 bin，输出 16kHz mono PCM wav。这里把
    wav 头剥掉后返回 PCM16 bytes，让上层伪流式双缓冲继续复用同一套发布逻辑。
    """

    sample_rate = 16_000

    def __init__(
        self,
        binary_path: str,
        model_path: str,
        *,
        timeout_s: float = 30.0,
    ):
        self._binary_path = Path(binary_path).expanduser()
        self._model_path = Path(model_path).expanduser()
        self._timeout_s = timeout_s
        self._require_file(self._binary_path, "SummerTTS binary")
        self._require_file(self._model_path, "SummerTTS model")

    def synthesize(self, text: str) -> bytes:
        normalized = (text or "").strip()
        if not normalized:
            return b""
        with tempfile.TemporaryDirectory(prefix="summer_tts_") as tmp:
            text_path = Path(tmp) / "input.txt"
            wav_path = Path(tmp) / "output.wav"
            text_path.write_text(normalized, encoding="utf-8")
            result = subprocess.run(
                [
                    str(self._binary_path),
                    str(text_path),
                    str(self._model_path),
                    str(wav_path),
                ],
                cwd=str(self._binary_path.parent),
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                timeout=self._timeout_s,
                check=False,
            )
            if result.returncode != 0 or not wav_path.is_file():
                raise SummerTtsError(
                    "SummerTTS synthesis failed: "
                    f"exit={result.returncode}, stdout={result.stdout.strip()}, "
                    f"stderr={result.stderr.strip()}"
                )
            return self._read_pcm16_wav(wav_path)

    @classmethod
    def _read_pcm16_wav(cls, path: Path) -> bytes:
        with wave.open(str(path), "rb") as wav:
            channels = wav.getnchannels()
            sample_width = wav.getsampwidth()
            sample_rate = wav.getframerate()
            if channels != 1:
                raise SummerTtsError(f"{path} must be mono; got channels={channels}")
            if sample_width != 2:
                raise SummerTtsError(
                    f"{path} must be PCM16; got sample_width={sample_width}"
                )
            if sample_rate != cls.sample_rate:
                raise SummerTtsError(
                    f"{path} sample rate must be {cls.sample_rate}; got {sample_rate}"
                )
            return wav.readframes(wav.getnframes())

    @staticmethod
    def _require_file(path: Path, label: str) -> None:
        if not path.is_file():
            raise FileNotFoundError(f"{label} not found: {path}")
