import os
import stat
import textwrap
import wave
from pathlib import Path

import pytest

from embodied_offline_agent.providers.summer_tts import SummerTts, SummerTtsError


def _write_fake_tts_binary(path: Path, *, fail: bool = False) -> None:
    body = """#!/usr/bin/env python3
import math
import struct
import sys
import wave

if {fail}:
    print("fake SummerTTS failed", file=sys.stderr)
    raise SystemExit(3)

text_path, model_path, wav_path = sys.argv[1:4]
text = open(text_path, encoding="utf-8").read().strip()
if not text:
    raise SystemExit(2)
sample_rate = 16000
samples = [
    int(600 * math.sin(2.0 * math.pi * 440.0 * i / sample_rate))
    for i in range(0, sample_rate // 20)
]
with wave.open(wav_path, "wb") as wav:
    wav.setnchannels(1)
    wav.setsampwidth(2)
    wav.setframerate(sample_rate)
    wav.writeframes(struct.pack("<" + "h" * len(samples), *samples))
"""
    path.write_text(textwrap.dedent(body.format(fail="True" if fail else "False")), encoding="utf-8")
    path.chmod(path.stat().st_mode | stat.S_IXUSR)


def test_summer_tts_invokes_binary_and_returns_pcm16(tmp_path):
    binary = tmp_path / "tts_test"
    model = tmp_path / "single_speaker_fast.bin"
    _write_fake_tts_binary(binary)
    model.write_bytes(os.urandom(32))

    provider = SummerTts(str(binary), str(model), timeout_s=2.0)
    pcm = provider.synthesize("你好")

    assert provider.sample_rate == 16000
    assert len(pcm) > 0
    assert len(pcm) % 2 == 0


def test_summer_tts_surfaces_binary_failure(tmp_path):
    binary = tmp_path / "tts_test"
    model = tmp_path / "single_speaker_fast.bin"
    _write_fake_tts_binary(binary, fail=True)
    model.write_bytes(os.urandom(32))

    provider = SummerTts(str(binary), str(model), timeout_s=2.0)

    with pytest.raises(SummerTtsError, match="exit=3"):
        provider.synthesize("你好")


def test_summer_tts_validates_runtime_files(tmp_path):
    with pytest.raises(FileNotFoundError, match="SummerTTS binary"):
        SummerTts(str(tmp_path / "missing_binary"), str(tmp_path / "missing_model"))
