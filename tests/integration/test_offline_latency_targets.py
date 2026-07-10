import importlib.util
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "offline_latency_targets.py"
spec = importlib.util.spec_from_file_location("offline_latency_targets", SCRIPT)
latency = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = latency
spec.loader.exec_module(latency)


def test_non_streaming_tts_is_reported_as_full_synthesis_not_first_audio(monkeypatch):
    class FakeTts:
        sample_rate = 24000

        def synthesize(self, _text):
            return b"\x00\x00" * 240

    monkeypatch.setattr(latency, "create_tts", lambda _provider: FakeTts())

    report = latency.measure_tts("sherpa", "好的。", warmup=False)

    assert report["measurement_kind"] == "full_utterance_synthesis"
    assert report["first_audio_supported"] is False
    assert report["synthesis_ms"] >= 0.0
    assert "first_audio_ms" not in report
