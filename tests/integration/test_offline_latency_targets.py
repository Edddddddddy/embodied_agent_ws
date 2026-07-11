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


def test_llm_runtime_summary_uses_warm_turn_p95_not_cold_warmup():
    report = latency.summarize_llm_samples(
        [
            {"first_token_ms": 95.0, "decode_tokens_per_s": 30.0},
            {"first_token_ms": 420.0, "decode_tokens_per_s": 28.0},
            {"first_token_ms": 390.0, "decode_tokens_per_s": 29.0},
        ],
        warmup={"first_token_ms": 3900.0},
        target_ms=1000.0,
    )

    assert report["measurement_kind"] == "warm_agent_turn_after_prefix_warmup"
    assert report["first_token_ms"] == 420.0
    assert report["median_first_token_ms"] == 390.0
    assert report["median_decode_tokens_per_s"] == 29.0
    assert report["warmup"]["first_token_ms"] == 3900.0
    assert report["ok"] is True
