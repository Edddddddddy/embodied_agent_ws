import json
import sys
import types

from embodied_online_agent.keyword_wake import (
    KeywordWakeBridge,
    SherpaKeywordWakeDetector,
    TextKeywordWakeDetector,
)


def test_text_keyword_detector_matches_aliases():
    detector = TextKeywordWakeDetector(
        ["小智"], aliases=["小志"], provider_name="mock_kws"
    )

    match = detector.detect_text("你好小志")

    assert match is not None
    assert match.keyword == "小志"
    assert match.provider == "mock_kws"
    assert detector.detect_text("向前走一秒") is None


def test_keyword_bridge_outputs_standard_wake_event_and_applies_cooldown():
    now = [10.0]
    detector = TextKeywordWakeDetector(["小智"], provider_name="test_kws")
    bridge = KeywordWakeBridge(cooldown_s=1.0, clock=lambda: now[0])

    first = bridge.wake_payload(detector.detect_text("小智"))
    second = bridge.wake_payload(detector.detect_text("小智"))
    now[0] += 1.1
    third = bridge.wake_payload(detector.detect_text("小智"))

    assert first is not None
    payload = json.loads(first)
    assert payload["kind"] == "wake"
    assert payload["provider"] == "test_kws"
    assert payload["transcript"] == "小智"
    assert second is None
    assert third is not None


def test_sherpa_keyword_detector_uses_keyword_spotter_api(monkeypatch):
    calls = {"reset": 0, "decoded": 0}

    class FakeStream:
        def __init__(self):
            self.accepted = False

        def accept_waveform(self, sample_rate, samples):
            assert sample_rate == 16000
            assert len(samples) == 2
            self.accepted = True

    class FakeSpotter:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.stream = FakeStream()

        def create_stream(self):
            return self.stream

        def is_ready(self, stream):
            return stream.accepted and calls["decoded"] == 0

        def decode_stream(self, _stream):
            calls["decoded"] += 1

        def get_result(self, _stream):
            return "小智" if calls["decoded"] else ""

        def reset_stream(self, _stream):
            calls["reset"] += 1

    fake_sherpa = types.SimpleNamespace(KeywordSpotter=FakeSpotter)
    monkeypatch.setitem(sys.modules, "sherpa_onnx", fake_sherpa)

    detector = SherpaKeywordWakeDetector(
        tokens="tokens.txt",
        encoder="encoder.onnx",
        decoder="decoder.onnx",
        joiner="joiner.onnx",
        keywords_file="keywords.txt",
        provider_name="sherpa_kws",
    )

    match = detector.detect_audio(b"\x01\x00\x02\x00")

    assert match is not None
    assert match.keyword == "小智"
    assert match.provider == "sherpa_kws"
    assert calls == {"reset": 1, "decoded": 1}
