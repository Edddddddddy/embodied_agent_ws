import sys
import types

from embodied_online_agent.keyword_wake import (
    KeywordWakeBridge,
    LiveKitWakeWordDetector,
    OpenWakeWordDetector,
    SherpaKeywordWakeDetector,
    TextKeywordWakeDetector,
)
from embodied_online_agent.keyword_wake_node import _string_list_param


def test_text_keyword_detector_matches_aliases():
    detector = TextKeywordWakeDetector(
        ["小智"], aliases=["小志"], provider_name="mock_kws"
    )

    match = detector.detect_text("你好小志")

    assert match is not None
    assert match.keyword == "小志"
    assert match.provider == "mock_kws"
    assert detector.detect_text("向前走一秒") is None


def test_keyword_wake_node_accepts_comma_separated_model_paths():
    assert _string_list_param("xiaozhi.onnx, nihaoxiaozhi.onnx") == [
        "xiaozhi.onnx",
        "nihaoxiaozhi.onnx",
    ]
    assert _string_list_param(["xiaozhi.onnx", ""]) == ["xiaozhi.onnx"]
    assert _string_list_param("") == []


def test_keyword_bridge_outputs_standard_wake_event_and_applies_cooldown():
    now = [10.0]
    detector = TextKeywordWakeDetector(["小智"], provider_name="test_kws")
    bridge = KeywordWakeBridge(cooldown_s=1.0, clock=lambda: now[0])

    first = bridge.accept(detector.detect_text("小智"))
    second = bridge.accept(detector.detect_text("小智"))
    now[0] += 1.1
    third = bridge.accept(detector.detect_text("小智"))

    assert first is not None
    assert first.provider == "test_kws"
    assert first.keyword == "小智"
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


def test_openwakeword_detector_reports_highest_score_above_threshold(monkeypatch):
    class FakeModel:
        def __init__(self, **kwargs):
            self.kwargs = kwargs
            self.last_audio = None

        def predict(self, audio):
            self.last_audio = audio
            assert audio.dtype.name == "int16"
            assert audio.tolist() == [1, 2]
            return {"小智": 0.42, "hey_jarvis": 0.81}

    fake_openwakeword_model = types.SimpleNamespace(Model=FakeModel)
    monkeypatch.setitem(
        sys.modules, "openwakeword.model", fake_openwakeword_model
    )

    detector = OpenWakeWordDetector(
        model_paths=["xiaozhi.onnx"],
        threshold=0.5,
        inference_framework="onnx",
        provider_name="openwakeword",
    )

    match = detector.detect_audio(b"\x01\x00\x02\x00")

    assert match is not None
    assert match.keyword == "hey_jarvis"
    assert match.provider == "openwakeword"
    assert match.score == 0.81
    assert detector.last_scores() == {"小智": 0.42, "hey_jarvis": 0.81}


def test_openwakeword_detector_ignores_scores_below_threshold(monkeypatch):
    class FakeModel:
        def __init__(self, **_kwargs):
            pass

        def predict(self, _audio):
            return {"小智": 0.49}

    monkeypatch.setitem(
        sys.modules, "openwakeword.model", types.SimpleNamespace(Model=FakeModel)
    )

    detector = OpenWakeWordDetector(
        model_paths=["xiaozhi.onnx"],
        threshold=0.5,
        provider_name="openwakeword",
    )

    assert detector.detect_audio(b"\x01\x00\x02\x00") is None
    assert detector.last_scores() == {"小智": 0.49}


def test_livekit_wakeword_detector_uses_wakeword_model_api(monkeypatch):
    class FakeWakeWordModel:
        def __init__(self, **kwargs):
            assert kwargs == {"models": ["xiaozhi.onnx"]}

        def predict(self, audio):
            assert audio.dtype.name == "int16"
            assert audio.tolist() == [1, 2]
            return {"xiaozhi": 0.88, "background": 0.02}

    monkeypatch.setitem(
        sys.modules,
        "livekit.wakeword",
        types.SimpleNamespace(WakeWordModel=FakeWakeWordModel),
    )

    detector = LiveKitWakeWordDetector(
        model_paths=["xiaozhi.onnx"],
        threshold=0.5,
        provider_name="livekit_wakeword",
    )

    match = detector.detect_audio(b"\x01\x00\x02\x00")

    assert match is not None
    assert match.keyword == "xiaozhi"
    assert match.provider == "livekit_wakeword"
    assert match.score == 0.88
    assert detector.last_scores() == {"xiaozhi": 0.88, "background": 0.02}


def test_livekit_wakeword_detector_ignores_low_scores(monkeypatch):
    class FakeWakeWordModel:
        def __init__(self, **_kwargs):
            pass

        def predict(self, _audio):
            return {"xiaozhi": 0.49}

    monkeypatch.setitem(
        sys.modules,
        "livekit.wakeword",
        types.SimpleNamespace(WakeWordModel=FakeWakeWordModel),
    )

    detector = LiveKitWakeWordDetector(
        model_paths=["xiaozhi.onnx"],
        threshold=0.5,
        provider_name="livekit_wakeword",
    )

    assert detector.detect_audio(b"\x01\x00\x02\x00") is None
    assert detector.last_scores() == {"xiaozhi": 0.49}
