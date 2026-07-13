import pytest

from embodied_voice_frontend.silero_vad_sidecar import (
    SileroOnnxVadProvider,
    SileroVadProvider,
    StreamingVadEndpoint,
    VadEventName,
    WebRtcVadProvider,
)


class ScriptedProbabilityProvider:
    """测试用概率提供器：每处理一帧就返回一个预设概率。"""

    def __init__(self, probabilities):
        self._probabilities = list(probabilities)

    def speech_probability(self, _pcm_frame: bytes, _sample_rate: int) -> float:
        if not self._probabilities:
            return 0.0
        return self._probabilities.pop(0)


def frame(sample_count: int = 160) -> bytes:
    return b"\x01\x00" * sample_count


def test_silero_onnx_provider_runs_stateful_32ms_frames_without_torch(tmp_path):
    import numpy as np

    model = tmp_path / "silero_vad.onnx"
    model.write_bytes(b"onnx-stub")

    class FakeSession:
        def __init__(self):
            self.calls = []

        def run(self, _outputs, inputs):
            self.calls.append(inputs)
            return [
                np.array([[0.75]], dtype=np.float32),
                np.ones((2, 1, 128), dtype=np.float32),
            ]

    session = FakeSession()
    provider = SileroOnnxVadProvider(
        model_path=str(model), session_factory=lambda _path: session
    )

    probability = provider.speech_probability(frame(sample_count=512), 16000)
    provider.speech_probability(b"\x02\x00" * 512, 16000)

    assert probability == pytest.approx(0.75)
    assert session.calls[0]["input"].shape == (1, 576)
    assert session.calls[0]["state"].shape == (2, 1, 128)
    assert int(session.calls[0]["sr"]) == 16000
    assert np.all(
        session.calls[1]["input"][0, :64]
        == session.calls[0]["input"][0, -64:]
    )


def test_silero_provider_uses_lightweight_onnx_path_without_torch(tmp_path):
    import numpy as np

    model = tmp_path / "silero_vad.onnx"
    model.write_bytes(b"onnx-stub")

    class FakeSession:
        def run(self, _outputs, inputs):
            return [
                np.array([[0.6]], dtype=np.float32),
                inputs["state"],
            ]

    provider = SileroVadProvider(
        use_onnx=True,
        model_path=str(model),
        session_factory=lambda _path: FakeSession(),
    )

    assert provider.speech_probability(frame(sample_count=512), 16000) == pytest.approx(
        0.6
    )


def test_silero_onnx_provider_rejects_non_32ms_frame(tmp_path):
    model = tmp_path / "silero_vad.onnx"
    model.write_bytes(b"onnx-stub")

    class FakeSession:
        def run(self, _outputs, _inputs):
            raise AssertionError("invalid frame must fail before inference")

    provider = SileroOnnxVadProvider(
        model_path=str(model), session_factory=lambda _path: FakeSession()
    )

    with pytest.raises(ValueError, match="512"):
        provider.speech_probability(frame(sample_count=320), 16000)


def test_streaming_endpoint_emits_start_and_end_for_complete_utterance():
    endpoint = StreamingVadEndpoint(
        ScriptedProbabilityProvider([0.8, 0.8, 0.1, 0.1]),
        sample_rate=16000,
        frame_ms=10,
        threshold=0.5,
        speech_end_silence_s=0.02,
        min_utterance_ms=20.0,
        max_utterance_s=5.0,
    )

    names = []
    for _ in range(4):
        names.extend(event.name for event in endpoint.process_pcm(frame()))

    assert names == [VadEventName.SPEECH_STARTED, VadEventName.SPEECH_ENDED]


def test_streaming_endpoint_drops_short_noise_without_unpaired_start_event():
    endpoint = StreamingVadEndpoint(
        ScriptedProbabilityProvider([0.9, 0.1, 0.1]),
        sample_rate=16000,
        frame_ms=10,
        threshold=0.5,
        speech_end_silence_s=0.02,
        min_utterance_ms=30.0,
        max_utterance_s=5.0,
    )

    names = []
    for _ in range(3):
        names.extend(event.name for event in endpoint.process_pcm(frame()))

    assert names == []


def test_streaming_endpoint_debounces_single_frame_noise_before_start():
    endpoint = StreamingVadEndpoint(
        ScriptedProbabilityProvider([0.9, 0.1, 0.1]),
        sample_rate=16000,
        frame_ms=10,
        threshold=0.5,
        speech_start_ms=20.0,
        speech_end_silence_s=0.02,
        min_utterance_ms=20.0,
        max_utterance_s=5.0,
    )

    events = []
    for _ in range(3):
        events.extend(endpoint.process_pcm(frame()))

    assert events == []


def test_streaming_endpoint_confirms_start_after_consecutive_speech_frames():
    endpoint = StreamingVadEndpoint(
        ScriptedProbabilityProvider([0.9, 0.8, 0.1, 0.1]),
        sample_rate=16000,
        frame_ms=10,
        threshold=0.5,
        speech_start_ms=20.0,
        speech_end_silence_s=0.02,
        min_utterance_ms=20.0,
        max_utterance_s=5.0,
    )

    names = []
    for _ in range(4):
        names.extend(event.name for event in endpoint.process_pcm(frame()))

    assert names == [VadEventName.SPEECH_STARTED, VadEventName.SPEECH_ENDED]


def test_streaming_endpoint_uses_lower_end_threshold_to_avoid_probability_flapping():
    endpoint = StreamingVadEndpoint(
        ScriptedProbabilityProvider([0.8, 0.8, 0.4, 0.4, 0.1, 0.1]),
        sample_rate=16000,
        frame_ms=10,
        threshold=0.5,
        speech_end_threshold=0.35,
        speech_end_silence_s=0.02,
        min_utterance_ms=20.0,
        max_utterance_s=5.0,
    )

    names = []
    for _ in range(6):
        names.extend(event.name for event in endpoint.process_pcm(frame()))

    assert names == [VadEventName.SPEECH_STARTED, VadEventName.SPEECH_ENDED]


def test_streaming_endpoint_forces_end_at_maximum_duration():
    endpoint = StreamingVadEndpoint(
        ScriptedProbabilityProvider([0.9, 0.9, 0.9]),
        sample_rate=16000,
        frame_ms=10,
        threshold=0.5,
        speech_end_silence_s=0.4,
        min_utterance_ms=0.0,
        max_utterance_s=0.03,
    )

    names = []
    for _ in range(3):
        names.extend(event.name for event in endpoint.process_pcm(frame()))

    assert names == [VadEventName.SPEECH_STARTED, VadEventName.SPEECH_ENDED]


def test_streaming_endpoint_rejects_invalid_frame_configuration():
    with pytest.raises(ValueError, match="frame_ms"):
        StreamingVadEndpoint(
            ScriptedProbabilityProvider([]),
            sample_rate=16000,
            frame_ms=0,
            threshold=0.5,
            speech_end_silence_s=0.4,
            min_utterance_ms=100.0,
            max_utterance_s=12.0,
        )


def test_webrtc_vad_provider_maps_binary_decision_to_probability(monkeypatch):
    class FakeVad:
        def __init__(self, aggressiveness):
            self.aggressiveness = aggressiveness

        def is_speech(self, pcm_frame, sample_rate):
            return sample_rate == 16000 and any(pcm_frame)

    class FakeWebRtcVadModule:
        Vad = FakeVad

    monkeypatch.setitem(__import__("sys").modules, "webrtcvad", FakeWebRtcVadModule())
    provider = WebRtcVadProvider(aggressiveness=2)

    assert provider.speech_probability(frame(sample_count=320), 16000) == 1.0
    assert provider.speech_probability(b"\x00\x00" * 320, 16000) == 0.0


def test_webrtc_vad_provider_requires_supported_frame_duration(monkeypatch):
    class FakeVad:
        def __init__(self, _aggressiveness):
            pass

        def is_speech(self, _pcm_frame, _sample_rate):
            return False

    class FakeWebRtcVadModule:
        Vad = FakeVad

    monkeypatch.setitem(__import__("sys").modules, "webrtcvad", FakeWebRtcVadModule())
    provider = WebRtcVadProvider()

    with pytest.raises(ValueError, match="frame_ms"):
        provider.speech_probability(frame(sample_count=240), 16000)
