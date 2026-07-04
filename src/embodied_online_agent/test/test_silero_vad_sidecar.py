import pytest

from embodied_online_agent.silero_vad_sidecar import (
    StreamingVadEndpoint,
    VadEventName,
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


def test_streaming_endpoint_drops_short_noise_without_end_event():
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

    assert names == [VadEventName.SPEECH_STARTED]


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
