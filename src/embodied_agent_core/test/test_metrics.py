from embodied_agent_core.metrics import LatencyTracker
from embodied_agent_core.types import LatencySnapshot


def test_snapshot_returns_all_observed_streaming_latencies():
    timestamps = iter([0.0, 0.1, 0.3, 0.4, 0.65])
    tracker = LatencyTracker(clock=lambda: next(timestamps))

    tracker.mark_asr_final()
    tracker.mark_llm_requested()
    tracker.mark_llm_first_token()
    tracker.mark_tts_requested()
    tracker.mark_tts_first_audio()

    assert tracker.snapshot() == LatencySnapshot(
        llm_first_token_ms=200.0,
        asr_to_first_token_ms=300.0,
        tts_first_audio_ms=250.0,
    )


def test_snapshot_keeps_missing_measurements_explicit():
    assert LatencyTracker().snapshot() == LatencySnapshot(None, None, None)
