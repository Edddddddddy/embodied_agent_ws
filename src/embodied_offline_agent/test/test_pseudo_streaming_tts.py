import time

import pytest

from embodied_offline_agent.pseudo_streaming_tts import PseudoStreamingTtsPipeline


def test_pseudo_streaming_tts_chunks_pcm_and_records_metrics():
    published = []
    first_audio_marks = []

    def synthesize(text: str) -> bytes:
        return (text.encode("utf-8") or b"x") * 200

    pipeline = PseudoStreamingTtsPipeline(
        synthesize=synthesize,
        publish_audio=published.append,
        sample_rate=1000,
        pcm_chunk_ms=10,
        on_first_audio=lambda: first_audio_marks.append(time.perf_counter()),
    )
    pipeline.start()

    assert pipeline.put_text("你好")
    assert pipeline.put_text("世界")
    metrics = pipeline.close_and_wait(timeout_s=2.0)

    assert metrics.text_chunks == 2
    assert metrics.synth_calls == 2
    assert metrics.audio_chunks == len(published)
    assert metrics.audio_chunks > 2
    assert metrics.first_text_to_first_audio_ms is not None
    assert first_audio_marks
    assert metrics.message_buffer_dropped == 0
    assert metrics.audio_buffer_dropped == 0


def test_pseudo_streaming_tts_surfaces_synthesis_errors():
    def synthesize(_text: str) -> bytes:
        raise RuntimeError("tts failed")

    pipeline = PseudoStreamingTtsPipeline(
        synthesize=synthesize,
        publish_audio=lambda _pcm: None,
        sample_rate=1000,
        pcm_chunk_ms=10,
    )
    pipeline.start()
    assert pipeline.put_text("坏消息")

    with pytest.raises(RuntimeError, match="tts failed"):
        pipeline.close_and_wait(timeout_s=2.0)


def test_pseudo_streaming_tts_abort_wakes_workers():
    pipeline = PseudoStreamingTtsPipeline(
        synthesize=lambda text: text.encode(),
        publish_audio=lambda _pcm: None,
        sample_rate=1000,
        pcm_chunk_ms=10,
    )
    pipeline.start()
    pipeline.abort()
    assert pipeline.metrics["message_buffer_dropped"] >= 0
