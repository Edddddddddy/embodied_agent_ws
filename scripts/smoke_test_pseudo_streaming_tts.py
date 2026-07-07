#!/usr/bin/env python3
"""Dependency-free smoke for llama.cpp-style token stream + pseudo-streaming TTS."""

from __future__ import annotations

import json
import time

from embodied_offline_agent.pseudo_streaming_tts import PseudoStreamingTtsPipeline
from embodied_online_agent.protocol import SentenceChunker, TaggedStreamParser


def fake_llm_stream():
    chunks = [
        "<speech>好的，",
        "我马上执行。",
        "</speech><action>",
        '{"name":"move","arguments":{"linear_x":0.2,"duration_s":1.0}}',
        "</action>",
    ]
    for chunk in chunks:
        time.sleep(0.005)
        yield chunk


def main() -> int:
    published: list[bytes] = []

    def synthesize(text: str) -> bytes:
        # 80ms 左右的假 PCM，避免 smoke 依赖真实 Sherpa/SumerTTS 模型。
        time.sleep(0.01)
        return (text.encode("utf-8") or b"x") * 80

    pipeline = PseudoStreamingTtsPipeline(
        synthesize=synthesize,
        publish_audio=published.append,
        sample_rate=16000,
        pcm_chunk_ms=40,
    )
    pipeline.start()
    parser = TaggedStreamParser()
    chunker = SentenceChunker(max_chars=8)
    actions = []
    speech_parts = []
    for token in fake_llm_stream():
        events = parser.feed(token)
        for delta in events.speech:
            speech_parts.append(delta)
            for sentence in chunker.feed(delta):
                if not pipeline.put_text(sentence):
                    raise RuntimeError("failed to enqueue pseudo-streaming TTS text")
        actions.extend(events.actions)
    final = parser.finish()
    actions.extend(final.actions)
    for sentence in chunker.finish():
        pipeline.put_text(sentence)
    metrics = pipeline.close_and_wait(timeout_s=2.0)
    report = {
        "speech": "".join(speech_parts),
        "actions": [action.as_dict() for action in actions],
        "audio_chunks": len(published),
        "tts_pipeline": metrics.as_dict(),
        "status": "PASS",
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not published or not actions or metrics.first_text_to_first_audio_ms is None:
        raise SystemExit(1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
