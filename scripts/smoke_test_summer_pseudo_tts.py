#!/usr/bin/env python3
"""SummerTTS + 项目伪流式双缓冲 smoke。"""

from __future__ import annotations

import json
import sys
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
for relative in ("src/embodied_offline_agent", "src/embodied_online_agent"):
    path = str(WORKSPACE / relative)
    if path not in sys.path:
        sys.path.insert(0, path)

from embodied_offline_agent.providers.summer_tts import SummerTts
from embodied_offline_agent.pseudo_streaming_tts import PseudoStreamingTtsPipeline


def main() -> int:
    provider = SummerTts(
        str(WORKSPACE / "third_party" / "SummerTTS" / "build" / "tts_test"),
        str(WORKSPACE / "third_party" / "SummerTTS" / "models" / "single_speaker_fast.bin"),
        timeout_s=30.0,
    )
    published: list[bytes] = []
    pipeline = PseudoStreamingTtsPipeline(
        synthesize=provider.synthesize,
        publish_audio=published.append,
        sample_rate=provider.sample_rate,
        pcm_chunk_ms=80,
    )
    pipeline.start()
    for sentence in ("好的。", "开始执行。"):
        if not pipeline.put_text(sentence):
            raise RuntimeError("failed to enqueue SummerTTS text")
    metrics = pipeline.close_and_wait(timeout_s=90.0)
    report = {
        "status": "PASS",
        "audio_chunks": len(published),
        "sample_rate": provider.sample_rate,
        "tts_pipeline": metrics.as_dict(),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if metrics.synth_calls != 2 or not published:
        raise SystemExit(1)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
