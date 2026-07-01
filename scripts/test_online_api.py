#!/usr/bin/env python3
"""Minimal-token live test for Qwen LLM, realtime TTS, and realtime ASR."""

import argparse
import json
import threading
import time

import numpy as np

from embodied_online_agent.providers.openai_compatible_llm import OpenAiCompatibleLlm
from embodied_online_agent.providers.qwen_asr import QwenRealtimeAsr
from embodied_online_agent.providers.qwen_tts import QwenRealtimeTts


def resample_pcm16(pcm: bytes, source_rate: int, target_rate: int) -> bytes:
    source = np.frombuffer(pcm, dtype="<i2").astype(np.float32)
    if source.size == 0:
        return b""
    target_size = max(1, round(source.size * target_rate / source_rate))
    positions = np.linspace(0, source.size - 1, target_size)
    samples = np.interp(positions, np.arange(source.size), source)
    return np.clip(samples, -32768, 32767).astype("<i2").tobytes()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--enforce-targets", action="store_true")
    args = parser.parse_args()
    report = {}

    llm = OpenAiCompatibleLlm()
    llm_messages = [
        {"role": "system", "content": "你正在做连通性测试。只回复OK，不要解释。"},
        {"role": "user", "content": "回复OK"},
    ]

    def measure_llm():
        started = time.perf_counter()
        first_token_at = None
        text = []
        for token in llm.stream(llm_messages):
            if first_token_at is None:
                first_token_at = time.perf_counter()
            text.append(token)
        if first_token_at is None or not "OK" in "".join(text).upper():
            raise RuntimeError("online LLM returned no usable response")
        return round((first_token_at - started) * 1000, 2)

    report["llm_cold_first_token_ms"] = measure_llm()
    report["llm_first_token_ms"] = measure_llm()

    audio_parts = []
    first_audio_at = None
    tts = QwenRealtimeTts()
    connect_started = time.perf_counter()
    tts.connect()
    report["tts_connect_ms"] = round((time.perf_counter() - connect_started) * 1000, 2)
    tts_started = time.perf_counter()

    def on_audio(chunk):
        nonlocal first_audio_at
        if first_audio_at is None:
            first_audio_at = time.perf_counter()
        audio_parts.append(chunk)

    tts.synthesize(["小智停止"], on_audio)
    if first_audio_at is None or not audio_parts:
        raise RuntimeError("online TTS returned no PCM audio")
    report["tts_first_audio_ms"] = round((first_audio_at - tts_started) * 1000, 2)
    report["tts_pcm_bytes"] = sum(map(len, audio_parts))

    final_event = threading.Event()
    final_text = []
    tts.close()
    asr = QwenRealtimeAsr()
    asr.start(lambda _text: None, lambda value: (final_text.append(value), final_event.set()))
    try:
        time.sleep(0.8)
        pcm16 = resample_pcm16(b"".join(audio_parts), 24000, 16000)
        pcm16 += bytes(16000)  # 0.5 s trailing silence at 16-kHz PCM16.
        for offset in range(0, len(pcm16), 3200):
            asr.push_audio(pcm16[offset : offset + 3200])
            time.sleep(0.02)
        commit_at = time.perf_counter()
        asr.commit()
        if not final_event.wait(12.0):
            raise TimeoutError("online ASR produced no final transcript")
        report["asr_finalize_ms"] = round((time.perf_counter() - commit_at) * 1000, 2)
        report["asr_text"] = final_text[-1]
    finally:
        asr.stop()

    report["llm_target_met"] = report["llm_first_token_ms"] < 1000.0
    report["tts_target_met"] = report["tts_first_audio_ms"] < 300.0
    report["asr_target_met"] = report["asr_finalize_ms"] < 600.0
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.enforce_targets and not all(
        report[key] for key in ("llm_target_met", "tts_target_met", "asr_target_met")
    ):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
