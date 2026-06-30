#!/usr/bin/env python3
import argparse
import json
import time
import wave
from pathlib import Path

from embodied_offline_agent.providers.sherpa_asr import SherpaZipformerAsr
from embodied_offline_agent.providers.sherpa_tts import SherpaVitsTts


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default="/home/ubuntu/embodied_agent_ws")
    parser.add_argument("--wav", help="16-kHz mono PCM WAV; defaults to the bundled ASR sample")
    args = parser.parse_args()
    root = Path(args.workspace)
    asr_dir = root / "models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16"
    wav_path = Path(args.wav) if args.wav else next((asr_dir / "test_wavs").glob("*.wav"))
    with wave.open(str(wav_path), "rb") as wav:
        assert wav.getframerate() == 16000 and wav.getnchannels() == 1 and wav.getsampwidth() == 2
        pcm = wav.readframes(wav.getnframes())
        audio_seconds = wav.getnframes() / wav.getframerate()
    final = []
    asr = SherpaZipformerAsr(str(asr_dir), 16000, 2)
    asr.start(lambda _text: None, final.append)
    started = time.perf_counter()
    for offset in range(0, len(pcm), 3200):
        asr.push_audio(pcm[offset : offset + 3200])
    commit_at = time.perf_counter()
    asr.commit()
    ended = time.perf_counter()

    tts = SherpaVitsTts(str(root / "models/vits-melo-tts-zh_en"), 2, 0, 1.0)
    tts_started = time.perf_counter()
    tts_pcm = tts.synthesize("你好，我是离线机器人。")
    tts_ended = time.perf_counter()
    synthesized_seconds = len(tts_pcm) / 2 / tts.sample_rate
    report = {
        "asr_text": final[-1] if final else "",
        "asr_audio_seconds": round(audio_seconds, 3),
        "asr_compute_ms": round((ended - started) * 1000, 2),
        "asr_finalize_ms": round((ended - commit_at) * 1000, 2),
        "asr_realtime_factor": round((ended - started) / audio_seconds, 4),
        "tts_compute_ms": round((tts_ended - tts_started) * 1000, 2),
        "tts_audio_seconds": round(synthesized_seconds, 3),
        "tts_realtime_factor": round((tts_ended - tts_started) / synthesized_seconds, 4),
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
