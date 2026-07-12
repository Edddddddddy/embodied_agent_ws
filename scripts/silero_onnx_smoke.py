#!/usr/bin/env python3
"""Run the pinned Silero ONNX model and emit machine-readable latency evidence."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import statistics
import time
from pathlib import Path
import wave

from embodied_voice_frontend.silero_vad_sidecar import SileroOnnxVadProvider


DEFAULT_MODEL = Path("models/silero_vad/silero_vad.onnx")
DEFAULT_WAV = Path(
    "models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16"
    "/test_wavs/0.wav"
)


def _wav_frames(path: Path, frame_samples: int = 512) -> list[bytes]:
    with wave.open(str(path), "rb") as source:
        if source.getnchannels() != 1:
            raise ValueError("Silero smoke wav must be mono")
        if source.getsampwidth() != 2:
            raise ValueError("Silero smoke wav must use PCM16 samples")
        if source.getframerate() != 16000:
            raise ValueError("Silero smoke wav must use 16000Hz sample rate")
        pcm = source.readframes(source.getnframes())

    frame_bytes = frame_samples * 2
    frames = []
    for offset in range(0, len(pcm), frame_bytes):
        frame = pcm[offset:offset + frame_bytes]
        if len(frame) < frame_bytes:
            frame += b"\x00" * (frame_bytes - len(frame))
        frames.append(frame)
    return frames


def _percentile(values: list[float], ratio: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(len(ordered) * ratio) - 1))
    return ordered[index]


def run_smoke(model: Path, wav_path: Path | None, threshold: float) -> dict:
    provider = SileroOnnxVadProvider(model_path=str(model))
    silence_frames = [b"\x00\x00" * 512 for _ in range(8)]
    provider.reset()
    silence_probabilities = [
        provider.speech_probability(frame, 16000) for frame in silence_frames
    ]

    frames = _wav_frames(wav_path) if wav_path is not None else silence_frames
    provider.reset()
    probabilities: list[float] = []
    inference_ms: list[float] = []
    for frame in frames:
        started = time.perf_counter()
        probabilities.append(provider.speech_probability(frame, 16000))
        inference_ms.append((time.perf_counter() - started) * 1000.0)

    if not probabilities or not all(math.isfinite(value) for value in probabilities):
        raise RuntimeError("Silero ONNX returned no finite probabilities")
    if not all(0.0 <= value <= 1.0 for value in probabilities):
        raise RuntimeError("Silero ONNX probability is outside [0, 1]")
    speech_like_frames = sum(value >= threshold for value in probabilities)
    if wav_path is not None and speech_like_frames == 0:
        raise RuntimeError("Silero ONNX did not detect speech in the acceptance wav")

    import onnxruntime

    return {
        "status": "PASS",
        "provider": "silero_onnx",
        "onnxruntime_version": onnxruntime.__version__,
        "model_path": str(model.resolve()),
        "model_bytes": model.stat().st_size,
        "model_sha256": hashlib.sha256(model.read_bytes()).hexdigest(),
        "wav_path": str(wav_path.resolve()) if wav_path is not None else None,
        "frames": len(frames),
        "frame_ms": 32,
        "threshold": threshold,
        "speech_like_frames": speech_like_frames,
        "max_probability": round(max(probabilities), 6),
        "silence_max_probability": round(max(silence_probabilities), 6),
        "inference_ms": {
            "mean": round(statistics.fmean(inference_ms), 4),
            "p50": round(_percentile(inference_ms, 0.50), 4),
            "p95": round(_percentile(inference_ms, 0.95), 4),
            "max": round(max(inference_ms), 4),
        },
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--wav", type=Path, default=DEFAULT_WAV)
    parser.add_argument("--threshold", type=float, default=0.5)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    wav_path = args.wav if args.wav.exists() else None
    report = run_smoke(args.model, wav_path, args.threshold)
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    print(rendered)
    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
