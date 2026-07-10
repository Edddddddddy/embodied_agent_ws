#!/usr/bin/env python3
"""Probe the real sherpa-onnx speaker embedding runtime with an existing WAV."""

from __future__ import annotations

import argparse
import hashlib
import json
import time
import wave
from pathlib import Path

import numpy as np
import sherpa_onnx

from embodied_online_agent.speaker_identity_node import classify_speaker_scores


WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_MODEL = (
    WORKSPACE
    / "models"
    / "speaker_id"
    / "3dspeaker_speech_eres2net_base_200k_sv_zh-cn_16k-common.onnx"
)
DEFAULT_WAV = (
    WORKSPACE
    / "models"
    / "sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16"
    / "test_wavs"
    / "0.wav"
)


def read_wav(path: Path) -> tuple[np.ndarray, int, float]:
    with wave.open(str(path), "rb") as stream:
        sample_rate = stream.getframerate()
        channels = stream.getnchannels()
        sample_width = stream.getsampwidth()
        frames = stream.readframes(stream.getnframes())
    if sample_width != 2:
        raise ValueError("speaker probe requires int16 WAV")
    samples = np.frombuffer(frames, dtype=np.int16)
    if channels > 1:
        samples = samples.reshape(-1, channels)[:, 0]
    waveform = np.ascontiguousarray(samples.astype(np.float32) / 32768.0)
    return waveform, sample_rate, len(waveform) / sample_rate


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--wav", type=Path, default=DEFAULT_WAV)
    parser.add_argument("--threshold", type=float, default=0.6)
    parser.add_argument("--min-margin", type=float, default=0.05)
    parser.add_argument(
        "--output", type=Path, default=WORKSPACE / "logs" / "speaker_runtime_report.json"
    )
    args = parser.parse_args()

    if not args.model.is_file() or not args.wav.is_file():
        missing = [str(path) for path in (args.model, args.wav) if not path.is_file()]
        print(json.dumps({"status": "MISSING", "files": missing}, ensure_ascii=False))
        return 2

    config = sherpa_onnx.SpeakerEmbeddingExtractorConfig(
        model=str(args.model), num_threads=2, provider="cpu"
    )
    if not config.validate():
        raise RuntimeError(f"invalid speaker embedding config: {config}")
    extractor = sherpa_onnx.SpeakerEmbeddingExtractor(config)
    waveform, sample_rate, duration_s = read_wav(args.wav)
    stream = extractor.create_stream()
    stream.accept_waveform(sample_rate=sample_rate, waveform=waveform)
    stream.input_finished()
    started = time.perf_counter()
    embedding = extractor.compute(stream)
    inference_ms = (time.perf_counter() - started) * 1000.0

    manager = sherpa_onnx.SpeakerEmbeddingManager(extractor.dim)
    if not manager.add("runtime_probe_user", embedding):
        raise RuntimeError("failed to register runtime probe embedding")
    self_score = float(manager.score("runtime_probe_user", embedding))
    decision = classify_speaker_scores(
        {"runtime_probe_user": self_score},
        threshold=args.threshold,
        min_margin=args.min_margin,
    )
    # 用等分数的合成冲突验证 margin 防护；这不是第二个真人声纹准确率证据。
    ambiguous = classify_speaker_scores(
        {"runtime_probe_user": self_score, "synthetic_collision": self_score},
        threshold=args.threshold,
        min_margin=args.min_margin,
    )
    report = {
        "schema_version": 1,
        "status": "PASS" if decision.matched and not ambiguous.matched else "FAIL",
        "provider": "sherpa-onnx",
        "model": str(args.model),
        "model_sha256": hashlib.sha256(args.model.read_bytes()).hexdigest(),
        "model_size_bytes": args.model.stat().st_size,
        "embedding_dim": extractor.dim,
        "wav": str(args.wav),
        "audio_duration_s": round(duration_s, 3),
        "embedding_inference_ms": round(inference_ms, 3),
        "self_match_score": round(self_score, 4),
        "self_match_passed": decision.matched,
        "ambiguity_guard_passed": (
            not ambiguous.matched and ambiguous.reason == "ambiguous_match"
        ),
        "evidence_scope": "real_runtime_self_match_not_multi_speaker_accuracy",
        "multi_speaker_accuracy_evaluated": False,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"Evidence: {args.output}")
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
