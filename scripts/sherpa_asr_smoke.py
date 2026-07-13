#!/usr/bin/env python3
"""Sherpa-ONNX ZipFormer ASR 部署预检与单 wav 冒烟测试。

这个脚本只验证离线 ASR 层，不启动 llama.cpp/TTS/Gazebo。它复用项目里的
SherpaZipformerAsr provider，因此能证明“真实 sherpa-onnx 包 + 真实模型文件”
已经能被当前 Offline Agent 代码加载和解码。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import wave
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_MODEL_DIR = (
    WORKSPACE
    / "models"
    / "sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16"
)
DEFAULT_WAV = DEFAULT_MODEL_DIR / "test_wavs" / "0.wav"
REQUIRED_MODEL_FILES: dict[str, int] = {
    "tokens.txt": 50_000,
    "encoder-epoch-99-avg-1.int8.onnx": 40_000_000,
    "decoder-epoch-99-avg-1.int8.onnx": 3_000_000,
    "joiner-epoch-99-avg-1.int8.onnx": 3_000_000,
}


def _ensure_repo_import_path() -> None:
    """让脚本在未 colcon install 的源码树中也能导入 provider。

    正常验收会先 source scripts/activate.sh；这里额外加入源码路径，是为了让
    ASR-only 部署脚本可以作为独立探针使用，不被完整 ROS 构建状态绑死。
    """

    for relative in ("src/embodied_offline_agent", "src/embodied_online_agent"):
        path = str(WORKSPACE / relative)
        if path not in sys.path:
            sys.path.insert(0, path)


def check_model_dir(model_dir: Path) -> list[dict[str, object]]:
    """返回模型文件检查报告；调用方决定缺失是否终止。"""

    report: list[dict[str, object]] = []
    for name, minimum_bytes in REQUIRED_MODEL_FILES.items():
        path = model_dir / name
        size = path.stat().st_size if path.exists() else 0
        report.append(
            {
                "file": str(path),
                "exists": path.exists(),
                "size": size,
                "minimum_bytes": minimum_bytes,
                "ok": path.exists() and size >= minimum_bytes,
            }
        )
    return report


def assert_preflight(model_dir: Path, wav_path: Path | None) -> None:
    failed = [item for item in check_model_dir(model_dir) if not item["ok"]]
    if failed:
        raise SystemExit(
            "Sherpa ASR model preflight failed: "
            + json.dumps(failed, ensure_ascii=False)
        )
    if wav_path is not None and not wav_path.is_file():
        raise SystemExit(f"wav file not found: {wav_path}")


def read_pcm16_wav(path: Path, expected_sample_rate: int) -> tuple[bytes, float]:
    """读取 16kHz/mono/PCM16 wav，返回原始 PCM16 bytes 与时长。"""

    with wave.open(str(path), "rb") as wav:
        channels = wav.getnchannels()
        sample_width = wav.getsampwidth()
        sample_rate = wav.getframerate()
        frame_count = wav.getnframes()
        if channels != 1:
            raise ValueError(f"{path} must be mono; got channels={channels}")
        if sample_width != 2:
            raise ValueError(f"{path} must be PCM16; got sample_width={sample_width}")
        if sample_rate != expected_sample_rate:
            raise ValueError(
                f"{path} sample rate must be {expected_sample_rate}; got {sample_rate}"
            )
        return wav.readframes(frame_count), frame_count / float(sample_rate)


def run_smoke(args: argparse.Namespace) -> dict[str, object]:
    model_dir = Path(args.model_dir).expanduser()
    wav_path = Path(args.wav).expanduser()
    assert_preflight(model_dir, wav_path)
    pcm16, audio_seconds = read_pcm16_wav(wav_path, args.sample_rate)

    _ensure_repo_import_path()
    from embodied_offline_agent.providers.sherpa_asr import SherpaZipformerAsr

    finals: list[str] = []
    partials: list[str] = []
    asr = SherpaZipformerAsr(
        str(model_dir),
        args.sample_rate,
        args.num_threads,
        decoding_method=args.decoding_method,
        hotwords_file=args.hotwords_file,
        hotwords_score=args.hotwords_score,
        max_active_paths=args.max_active_paths,
        modeling_unit=args.modeling_unit,
    )
    asr.start(partials.append, finals.append)

    chunk_bytes = max(1, args.chunk_samples) * 2
    started = time.perf_counter()
    for offset in range(0, len(pcm16), chunk_bytes):
        asr.push_audio(pcm16[offset : offset + chunk_bytes])
    fed_at = time.perf_counter()
    asr.commit()
    ended = time.perf_counter()

    text = finals[-1] if finals else ""
    report = {
        "ok": bool(text) or args.allow_empty,
        "text": text,
        "partial_count": len(partials),
        "final_count": len(finals),
        "audio_seconds": round(audio_seconds, 3),
        "feed_ms": round((fed_at - started) * 1000.0, 2),
        "finalize_ms": round((ended - fed_at) * 1000.0, 2),
        "total_ms": round((ended - started) * 1000.0, 2),
        "rtf": round((ended - started) / audio_seconds, 4) if audio_seconds else 0.0,
        "model_dir": str(model_dir),
        "wav": str(wav_path),
    }
    if not report["ok"]:
        raise SystemExit(json.dumps(report, ensure_ascii=False, indent=2))
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model-dir", default=str(DEFAULT_MODEL_DIR))
    parser.add_argument("--wav", default=str(DEFAULT_WAV))
    parser.add_argument("--sample-rate", type=int, default=16_000)
    parser.add_argument("--num-threads", type=int, default=2)
    parser.add_argument("--decoding-method", default="modified_beam_search")
    parser.add_argument("--max-active-paths", type=int, default=4)
    parser.add_argument("--modeling-unit", default="cjkchar")
    parser.add_argument("--hotwords-file", default="")
    parser.add_argument("--hotwords-score", type=float, default=3.0)
    parser.add_argument("--chunk-samples", type=int, default=1600)
    parser.add_argument("--allow-empty", action="store_true")
    parser.add_argument(
        "--preflight-only",
        action="store_true",
        help="只检查 sherpa_onnx import、模型文件和 wav 文件，不执行解码。",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    model_dir = Path(args.model_dir).expanduser()
    wav_path = Path(args.wav).expanduser()
    _ensure_repo_import_path()
    try:
        import sherpa_onnx  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "sherpa_onnx is not installed. Run: bash scripts/setup_sherpa_asr_runtime.sh"
        ) from exc

    if args.preflight_only:
        assert_preflight(model_dir, wav_path)
        print(
            json.dumps(
                {
                    "ok": True,
                    "mode": "preflight",
                    "model_dir": str(model_dir),
                    "wav": str(wav_path),
                    "files": check_model_dir(model_dir),
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 0

    print(json.dumps(run_smoke(args), ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
