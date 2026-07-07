#!/usr/bin/env python3
"""SummerTTS 部署预检与真实合成 smoke。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
DEFAULT_BINARY = WORKSPACE / "third_party" / "SummerTTS" / "build" / "tts_test"
DEFAULT_MODEL = (
    WORKSPACE / "third_party" / "SummerTTS" / "models" / "single_speaker_fast.bin"
)
DEFAULT_SOURCE = WORKSPACE / "third_party" / "SummerTTS"


def _ensure_repo_import_path() -> None:
    for relative in ("src/embodied_offline_agent", "src/embodied_online_agent"):
        path = str(WORKSPACE / relative)
        if path not in sys.path:
            sys.path.insert(0, path)


def _file_state(path: Path, minimum_bytes: int = 1) -> dict[str, object]:
    size = path.stat().st_size if path.exists() else 0
    return {
        "file": str(path),
        "exists": path.exists(),
        "size": size,
        "minimum_bytes": minimum_bytes,
        "ok": path.is_file() and size >= minimum_bytes,
    }


def check_runtime(binary: Path, model: Path, source_dir: Path) -> list[dict[str, object]]:
    return [
        _file_state(source_dir / "README.md"),
        _file_state(source_dir / "include" / "SynthesizerTrn.h"),
        _file_state(binary, 100_000),
        _file_state(model, 10_000_000),
    ]


def assert_preflight(binary: Path, model: Path, source_dir: Path) -> None:
    failed = [item for item in check_runtime(binary, model, source_dir) if not item["ok"]]
    if failed:
        raise SystemExit(
            "SummerTTS runtime preflight failed: "
            + json.dumps(failed, ensure_ascii=False)
        )


def run_smoke(args: argparse.Namespace) -> dict[str, object]:
    binary = Path(args.binary).expanduser()
    model = Path(args.model).expanduser()
    source_dir = Path(args.source_dir).expanduser()
    assert_preflight(binary, model, source_dir)
    _ensure_repo_import_path()
    from embodied_offline_agent.providers.summer_tts import SummerTts

    started = time.perf_counter()
    provider = SummerTts(str(binary), str(model), timeout_s=args.timeout_s)
    loaded_at = time.perf_counter()
    pcm = provider.synthesize(args.text)
    ended = time.perf_counter()
    report = {
        "ok": len(pcm) > 0,
        "text": args.text,
        "pcm_bytes": len(pcm),
        "sample_rate": provider.sample_rate,
        "load_ms": round((loaded_at - started) * 1000.0, 2),
        "synthesize_ms": round((ended - loaded_at) * 1000.0, 2),
        "total_ms": round((ended - started) * 1000.0, 2),
        "binary": str(binary),
        "model": str(model),
    }
    if not report["ok"]:
        raise SystemExit(json.dumps(report, ensure_ascii=False, indent=2))
    return report


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", default=str(DEFAULT_BINARY))
    parser.add_argument("--model", default=str(DEFAULT_MODEL))
    parser.add_argument("--source-dir", default=str(DEFAULT_SOURCE))
    parser.add_argument("--text", default="你好，我是 SummerTTS 离线语音。")
    parser.add_argument("--timeout-s", type=float, default=30.0)
    parser.add_argument("--preflight-only", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    binary = Path(args.binary).expanduser()
    model = Path(args.model).expanduser()
    source_dir = Path(args.source_dir).expanduser()
    if args.preflight_only:
        assert_preflight(binary, model, source_dir)
        print(
            json.dumps(
                {
                    "ok": True,
                    "mode": "preflight",
                    "files": check_runtime(binary, model, source_dir),
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
