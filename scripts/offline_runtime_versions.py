#!/usr/bin/env python3
"""Report and optionally verify pinned offline runtime versions."""

from __future__ import annotations

import argparse
import hashlib
import importlib.metadata
import json
import subprocess
from pathlib import Path


WORKSPACE = Path(__file__).resolve().parents[1]
EXPECTED = {
    "llama.cpp": "0eca4d490e591d4e93058d07540cf47278a72577",
    "SummerTTS": "c90e0e8d31e09c98199ab9b5a605af74c179f811",
    "sherpa-onnx": "1.13.3",
    "onnxruntime": "1.27.0",
    "Silero VAD model": "1a153a22f4509e292a94e67d6f9b85e8deb25b4988682b7e174c65279d8788e3",
}


def _git_head(path: Path) -> str | None:
    if not (path / ".git").exists():
        return None
    result = subprocess.run(
        ["git", "-C", str(path), "rev-parse", "HEAD"],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if result.returncode != 0:
        return None
    return result.stdout.strip()


def _package_version(name: str) -> str | None:
    try:
        return importlib.metadata.version(name)
    except importlib.metadata.PackageNotFoundError:
        return None


def _sha256(path: Path) -> str | None:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _item(name: str, actual: str | None, path: Path | None = None) -> dict[str, object]:
    expected = EXPECTED[name]
    return {
        "name": name,
        "expected": expected,
        "actual": actual,
        "ok": actual == expected,
        "path": str(path) if path is not None else None,
    }


def collect() -> dict[str, object]:
    items = [
        _item(
            "llama.cpp",
            _git_head(WORKSPACE / "third_party" / "llama.cpp"),
            WORKSPACE / "third_party" / "llama.cpp",
        ),
        _item(
            "SummerTTS",
            _git_head(WORKSPACE / "third_party" / "SummerTTS"),
            WORKSPACE / "third_party" / "SummerTTS",
        ),
        _item("sherpa-onnx", _package_version("sherpa-onnx")),
        _item("onnxruntime", _package_version("onnxruntime")),
        _item(
            "Silero VAD model",
            _sha256(WORKSPACE / "models" / "silero_vad" / "silero_vad.onnx"),
            WORKSPACE / "models" / "silero_vad" / "silero_vad.onnx",
        ),
    ]
    return {"ok": all(bool(item["ok"]) for item in items), "items": items}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="exit non-zero on mismatch")
    args = parser.parse_args()
    report = collect()
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.check and not report["ok"]:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
