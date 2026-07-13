#!/usr/bin/env python3
"""Download, verify and extract one OpenLORIS-Scene ground-truth trajectory."""

from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import tempfile
import urllib.request
import zipfile
from pathlib import Path


ARCHIVE_URL = (
    "https://huggingface.co/datasets/shixuesong/openloris-scene/resolve/"
    "main/package/groundtruth.zip"
)
ARCHIVE_SHA256 = "07564d7ed3d6739585002afa12bcf481cc0e9e358fc64efd5e658e2c994bdc3b"
DATASET_PAGE = "https://lifelong-robotic-vision.github.io/dataset/scene.html"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def extract_sequence(archive: Path, sequence: str, output_root: Path) -> Path:
    member = f"per-sequence/{sequence}/groundtruth.txt"
    with zipfile.ZipFile(archive) as zipped:
        if member not in zipped.namelist():
            available = sorted(
                name.split("/")[1]
                for name in zipped.namelist()
                if name.startswith("per-sequence/") and name.endswith("/groundtruth.txt")
            )
            raise ValueError(f"unknown sequence {sequence!r}; available: {', '.join(available)}")
        destination = output_root / "groundtruth" / sequence / "groundtruth.txt"
        destination.parent.mkdir(parents=True, exist_ok=True)
        # 只按精确成员名复制内容，不让 ZIP 内路径控制写入位置。
        with zipped.open(member) as source, destination.open("wb") as target:
            shutil.copyfileobj(source, target)
    return destination


def prepare(
    *,
    sequence: str,
    output_root: Path,
    archive: Path | None = None,
    expected_sha256: str = ARCHIVE_SHA256,
) -> Path:
    temporary: tempfile.TemporaryDirectory[str] | None = None
    if archive is None:
        temporary = tempfile.TemporaryDirectory(prefix="openloris-")
        archive = Path(temporary.name) / "groundtruth.zip"
        print(f"Downloading {ARCHIVE_URL}")
        urllib.request.urlretrieve(ARCHIVE_URL, archive)
    actual_hash = sha256(archive)
    if actual_hash != expected_sha256:
        if temporary:
            temporary.cleanup()
        raise ValueError(f"SHA256 mismatch: expected {expected_sha256}, got {actual_hash}")
    destination = extract_sequence(archive, sequence, output_root)
    metadata = {
        "dataset": "OpenLORIS-Scene",
        "sequence": sequence,
        "dataset_page": DATASET_PAGE,
        "archive_url": ARCHIVE_URL,
        "archive_sha256": actual_hash,
        "ground_truth_note": (
            "office sequences use OptiTrack motion-capture ground truth; "
            "check the official page before interpreting other scenes"
        ),
    }
    (destination.parent / "source.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    if temporary:
        temporary.cleanup()
    return destination


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", default="office1-1")
    parser.add_argument("--output-root", type=Path, default=Path("datasets/openloris"))
    parser.add_argument("--archive", type=Path, help="Use an already downloaded archive")
    args = parser.parse_args()
    destination = prepare(
        sequence=args.sequence, output_root=args.output_root, archive=args.archive
    )
    print(f"PASS: verified ground truth -> {destination}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
