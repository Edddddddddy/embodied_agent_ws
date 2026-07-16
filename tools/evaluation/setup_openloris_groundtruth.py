#!/usr/bin/env python3
"""Download, verify and extract one OpenLORIS-Scene ground-truth trajectory."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import time
import urllib.request
import zipfile
from pathlib import Path


ARCHIVE_URL = (
    "https://huggingface.co/datasets/shixuesong/openloris-scene/resolve/"
    "main/package/groundtruth.zip"
)
ARCHIVE_SIZE = 11_076_559
ARCHIVE_SHA256 = "07564d7ed3d6739585002afa12bcf481cc0e9e358fc64efd5e658e2c994bdc3b"
DATASET_PAGE = "https://lifelong-robotic-vision.github.io/dataset/scene.html"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def download_with_resume(
    url: str,
    destination: Path,
    *,
    expected_size: int = ARCHIVE_SIZE,
    max_attempts: int = 5,
    chunk_size: int = 1024 * 1024,
) -> Path:
    """断点续传小型真值包，并在大小满足契约后原子发布。

    Hugging Face CDN 偶尔会在响应末尾提前 EOF。直接使用 ``urlretrieve`` 会丢弃
    本轮进度；这里保留 ``.part``，下一次只请求缺失尾部，同时拒绝错误的 206
    ``Content-Range``，避免把不连续字节拼成表面合法的 ZIP。
    """

    destination.parent.mkdir(parents=True, exist_ok=True)
    partial = destination.with_name(destination.name + ".part")
    for attempt in range(1, max_attempts + 1):
        offset = partial.stat().st_size if partial.exists() else 0
        if offset > expected_size:
            raise ValueError(f"partial archive is larger than expected: {partial}")
        if offset == expected_size:
            os.replace(partial, destination)
            return destination

        request = urllib.request.Request(url)
        if offset:
            request.add_header("Range", f"bytes={offset}-{expected_size - 1}")
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                status = getattr(response, "status", response.getcode())
                append = offset > 0 and status == 206
                if append:
                    content_range = response.headers.get("Content-Range", "")
                    expected_prefix = f"bytes {offset}-{expected_size - 1}/"
                    if content_range and not content_range.startswith(expected_prefix):
                        raise ValueError(
                            "unexpected Content-Range: "
                            f"expected {expected_prefix}*, got {content_range}"
                        )
                # CDN 忽略 Range 并返回 200 时，从头覆盖，绝不能追加到旧半包。
                mode = "ab" if append else "wb"
                with partial.open(mode) as target:
                    while block := response.read(chunk_size):
                        target.write(block)
        except (OSError, TimeoutError) as error:
            if attempt == max_attempts:
                raise
            print(
                f"retry ground truth attempt={attempt + 1}/{max_attempts}: {error}",
                flush=True,
            )
            time.sleep(min(attempt, 3))
            continue

        current_size = partial.stat().st_size
        if current_size == expected_size:
            os.replace(partial, destination)
            return destination
        if current_size > expected_size:
            raise ValueError(
                f"download exceeded expected size: {current_size} > {expected_size}"
            )
        if attempt < max_attempts:
            print(
                f"retry short ground truth downloaded={current_size}/{expected_size} "
                f"attempt={attempt + 1}/{max_attempts}",
                flush=True,
            )
            time.sleep(min(attempt, 3))

    raise ValueError(
        f"download incomplete after {max_attempts} attempts: "
        f"expected {expected_size}, got {partial.stat().st_size}"
    )


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
    if archive is None:
        archive = output_root / "archive" / "groundtruth.zip"
        if not archive.is_file():
            print(f"Downloading {ARCHIVE_URL}")
            download_with_resume(ARCHIVE_URL, archive)
    actual_hash = sha256(archive)
    if actual_hash != expected_sha256:
        raise ValueError(f"SHA256 mismatch: expected {expected_sha256}, got {actual_hash}")
    destination = extract_sequence(archive, sequence, output_root)
    office_mocap = sequence.startswith("office")
    metadata = {
        "dataset": "OpenLORIS-Scene",
        "sequence": sequence,
        "dataset_page": DATASET_PAGE,
        "archive_url": ARCHIVE_URL,
        "archive_size_bytes": archive.stat().st_size,
        "archive_sha256": actual_hash,
        "archive_verification": "full_size_and_sha256",
        "ground_truth_kind": (
            "optitrack_motion_capture"
            if office_mocap
            else "official_offline_lidar_slam"
        ),
        "independent_of_online_hokuyo": office_mocap,
        "ground_truth_note": (
            "office sequences use OptiTrack motion-capture ground truth; other scenes "
            "use the dataset's offline high-resolution LiDAR SLAM trajectory"
        ),
    }
    (destination.parent / "source.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
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
