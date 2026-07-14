#!/usr/bin/env python3
"""Download, verify and safely extract one OpenLORIS office rosbag."""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import json
import math
import os
import shutil
import tarfile
import time
import urllib.request
from pathlib import Path, PurePosixPath
from typing import NamedTuple


ARCHIVE_NAME = "office1-1_7-rosbag.tar"
ARCHIVE_URL = (
    "https://huggingface.co/datasets/shixuesong/openloris-scene/resolve/"
    f"main/rosbag/{ARCHIVE_NAME}?download=true"
)
ARCHIVE_SIZE = 9_267_230_720
ARCHIVE_SHA256 = "f43ae38cd560e150d8a06bbac527235dc53a536a07bdea77db7e672c13875912"
DATASET_COMMIT = "cbc03108723d08322b23d0338680bffa9404cce9"
DATASET_PAGE = "https://lifelong-robotic-vision.github.io/dataset/scene.html"
OFFICE_SEQUENCES = tuple(f"office1-{index}" for index in range(1, 8))
OFFICE1_1_MEMBER_SIZE = 1_245_178_774
OFFICE1_1_RANGE_END = 1_245_179_391
OFFICE1_1_RANGE_SHA256 = "c637329c32caa00561419cdce8d04bc7e659cf5edcdc3a6b8f220791210f3c52"
OFFICE1_1_BAG_SHA256 = "d18e335a34dc25b6f26df911886bdefbeeeacd41620c0ea525fe0314ea9f7a65"


class SequenceRangeContract(NamedTuple):
    range_start: int
    range_end: int
    member_size: int
    range_sha256: str
    bag_sha256: str


# 这些边界来自固定 commit 的未压缩 tar header；range 与解出的 bag 还必须分别过摘要校验。
SEQUENCE_RANGE_CONTRACTS = {
    "office1-7": SequenceRangeContract(
        range_start=7_838_556_672,
        range_end=9_267_224_063,
        member_size=1_428_666_703,
        range_sha256="da2abbacc4a4c200890c8128186b677d0c3b0a7de6a66a2d7095c656ff0e4b6b",
        bag_sha256="23f443c33353e4059b5d108ca099d452550954613b8af1b02e8159fb147af3ca",
    )
}


def sha256(path: Path) -> str:
    """Hash a potentially large file without reading it all into memory."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify_archive(
    archive: Path,
    *,
    expected_size: int = ARCHIVE_SIZE,
    expected_sha256: str = ARCHIVE_SHA256,
) -> dict[str, object]:
    """Fail closed when the public object is incomplete or has changed."""

    actual_size = archive.stat().st_size
    if actual_size != expected_size:
        raise ValueError(
            f"archive size mismatch: expected {expected_size}, got {actual_size}"
        )
    actual_sha256 = sha256(archive)
    if actual_sha256 != expected_sha256:
        raise ValueError(
            f"archive SHA256 mismatch: expected {expected_sha256}, got {actual_sha256}"
        )
    return {"size_bytes": actual_size, "sha256": actual_sha256}


def download_with_resume(
    url: str,
    destination: Path,
    *,
    expected_size: int = ARCHIVE_SIZE,
    chunk_size: int = 8 * 1024 * 1024,
) -> Path:
    """Download with HTTP Range support and atomically publish the complete file."""

    destination.parent.mkdir(parents=True, exist_ok=True)
    partial = destination.with_name(destination.name + ".part")
    offset = partial.stat().st_size if partial.exists() else 0
    if offset > expected_size:
        raise ValueError(f"partial file is larger than expected: {partial}")
    if offset == expected_size:
        os.replace(partial, destination)
        return destination
    request = urllib.request.Request(url)
    if offset:
        request.add_header("Range", f"bytes={offset}-")
    with urllib.request.urlopen(request, timeout=60) as response:
        status = getattr(response, "status", response.getcode())
        # CDN 忽略 Range 时必须从头覆盖，不能把完整响应追加到半包后面。
        append = offset > 0 and status == 206
        mode = "ab" if append else "wb"
        downloaded = offset if append else 0
        started = time.monotonic()
        with partial.open(mode) as target:
            while True:
                block = response.read(chunk_size)
                if not block:
                    break
                target.write(block)
                downloaded += len(block)
                elapsed = max(time.monotonic() - started, 1e-6)
                print(
                    f"downloaded={downloaded}/{expected_size} "
                    f"speed_mib_s={(downloaded - (offset if append else 0)) / elapsed / 2**20:.2f}",
                    flush=True,
                )
    if partial.stat().st_size != expected_size:
        raise ValueError(
            f"download incomplete: expected {expected_size}, got {partial.stat().st_size}"
        )
    os.replace(partial, destination)
    return destination


def _download_range_part(
    url: str,
    destination: Path,
    *,
    start: int,
    end: int,
    chunk_size: int = 8 * 1024 * 1024,
    max_attempts: int = 5,
) -> Path:
    """Resume one byte range and reject servers that silently return another object."""

    expected_size = end - start + 1
    destination.parent.mkdir(parents=True, exist_ok=True)
    for attempt in range(1, max_attempts + 1):
        offset = destination.stat().st_size if destination.exists() else 0
        if offset > expected_size:
            raise ValueError(f"range part is larger than expected: {destination}")
        if offset == expected_size:
            return destination

        request_start = start + offset
        request = urllib.request.Request(url)
        request.add_header("Range", f"bytes={request_start}-{end}")
        try:
            with urllib.request.urlopen(request, timeout=60) as response:
                status = getattr(response, "status", response.getcode())
                content_range = response.headers.get("Content-Range", "")
                if status != 206:
                    raise ValueError(f"server ignored required byte range: HTTP {status}")
                # 只信任与请求起点完全一致的 206；否则分段可能拼成表面合法的损坏 tar。
                expected_prefix = f"bytes {request_start}-{end}/"
                if content_range and not content_range.startswith(expected_prefix):
                    raise ValueError(
                        f"unexpected Content-Range: expected {expected_prefix}*, "
                        f"got {content_range}"
                    )
                downloaded = offset
                next_report = downloaded + 64 * 1024 * 1024
                with destination.open("ab") as target:
                    while True:
                        block = response.read(chunk_size)
                        if not block:
                            break
                        target.write(block)
                        downloaded += len(block)
                        if downloaded >= next_report:
                            print(
                                f"range={start}-{end} "
                                f"downloaded={downloaded}/{expected_size}",
                                flush=True,
                            )
                            next_report += 64 * 1024 * 1024
        except (OSError, TimeoutError) as error:
            if attempt == max_attempts:
                raise
            print(
                f"retry range={start}-{end} attempt={attempt + 1}/{max_attempts}: {error}",
                flush=True,
            )
            time.sleep(min(attempt, 3))
            continue

        current_size = destination.stat().st_size
        if current_size == expected_size:
            return destination
        # Hugging Face CDN 偶尔在 206 响应末尾提前 EOF；下次只请求缺失尾部。
        if attempt < max_attempts:
            print(
                f"retry short range={start}-{end} "
                f"downloaded={current_size}/{expected_size} "
                f"attempt={attempt + 1}/{max_attempts}",
                flush=True,
            )
            time.sleep(min(attempt, 3))

    raise ValueError(
        f"range download incomplete after {max_attempts} attempts: "
        f"expected {expected_size}, got {destination.stat().st_size} ({destination})"
    )


def download_range_parallel(
    url: str,
    destination: Path,
    *,
    expected_size: int,
    connections: int = 8,
    remote_start: int = 0,
) -> Path:
    """Download one bounded remote interval with resumable byte ranges.

    ``remote_start`` keeps local part offsets independent from the source object.  That
    lets callers fetch a later tar member without downloading every preceding bag.
    """

    if connections < 1:
        raise ValueError("connections must be positive")
    if expected_size < 1 or remote_start < 0:
        raise ValueError("range bounds must be non-negative and non-empty")
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.is_file():
        if destination.stat().st_size == expected_size:
            return destination
        raise ValueError(f"existing range archive has an unexpected size: {destination}")

    part_root = destination.with_name(destination.name + ".parts")
    part_root.mkdir(parents=True, exist_ok=True)
    span = math.ceil(expected_size / connections)
    ranges = [
        (index, index * span, min((index + 1) * span, expected_size) - 1)
        for index in range(connections)
        if index * span < expected_size
    ]

    # 兼容早期单连接实现留下的 `.part`：它就是从 offset=0 开始的第 0 段前缀。
    legacy_partial = destination.with_name(destination.name + ".part")
    first_part = part_root / "000.part"
    if legacy_partial.exists():
        if first_part.exists():
            raise ValueError(
                f"both legacy and segmented partial files exist: {legacy_partial}, {first_part}"
            )
        if legacy_partial.stat().st_size > ranges[0][2] + 1:
            raise ValueError(f"legacy partial is too large for first segment: {legacy_partial}")
        os.replace(legacy_partial, first_part)

    with concurrent.futures.ThreadPoolExecutor(max_workers=len(ranges)) as executor:
        futures = {
            executor.submit(
                _download_range_part,
                url,
                part_root / f"{index:03d}.part",
                start=remote_start + start,
                end=remote_start + end,
            ): (index, start, end)
            for index, start, end in ranges
        }
        for future in concurrent.futures.as_completed(futures):
            index, start, end = futures[future]
            future.result()
            print(
                f"completed range[{index}]="
                f"{remote_start + start}-{remote_start + end}",
                flush=True,
            )

    assembled = destination.with_name(destination.name + ".assemble.part")
    with assembled.open("wb") as target:
        for index, _start, _end in ranges:
            with (part_root / f"{index:03d}.part").open("rb") as source:
                shutil.copyfileobj(source, target, length=8 * 1024 * 1024)
    if assembled.stat().st_size != expected_size:
        raise ValueError(
            f"assembled range has unexpected size: {assembled.stat().st_size}"
        )
    os.replace(assembled, destination)
    for index, _start, _end in ranges:
        (part_root / f"{index:03d}.part").unlink()
    part_root.rmdir()
    return destination


def _sequence_member(archive: tarfile.TarFile, sequence: str) -> tarfile.TarInfo:
    candidates: list[tarfile.TarInfo] = []
    for member in archive.getmembers():
        path = PurePosixPath(member.name)
        # 不调用 TarFile.extract；只允许普通文件且 basename 精确匹配，封死路径穿越/链接覆盖。
        if (
            member.isfile()
            and not path.is_absolute()
            and ".." not in path.parts
            and path.name == f"{sequence}.bag"
        ):
            candidates.append(member)
    if len(candidates) != 1:
        names = [item.name for item in candidates]
        raise ValueError(f"expected one {sequence}.bag member, found {names}")
    return candidates[0]


def extract_sequence(archive_path: Path, sequence: str, output_root: Path) -> Path:
    """Extract exactly one bag into a deterministic dataset directory."""

    if sequence not in OFFICE_SEQUENCES:
        raise ValueError(f"unsupported office sequence: {sequence}")
    destination = output_root / "rosbag" / sequence / f"{sequence}.bag"
    if destination.is_file() and destination.stat().st_size > 0:
        return destination
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(".bag.part")
    with tarfile.open(archive_path, mode="r:") as archive:
        member = _sequence_member(archive, sequence)
        source = archive.extractfile(member)
        if source is None:
            raise ValueError(f"cannot read tar member: {member.name}")
        with source, temporary.open("wb") as target:
            shutil.copyfileobj(source, target, length=8 * 1024 * 1024)
    os.replace(temporary, destination)
    return destination


def prepare_sequence_range(
    *,
    sequence: str,
    contract: SequenceRangeContract,
    output_root: Path,
    download: bool = True,
    connections: int = 8,
) -> Path:
    """Fetch and verify exactly one member interval from the pinned remote tar."""

    range_archive = output_root / "archive" / f"{sequence}.range.tar"
    expected_range_size = contract.range_end - contract.range_start + 1
    if not range_archive.is_file():
        if not download:
            raise FileNotFoundError(range_archive)
        download_range_parallel(
            ARCHIVE_URL,
            range_archive,
            expected_size=expected_range_size,
            connections=connections,
            remote_start=contract.range_start,
        )
    if range_archive.stat().st_size != expected_range_size:
        raise ValueError(
            f"range size mismatch: expected {expected_range_size}, "
            f"got {range_archive.stat().st_size}"
        )
    range_sha256 = sha256(range_archive)
    if range_sha256 != contract.range_sha256:
        raise ValueError(
            f"range SHA256 mismatch: expected {contract.range_sha256}, "
            f"got {range_sha256}"
        )
    destination = output_root / "rosbag" / sequence / f"{sequence}.bag"
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_suffix(".bag.part")
    # 本地 range 从目标成员 header 开始；流式读取无需伪造前序成员或 tar 结束块。
    with tarfile.open(range_archive, mode="r|") as archive:
        member = archive.next()
        if (
            member is None
            or not member.isfile()
            or PurePosixPath(member.name).name != f"{sequence}.bag"
            or member.size != contract.member_size
        ):
            raise ValueError(f"unexpected range archive member: {member}")
        source = archive.extractfile(member)
        if source is None:
            raise ValueError("cannot read range rosbag member")
        with source, temporary.open("wb") as target:
            shutil.copyfileobj(source, target, length=8 * 1024 * 1024)
    bag_sha256 = sha256(temporary)
    if bag_sha256 != contract.bag_sha256:
        raise ValueError(
            f"bag SHA256 mismatch: expected {contract.bag_sha256}, got {bag_sha256}"
        )
    # 只有 tar 成员和内容摘要都满足固定契约，消费者才会看到最终 .bag。
    os.replace(temporary, destination)
    metadata = {
        "dataset": "OpenLORIS-Scene",
        "sequence": sequence,
        "dataset_page": DATASET_PAGE,
        "dataset_commit": DATASET_COMMIT,
        "archive_url": ARCHIVE_URL,
        "archive_size_bytes": ARCHIVE_SIZE,
        "archive_sha256": ARCHIVE_SHA256,
        "archive_verification": "pinned_https_range_not_full_hash",
        "range_start": contract.range_start,
        "range_end": contract.range_end,
        "range_sha256": range_sha256,
        "bag": str(destination.resolve()),
        "bag_size_bytes": destination.stat().st_size,
        "bag_sha256": bag_sha256,
        "license": "CC BY-ND 4.0; consult the official dataset page",
    }
    (destination.parent / "source.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return destination


def prepare_first_sequence_range(
    *, output_root: Path, download: bool = True, connections: int = 8
) -> Path:
    """Backward-compatible office1-1 quick path used by existing automation."""

    return prepare_sequence_range(
        sequence="office1-1",
        contract=SequenceRangeContract(
            range_start=0,
            range_end=OFFICE1_1_RANGE_END,
            member_size=OFFICE1_1_MEMBER_SIZE,
            range_sha256=OFFICE1_1_RANGE_SHA256,
            bag_sha256=OFFICE1_1_BAG_SHA256,
        ),
        output_root=output_root,
        download=download,
        connections=connections,
    )


def prepare(
    *,
    sequence: str,
    output_root: Path,
    archive: Path | None = None,
    expected_size: int = ARCHIVE_SIZE,
    expected_sha256: str = ARCHIVE_SHA256,
    download: bool = True,
) -> Path:
    """Prepare one verified bag and write provenance beside it."""

    archive = archive or output_root / "archive" / ARCHIVE_NAME
    if not archive.is_file():
        if not download:
            raise FileNotFoundError(archive)
        print(f"Downloading {ARCHIVE_URL}")
        download_with_resume(ARCHIVE_URL, archive, expected_size=expected_size)
    contract = verify_archive(
        archive, expected_size=expected_size, expected_sha256=expected_sha256
    )
    destination = extract_sequence(archive, sequence, output_root)
    metadata = {
        "dataset": "OpenLORIS-Scene",
        "sequence": sequence,
        "dataset_page": DATASET_PAGE,
        "dataset_commit": DATASET_COMMIT,
        "archive_url": ARCHIVE_URL,
        "archive": str(archive.resolve()),
        "archive_size_bytes": contract["size_bytes"],
        "archive_sha256": contract["sha256"],
        "archive_verification": "full_size_and_sha256",
        "bag": str(destination.resolve()),
        "bag_size_bytes": destination.stat().st_size,
        "bag_sha256": sha256(destination),
        "license": "CC BY-ND 4.0; consult the official dataset page",
    }
    (destination.parent / "source.json").write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return destination


def main() -> int:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--sequence", choices=OFFICE_SEQUENCES, default="office1-1")
    parser.add_argument("--output-root", type=Path, default=Path("datasets/openloris"))
    parser.add_argument("--archive", type=Path)
    parser.add_argument(
        "--no-download", action="store_true", help="Require an existing local archive"
    )
    parser.add_argument(
        "--range-only",
        action="store_true",
        help="Download a pinned office1-1 or office1-7 tar member interval",
    )
    parser.add_argument(
        "--download-connections",
        type=int,
        default=8,
        help="Parallel HTTP ranges used by --range-only (default: 8)",
    )
    args = parser.parse_args()
    if args.range_only:
        if args.archive is not None or args.sequence not in {"office1-1", "office1-7"}:
            parser.error("--range-only supports office1-1/office1-7 without --archive")
        if args.sequence == "office1-1":
            bag = prepare_first_sequence_range(
                output_root=args.output_root,
                download=not args.no_download,
                connections=args.download_connections,
            )
        else:
            bag = prepare_sequence_range(
                sequence=args.sequence,
                contract=SEQUENCE_RANGE_CONTRACTS[args.sequence],
                output_root=args.output_root,
                download=not args.no_download,
                connections=args.download_connections,
            )
    else:
        bag = prepare(
            sequence=args.sequence,
            output_root=args.output_root,
            archive=args.archive,
            download=not args.no_download,
        )
    print(f"PASS: verified OpenLORIS rosbag -> {bag}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
