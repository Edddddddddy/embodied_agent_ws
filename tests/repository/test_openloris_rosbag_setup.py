from __future__ import annotations

import hashlib
import importlib.util
import io
import json
import tarfile
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "setup_openloris_rosbag", ROOT / "scripts" / "setup_openloris_rosbag.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _archive(path: Path) -> tuple[int, str]:
    payload = b"#ROSBAG V2.0\nfixture"
    with tarfile.open(path, "w") as archive:
        valid = tarfile.TarInfo("release/office1-1.bag")
        valid.size = len(payload)
        archive.addfile(valid, io.BytesIO(payload))
        escaped = tarfile.TarInfo("../../office1-1.bag")
        escaped.size = len(payload)
        archive.addfile(escaped, io.BytesIO(payload))
        link = tarfile.TarInfo("link/office1-1.bag")
        link.type = tarfile.SYMTYPE
        link.linkname = "/tmp/escape"
        archive.addfile(link)
    return path.stat().st_size, hashlib.sha256(path.read_bytes()).hexdigest()


def test_prepare_verifies_and_extracts_only_exact_regular_member(tmp_path):
    archive = tmp_path / "office.tar"
    size, digest = _archive(archive)
    destination = MODULE.prepare(
        sequence="office1-1",
        output_root=tmp_path / "dataset",
        archive=archive,
        expected_size=size,
        expected_sha256=digest,
        download=False,
    )
    assert destination.read_bytes().startswith(b"#ROSBAG")
    source = json.loads((destination.parent / "source.json").read_text())
    assert source["archive_sha256"] == digest
    assert source["bag_sha256"] == hashlib.sha256(destination.read_bytes()).hexdigest()
    assert not (tmp_path / "escape").exists()


def test_prepare_rejects_archive_size_before_extraction(tmp_path):
    archive = tmp_path / "office.tar"
    size, digest = _archive(archive)
    with pytest.raises(ValueError, match="size mismatch"):
        MODULE.prepare(
            sequence="office1-1",
            output_root=tmp_path / "dataset",
            archive=archive,
            expected_size=size + 1,
            expected_sha256=digest,
            download=False,
        )


def test_extract_rejects_unknown_sequence(tmp_path):
    archive = tmp_path / "office.tar"
    _archive(archive)
    with pytest.raises(ValueError, match="unsupported office sequence"):
        MODULE.extract_sequence(archive, "office9-9", tmp_path)


def test_download_resumes_partial_file_only_after_http_206(tmp_path, monkeypatch):
    destination = tmp_path / "archive.tar"
    partial = tmp_path / "archive.tar.part"
    partial.write_bytes(b"abc")

    class Response(io.BytesIO):
        status = 206

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            self.close()

        def getcode(self):
            return self.status

    monkeypatch.setattr(
        MODULE.urllib.request,
        "urlopen",
        lambda _request, timeout: Response(b"def"),
    )
    MODULE.download_with_resume("https://example.invalid/archive", destination, expected_size=6)
    assert destination.read_bytes() == b"abcdef"
    assert not partial.exists()


def test_prepare_first_sequence_range_records_limited_verification(tmp_path, monkeypatch):
    payload = b"#ROSBAG V2.0\nrange fixture"
    output_root = tmp_path / "dataset"
    range_archive = output_root / "archive" / "office1-1.range.tar"
    range_archive.parent.mkdir(parents=True)
    with tarfile.open(range_archive, "w") as archive:
        member = tarfile.TarInfo("office1-1.bag")
        member.size = len(payload)
        archive.addfile(member, io.BytesIO(payload))

    # 测试夹具比真实 bag 小；覆盖固定契约后，仍验证同一条流式提取与溯源路径。
    monkeypatch.setattr(MODULE, "OFFICE1_1_MEMBER_SIZE", len(payload))
    monkeypatch.setattr(MODULE, "OFFICE1_1_RANGE_END", range_archive.stat().st_size - 1)
    monkeypatch.setattr(
        MODULE,
        "OFFICE1_1_RANGE_SHA256",
        hashlib.sha256(range_archive.read_bytes()).hexdigest(),
    )
    monkeypatch.setattr(
        MODULE, "OFFICE1_1_BAG_SHA256", hashlib.sha256(payload).hexdigest()
    )

    destination = MODULE.prepare_first_sequence_range(
        output_root=output_root,
        download=False,
    )

    assert destination.read_bytes() == payload
    source = json.loads((destination.parent / "source.json").read_text())
    assert source["archive_verification"] == "pinned_https_range_not_full_hash"
    assert source["range_end"] == range_archive.stat().st_size - 1
    assert source["bag_sha256"] == hashlib.sha256(payload).hexdigest()


def test_range_extract_does_not_publish_a_bag_before_hash_verification(
    tmp_path, monkeypatch
):
    payload = b"#ROSBAG V2.0\nuntrusted fixture"
    output_root = tmp_path / "dataset"
    range_archive = output_root / "archive" / "office1-1.range.tar"
    range_archive.parent.mkdir(parents=True)
    with tarfile.open(range_archive, "w") as archive:
        member = tarfile.TarInfo("office1-1.bag")
        member.size = len(payload)
        archive.addfile(member, io.BytesIO(payload))
    monkeypatch.setattr(MODULE, "OFFICE1_1_MEMBER_SIZE", len(payload))
    monkeypatch.setattr(MODULE, "OFFICE1_1_RANGE_END", range_archive.stat().st_size - 1)
    monkeypatch.setattr(
        MODULE,
        "OFFICE1_1_RANGE_SHA256",
        hashlib.sha256(range_archive.read_bytes()).hexdigest(),
    )
    monkeypatch.setattr(MODULE, "OFFICE1_1_BAG_SHA256", "0" * 64)

    with pytest.raises(ValueError, match="bag SHA256 mismatch"):
        MODULE.prepare_first_sequence_range(output_root=output_root, download=False)

    assert not (output_root / "rosbag" / "office1-1" / "office1-1.bag").exists()


def test_parallel_range_downloader_assembles_segments_in_byte_order(tmp_path, monkeypatch):
    payload = b"0123456789abcdefghijklmnopqrstuvwxyz"
    destination = tmp_path / "range.tar"

    def fake_download(_url, part, *, start, end, chunk_size=0):
        del chunk_size
        part.parent.mkdir(parents=True, exist_ok=True)
        existing = part.stat().st_size if part.exists() else 0
        with part.open("ab") as target:
            target.write(payload[start + existing : end + 1])
        return part

    # 模拟旧版下载器留下的首段前缀，确认升级后不会丢掉已下载进度。
    destination.with_name(destination.name + ".part").write_bytes(payload[:3])
    monkeypatch.setattr(MODULE, "_download_range_part", fake_download)

    MODULE.download_range_parallel(
        "https://example.invalid/range",
        destination,
        expected_size=len(payload),
        connections=4,
    )

    assert destination.read_bytes() == payload
    assert not destination.with_name(destination.name + ".parts").exists()


def test_range_part_retries_only_missing_tail_after_short_read(tmp_path, monkeypatch):
    responses = [
        (b"abc", "bytes 0-5/99"),
        (b"def", "bytes 3-5/99"),
    ]
    requested_ranges = []

    class Response(io.BytesIO):
        status = 206

        def __init__(self, payload, content_range):
            super().__init__(payload)
            self.headers = {"Content-Range": content_range}

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            self.close()

        def getcode(self):
            return self.status

    def fake_urlopen(request, timeout):
        del timeout
        requested_ranges.append(request.get_header("Range"))
        payload, content_range = responses.pop(0)
        return Response(payload, content_range)

    monkeypatch.setattr(MODULE.urllib.request, "urlopen", fake_urlopen)
    monkeypatch.setattr(MODULE.time, "sleep", lambda _seconds: None)
    destination = tmp_path / "part"

    MODULE._download_range_part(
        "https://example.invalid/range",
        destination,
        start=0,
        end=5,
        chunk_size=2,
    )

    assert destination.read_bytes() == b"abcdef"
    assert requested_ranges == ["bytes=0-5", "bytes=3-5"]
