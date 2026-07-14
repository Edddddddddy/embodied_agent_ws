from __future__ import annotations

import hashlib
import importlib.util
import io
import json
import zipfile
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "setup_openloris_groundtruth", ROOT / "scripts" / "setup_openloris_groundtruth.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _archive(path: Path) -> str:
    with zipfile.ZipFile(path, "w") as zipped:
        zipped.writestr("per-sequence/office1-1/groundtruth.txt", "# gt\n1 0 0 0 0 0 0 1\n")
        zipped.writestr("per-sequence/market1-3/groundtruth.txt", "# gt\n1 0 0 0 0 0 0 1\n")
        zipped.writestr("../../escape.txt", "must not be extracted")
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_prepare_verifies_archive_and_extracts_only_requested_member(tmp_path):
    archive = tmp_path / "groundtruth.zip"
    expected_hash = _archive(archive)
    output = tmp_path / "dataset"
    destination = MODULE.prepare(
        sequence="office1-1",
        output_root=output,
        archive=archive,
        expected_sha256=expected_hash,
    )
    assert destination.read_text(encoding="utf-8").startswith("# gt")
    assert (destination.parent / "source.json").is_file()
    source = json.loads((destination.parent / "source.json").read_text())
    assert source["ground_truth_kind"] == "optitrack_motion_capture"
    assert source["independent_of_online_hokuyo"] is True
    assert not (tmp_path / "escape.txt").exists()


def test_prepare_rejects_wrong_checksum(tmp_path):
    archive = tmp_path / "groundtruth.zip"
    _archive(archive)
    with pytest.raises(ValueError, match="SHA256 mismatch"):
        MODULE.prepare(
            sequence="office1-1",
            output_root=tmp_path / "dataset",
            archive=archive,
            expected_sha256="0" * 64,
        )


def test_non_office_groundtruth_metadata_discloses_lidar_derivation(tmp_path):
    archive = tmp_path / "groundtruth.zip"
    expected_hash = _archive(archive)
    destination = MODULE.prepare(
        sequence="market1-3",
        output_root=tmp_path / "dataset",
        archive=archive,
        expected_sha256=expected_hash,
    )
    source = json.loads((destination.parent / "source.json").read_text())
    assert source["ground_truth_kind"] == "official_offline_lidar_slam"
    assert source["independent_of_online_hokuyo"] is False


def test_download_resumes_only_missing_tail_after_short_response(tmp_path, monkeypatch):
    destination = tmp_path / "groundtruth.zip"
    responses = [
        (200, b"abc", {}),
        (206, b"def", {"Content-Range": "bytes 3-5/6"}),
    ]
    requested_ranges: list[str | None] = []

    class Response(io.BytesIO):
        def __init__(self, status, payload, headers):
            super().__init__(payload)
            self.status = status
            self.headers = headers

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            self.close()

        def getcode(self):
            return self.status

    def fake_urlopen(request, timeout):
        del timeout
        requested_ranges.append(request.get_header("Range"))
        status, payload, headers = responses.pop(0)
        return Response(status, payload, headers)

    monkeypatch.setattr(MODULE.urllib.request, "urlopen", fake_urlopen)
    monkeypatch.setattr(MODULE.time, "sleep", lambda _seconds: None)

    MODULE.download_with_resume(
        "https://example.invalid/groundtruth.zip",
        destination,
        expected_size=6,
        chunk_size=2,
    )

    assert destination.read_bytes() == b"abcdef"
    assert requested_ranges == [None, "bytes=3-5"]
    assert not destination.with_name(destination.name + ".part").exists()


def test_download_rejects_discontinuous_range_response(tmp_path, monkeypatch):
    destination = tmp_path / "groundtruth.zip"
    destination.with_name(destination.name + ".part").write_bytes(b"abc")

    class Response(io.BytesIO):
        status = 206
        headers = {"Content-Range": "bytes 4-5/6"}

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

    with pytest.raises(ValueError, match="unexpected Content-Range"):
        MODULE.download_with_resume(
            "https://example.invalid/groundtruth.zip",
            destination,
            expected_size=6,
        )

    assert not destination.exists()
    assert destination.with_name(destination.name + ".part").read_bytes() == b"abc"


def test_prepare_reuses_verified_default_archive_cache(tmp_path, monkeypatch):
    fixture = tmp_path / "fixture.zip"
    expected_hash = _archive(fixture)
    downloads = 0

    def fake_download(_url, destination, **_kwargs):
        nonlocal downloads
        downloads += 1
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(fixture.read_bytes())
        return destination

    monkeypatch.setattr(MODULE, "download_with_resume", fake_download)
    output = tmp_path / "dataset"
    for _ in range(2):
        MODULE.prepare(
            sequence="office1-1",
            output_root=output,
            expected_sha256=expected_hash,
        )

    assert downloads == 1
    assert (output / "archive" / "groundtruth.zip").is_file()
