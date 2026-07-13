from __future__ import annotations

import hashlib
import importlib.util
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
