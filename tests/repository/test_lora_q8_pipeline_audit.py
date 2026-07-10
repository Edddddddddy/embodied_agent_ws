from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "audit_lora_q8_pipeline", ROOT / "scripts" / "audit_lora_q8_pipeline.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_pipeline_audit_distinguishes_ready_from_reproduced(tmp_path) -> None:
    missing_f16 = tmp_path / "missing-f16.gguf"
    missing_q8 = tmp_path / "missing-q8.gguf"
    report = MODULE.build_report(missing_f16, missing_q8)
    assert report["static_pipeline_ready"] is True
    assert report["status"] == "pipeline_ready_not_executed"
    assert report["reproduced_training"] is False


def test_pipeline_audit_uses_actual_artifact_sizes(tmp_path) -> None:
    f16 = tmp_path / "model-f16.gguf"
    q8 = tmp_path / "model-q8.gguf"
    f16.write_bytes(b"GGUF" + bytes(996))
    q8.write_bytes(b"GGUF" + bytes(496))
    report = MODULE.build_report(f16, q8)
    assert report["status"] == "artifacts_verified_training_unproven"
    assert report["q8_to_f16_ratio"] == 0.5
    assert report["artifacts_verified"] is True
    assert report["reproduced_training"] is False
