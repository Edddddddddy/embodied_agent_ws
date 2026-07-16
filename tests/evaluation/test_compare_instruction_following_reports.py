from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "tools" / "evaluation" / "compare_instruction_following_reports.py"
SPEC = importlib.util.spec_from_file_location("compare_instruction_following_reports", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _report(*, model_hash: str, action: float) -> dict:
    return {
        "scenario": "offline_llm_instruction_following_eval",
        "model": f"{model_hash}.gguf",
        "total": 10,
        "model_score": 0.2,
        "action_score": action,
        "protocol_score": 0.5,
        "effective_score": 0.8,
        "failure_counts": {},
        "provenance": {
            "dataset_sha256": "dataset",
            "system_prompt_sha256": "prompt",
            "model_file_sha256": model_hash,
            "temperature": 0.0,
            "max_tokens": 192,
            "seed": 42,
        },
    }


def test_comparison_separates_action_protocol_and_fallback() -> None:
    report = MODULE.compare_reports(
        _report(model_hash="baseline", action=0.3),
        _report(model_hash="tuned", action=0.6),
    )
    assert report["comparable"] is True
    assert report["delta"]["raw_action_score"] == 0.3
    assert report["delta"]["strict_protocol_and_action_score"] == 0.0
    assert "fallback" in report["interpretation"]


def test_comparison_rejects_different_eval_contract() -> None:
    baseline = _report(model_hash="baseline", action=0.3)
    tuned = _report(model_hash="tuned", action=0.6)
    tuned["provenance"]["seed"] = 7
    with pytest.raises(ValueError, match="not comparable"):
        MODULE.compare_reports(baseline, tuned)


def test_comparison_rejects_same_model_artifact() -> None:
    with pytest.raises(ValueError, match="same model"):
        MODULE.compare_reports(
            _report(model_hash="same", action=0.3),
            _report(model_hash="same", action=0.6),
        )
