from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "compare_gtsam_scan_overlap_sequences",
    ROOT / "tools" / "evaluation" / "compare_gtsam_scan_overlap_sequences.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _report(digest: str, *, baseline: float, dual: float, dual_p95: float) -> dict:
    def row(name: str, ate: float, p95: float, rejected: int = 0) -> dict:
        return {
            "name": name,
            "minimum_scan_overlap_ratio": 0.65,
            "minimum_translation_residual_m": 1.0,
            "ate_rmse_m": ate,
            "ate_p95_m": p95,
            "scan_overlap_rejected_constraints": rejected,
            "scan_overlap_unavailable_constraints": 0,
        }

    return {
        "passed": True,
        "graph_sha256": digest,
        "graph_nodes": 100,
        "graph_constraints": 140,
        "scan_evidence": {"unavailable_nonlocal_constraints": 0},
        "variants": [
            row("cauchy_baseline", baseline, baseline * 1.5),
            row("naive_overlap", baseline * 0.9, baseline * 1.4, rejected=30),
            row("innovation_gate", baseline * 0.95, baseline * 1.45, rejected=0),
            row("dual_evidence", dual, dual_p95, rejected=3),
        ],
    }


def test_multisequence_summary_enables_only_for_consistent_conservative_gain():
    report = MODULE.build_summary(
        [
            ("corridor1-1", _report("a" * 64, baseline=1.0, dual=0.9, dual_p95=1.49)),
            ("corridor1-2", _report("b" * 64, baseline=2.0, dual=1.8, dual_p95=2.98)),
        ]
    )

    assert report["passed"] is True
    assert report["aggregate"]["dual_improves_all_sequence_ate"] is True
    assert report["release_decision"]["recommended_default_enabled"] is True
    assert "ground truth only for offline" in report["claim_boundary"]


def test_one_sequence_regression_is_not_hidden_by_positive_mean():
    report = MODULE.build_summary(
        [
            ("corridor1-1", _report("a" * 64, baseline=2.0, dual=1.0, dual_p95=2.9)),
            ("corridor1-2", _report("b" * 64, baseline=1.0, dual=1.1, dual_p95=1.4)),
        ]
    )

    assert report["aggregate"]["mean_dual_ate_change_vs_baseline_pct"] < 0.0
    assert report["aggregate"]["dual_improves_all_sequence_ate"] is False
    assert report["release_decision"]["recommended_default_enabled"] is False


def test_duplicate_fixed_graph_or_threshold_mismatch_fails_contract():
    first = _report("a" * 64, baseline=1.0, dual=0.9, dual_p95=1.49)
    second = _report("a" * 64, baseline=1.1, dual=1.0, dual_p95=1.6)
    second["variants"][-1]["minimum_scan_overlap_ratio"] = 0.70

    report = MODULE.build_summary([("one", first), ("two", second)])

    assert report["passed"] is False
    assert report["checks"]["distinct_fixed_graphs"] is False
    assert report["checks"]["same_gate_thresholds"] is False


def test_missing_required_variant_is_rejected():
    report = _report("a" * 64, baseline=1.0, dual=0.9, dual_p95=1.49)
    report["variants"] = report["variants"][:-1]

    with pytest.raises(ValueError, match="missing variants"):
        MODULE.build_summary([("corridor1-1", report)])


def test_published_multisequence_evidence_keeps_gate_disabled_when_second_graph_is_inconclusive():
    import json

    evidence = json.loads(
        (ROOT / "docs" / "evidence" / "gtsam_scan_overlap_multisequence.json").read_text(
            encoding="utf-8"
        )
    )
    rows = {row["sequence"]: row for row in evidence["sequences"]}

    assert evidence["passed"] is True
    assert evidence["sequence_count"] == 2
    assert rows["corridor1-1"]["dual_ate_change_vs_baseline_pct"] < 0.0
    assert rows["corridor1-2"]["dual_ate_change_vs_baseline_pct"] == 0.0
    assert evidence["aggregate"]["dual_improves_all_sequence_ate"] is False
    assert evidence["release_decision"]["recommended_default_enabled"] is False
    assert evidence["release_decision"]["status"] == "keep_disabled_collect_more_sequences"
