from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "run_gtsam_scan_overlap_ablation",
    ROOT / "scripts" / "run_gtsam_scan_overlap_ablation.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _record(
    name: str,
    ate: float,
    *,
    consistency_gate: bool = False,
    overlap_gate: bool = False,
    overlap_unavailable: int = 0,
) -> dict:
    return {
        "name": name,
        "graph_sha256": "fixed-graph",
        "optimizer": {
            "nodes": 100,
            "constraints": 120,
            "constraints_used": 118,
            "consistency_gate": consistency_gate,
            "consistency_rejected_constraints": 1 if consistency_gate else 0,
            "scan_overlap_gate": overlap_gate,
            "minimum_scan_overlap_ratio": 0.65,
            "scan_overlap_gate_min_translation_residual_m": 1.0,
            "scan_overlap_evaluated_constraints": 20 if overlap_gate else 0,
            "scan_overlap_rejected_constraints": 1 if overlap_gate else 0,
            "scan_overlap_unavailable_constraints": overlap_unavailable,
        },
        "evaluation": {
            "passed": True,
            "association": {"matched_poses": 100},
            "ate_xy_m": {"rmse": ate, "p95": ate * 1.5},
            "rpe": {"translation_m": {"rmse": ate / 2.0}},
            "closure": {"final_pose_error_m": ate * 2.0},
        },
    }


def test_comparison_preserves_fixed_graph_and_selects_measured_winner():
    records = [
        _record("cauchy_baseline", 1.2),
        _record("naive_overlap", 1.3, overlap_gate=True),
        _record("innovation_gate", 1.1, consistency_gate=True),
        _record("dual_evidence", 1.0, consistency_gate=True, overlap_gate=True),
    ]

    report = MODULE.build_comparison(records, "fixed-graph")

    assert report["passed"] is True
    assert report["best_ate_variant"] == "dual_evidence"
    assert report["variants"][-1]["ate_change_vs_baseline_pct"] < 0.0
    assert "without ground truth" in report["interpretation_boundary"]


def test_comparison_fails_when_overlap_evidence_is_missing():
    records = [
        _record("cauchy_baseline", 1.2),
        _record("dual_evidence", 1.0, overlap_gate=True, overlap_unavailable=2),
    ]

    report = MODULE.build_comparison(records, "fixed-graph")

    assert report["passed"] is False
    assert report["checks"]["all_nonlocal_edges_have_overlap_evidence"] is False


def test_published_scan_overlap_evidence_is_truth_free_and_conservative():
    evidence = json.loads(
        (ROOT / "docs" / "evidence" / "gtsam_scan_overlap_ablation.json").read_text(
            encoding="utf-8"
        )
    )

    assert evidence["passed"] is True
    assert all(evidence["checks"].values())
    assert evidence["fixed_graph_contract"]["nodes"] == 1834
    assert evidence["scan_evidence"]["scan_messages"] == 11381
    assert evidence["scan_evidence"]["scored_nonlocal_constraints"] == 858
    assert evidence["scan_evidence"]["unavailable_nonlocal_constraints"] == 0
    assert evidence["gate_contract"]["uses_ground_truth_at_runtime"] is False
    assert evidence["gate_contract"]["default_enabled"] is False
    baseline = next(item for item in evidence["variants"] if item["name"] == "cauchy_baseline")
    dual = next(item for item in evidence["variants"] if item["name"] == "dual_evidence")
    naive = next(item for item in evidence["variants"] if item["name"] == "naive_overlap")
    assert dual["scan_overlap_rejected_constraints"] == 11
    assert dual["ate_rmse_m"] < baseline["ate_rmse_m"]
    assert naive["scan_overlap_rejected_constraints"] > 20 * dual[
        "scan_overlap_rejected_constraints"
    ]
    assert "neither generates candidates" in evidence["interpretation_boundary"]
