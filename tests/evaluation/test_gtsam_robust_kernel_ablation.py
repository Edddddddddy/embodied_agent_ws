from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "run_gtsam_robust_kernel_ablation",
    ROOT / "tools" / "evaluation" / "run_gtsam_robust_kernel_ablation.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _variant(name: str, ate: float, *, nodes: int = 80, matched: int = 80) -> dict:
    return {
        "name": name,
        "graph_sha256": "fixed-graph",
        "optimizer": {
            "kernel": "none" if name == "gaussian" else "huber",
            "kernel_k": 0.0 if name == "gaussian" else 1.345,
            "loop_only": name.endswith("loop"),
            "nodes": nodes,
            "constraints": 90,
            "robustified_constraints": 0 if name == "gaussian" else 10,
        },
        "evaluation": {
            "passed": True,
            "association": {"matched_poses": matched},
            "ate_xy_m": {"rmse": ate, "p95": ate * 1.5},
            "rpe": {"translation_m": {"rmse": ate / 2.0}},
            "closure": {"final_pose_error_m": ate * 2.0},
        },
    }


def test_fixed_graph_comparison_selects_measured_winner_without_preselection():
    variants = [
        _variant("gaussian", 1.2),
        _variant("huber_all", 1.1),
        _variant("huber_loop", 0.8),
        _variant("cauchy_loop", 0.9),
    ]

    report = MODULE.build_comparison(variants, "fixed-graph")

    assert report["passed"] is True
    assert report["best_ate_variant"] == "huber_loop"
    assert report["variants"][2]["ate_change_vs_gaussian_pct"] < 0.0
    assert "does not improve" in report["interpretation_boundary"]


def test_comparison_rejects_different_graph_or_temporal_association():
    variants = [_variant("gaussian", 1.2), _variant("huber_all", 1.0, matched=79)]
    variants[1]["graph_sha256"] = "another-graph"

    report = MODULE.build_comparison(variants, "fixed-graph")

    assert report["passed"] is False
    assert report["checks"]["same_graph_sha256"] is False
    assert report["checks"]["same_matched_pose_count"] is False


def test_comparison_allows_gate_to_use_fewer_edges_from_the_same_input_graph():
    variants = [_variant("gaussian", 1.2), _variant("cauchy_loop_gated", 0.9)]
    variants[1]["optimizer"].update(
        {
            "constraints_used": 80,
            "consistency_gate": True,
            "consistency_rejected_constraints": 10,
        }
    )

    report = MODULE.build_comparison(variants, "fixed-graph")

    assert report["passed"] is True
    assert report["checks"]["same_constraint_count"] is True
    assert report["variants"][1]["constraints_used"] == 80


def test_comparison_reports_switchable_constraint_state_without_changing_graph_size():
    variants = [_variant("gaussian", 1.2), _variant("switchable_gaussian", 0.7)]
    variants[1]["optimizer"].update(
        {
            "switchable_loop_constraints": True,
            "switchable_constraints": 12,
            "switch_suppressed_constraints": 3,
            "minimum_switch_value": 0.02,
            "mean_switch_value": 0.76,
        }
    )

    report = MODULE.build_comparison(variants, "fixed-graph")

    assert report["passed"] is True
    switchable = report["variants"][1]
    assert switchable["switchable_loop_constraints"] is True
    assert switchable["switch_suppressed_constraints"] == 3
    assert switchable["minimum_switch_value"] == 0.02


def test_published_real_data_evidence_preserves_fairness_and_claim_boundary():
    evidence = json.loads(
        (ROOT / "docs" / "evidence" / "slam" / "gtsam_robust_kernel_ablation.json").read_text(
            encoding="utf-8"
        )
    )

    assert evidence["passed"] is True
    assert all(evidence["checks"].values())
    assert evidence["fixed_graph_contract"]["nodes"] == 1834
    assert evidence["fixed_graph_contract"]["dangling_constraints"] == 0
    assert evidence["best_ate_variant"] == "cauchy_loop"
    assert "heuristic" in evidence["interpretation_boundary"]
    assert "does not improve" in evidence["interpretation_boundary"]


def test_published_consistency_gate_evidence_is_fair_and_keeps_runtime_truth_free():
    evidence = json.loads(
        (ROOT / "docs" / "evidence" / "slam" / "gtsam_loop_consistency_ablation.json").read_text(
            encoding="utf-8"
        )
    )

    assert evidence["passed"] is True
    assert all(evidence["checks"].values())
    assert evidence["fixed_graph_contract"]["sha256"] == (
        "792ecb7d1871b506274423631fdf16da60b52e46aa18eaa1287cddbd4cadbc65"
    )
    assert evidence["gate_contract"]["uses_ground_truth_at_runtime"] is False
    assert evidence["gate_contract"]["rejected_constraints"] == 23
    assert evidence["best_ate_variant"] == "cauchy_loop_gated"
    gated = next(item for item in evidence["variants"] if item["consistency_gate"])
    assert gated["constraints_used"] == 2751 - 23
    assert gated["ate_rmse_m"] < 1.223586
    assert "disabled by default" in evidence["interpretation_boundary"]
