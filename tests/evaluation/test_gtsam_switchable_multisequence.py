from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "compare_gtsam_switchable_sequences",
    ROOT / "tools" / "evaluation" / "compare_gtsam_switchable_sequences.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _report(graph: str, *, gaussian: float, cauchy: float, switchable: float, off: int) -> dict:
    def row(name: str, ate: float) -> dict:
        return {
            "name": name,
            "ate_rmse_m": ate,
            "switch_prior_sigma": 1.0,
            "switch_suppression_threshold": 0.5,
            "switchable_constraints": 10,
            "switch_suppressed_constraints": off if name == "switchable_cauchy" else 0,
            "minimum_switch_value": 0.01 if off else 0.99,
            "mean_switch_value": 0.8 if off else 0.99,
        }

    return {
        "passed": True,
        "graph_sha256": graph,
        "graph_nodes": 100,
        "graph_constraints": 120,
        "matched_poses_per_variant": 90,
        "variants": [
            row("gaussian", gaussian),
            row("cauchy_loop", cauchy),
            row("switchable_gaussian", switchable),
            row("switchable_cauchy", switchable),
        ],
    }


def test_multisequence_switchable_comparison_requires_improvement_without_forcing_switch_off():
    report = MODULE.compare(
        [
            ("a", _report("graph-a", gaussian=1.0, cauchy=0.8, switchable=0.7, off=2)),
            ("b", _report("graph-b", gaussian=0.2, cauchy=0.2, switchable=0.2, off=0)),
        ]
    )

    assert report["passed"] is True
    assert report["checks"]["observed_suppressed_loop"] is True
    assert report["checks"]["observed_retained_loop"] is True
    assert report["aggregate"]["switch_suppressed_constraints"] == 2


def test_multisequence_switchable_comparison_rejects_regression_or_reused_graph():
    report = MODULE.compare(
        [
            ("a", _report("same", gaussian=1.0, cauchy=0.8, switchable=0.9, off=1)),
            ("b", _report("same", gaussian=0.2, cauchy=0.2, switchable=0.2, off=0)),
        ]
    )

    assert report["passed"] is False
    assert report["checks"]["independent_graph_snapshots"] is False
    assert report["checks"]["no_per_sequence_ate_regression_vs_cauchy"] is False


def test_multisequence_switchable_comparison_requires_two_reports():
    with pytest.raises(ValueError, match="at least two"):
        MODULE.compare(
            [("a", _report("graph-a", gaussian=1.0, cauchy=0.8, switchable=0.7, off=1))]
        )


def test_published_switchable_evidence_keeps_real_data_and_claim_boundary():
    evidence = json.loads(
        (ROOT / "docs" / "evidence" / "slam" / "gtsam_switchable_multisequence.json").read_text(
            encoding="utf-8"
        )
    )

    assert evidence["passed"] is True
    assert all(evidence["checks"].values())
    assert len(evidence["sequences"]) == 2
    assert evidence["aggregate"]["switchable_constraints"] == 859
    assert evidence["aggregate"]["switch_suppressed_constraints"] == 80
    assert evidence["aggregate"]["switchable_cauchy_ate_rmse_m"] < evidence["aggregate"][
        "cauchy_loop_ate_rmse_m"
    ]
    assert "do not prove" in evidence["interpretation_boundary"]
