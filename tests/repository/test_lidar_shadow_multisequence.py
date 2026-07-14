from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "compare_lidar_shadow_match_sequences",
    ROOT / "scripts" / "compare_lidar_shadow_match_sequences.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _report(sequence: str, digest: str, precision: float) -> dict:
    metrics = {
        "accepted_pairs": 20,
        "true_accepted_pairs": 5,
        "false_accepted_pairs": 15,
        "precision": precision,
        "conditional_pair_recall": 0.25,
        "eligible_query_recall": 0.2,
        "relative_translation_error_m": {"median": 0.3},
        "relative_yaw_error_deg": {"median": 2.0},
    }
    return {
        "passed": True,
        "sequence": sequence,
        "association": {"scored_pairs": 100},
        "ground_truth": {
            "true_candidate_pairs": 20,
            "minimum_temporal_separation_s": 60.0,
            "revisit_radius_m": 1.0,
        },
        "profiles": {
            "cpp_default": metrics,
            "balanced_shadow": metrics,
            "conservative_shadow": metrics,
        },
        "event_recovery": {"recovered_events": 1, "event_count": 2},
        "source": {"matches_sha256": digest},
    }


def test_low_precision_evidence_passes_contract_but_blocks_graph_edges():
    report = MODULE.build_summary(
        [
            ("one", _report("one", "a" * 64, 0.35)),
            ("two", _report("two", "b" * 64, 0.10)),
        ]
    )

    assert report["passed"] is True
    assert report["release_decision"]["guarded_graph_edge_ablation_ready"] is False
    assert report["release_decision"]["direct_graph_edge_insertion_enabled"] is False


def test_only_consistent_multisequence_quality_can_unlock_future_ablation():
    report = MODULE.build_summary(
        [
            ("one", _report("one", "a" * 64, 0.85)),
            ("two", _report("two", "b" * 64, 0.90)),
        ]
    )

    assert report["release_decision"]["guarded_graph_edge_ablation_ready"] is True
    assert report["release_decision"]["direct_graph_edge_insertion_enabled"] is False


def test_published_real_evidence_remains_shadow_only():
    report = json.loads(
        (
            ROOT
            / "docs"
            / "evidence"
            / "lidar_shadow_matches_multisequence.json"
        ).read_text(encoding="utf-8")
    )

    assert report["passed"] is True
    assert report["sequence_count"] == 2
    assert report["release_decision"]["guarded_graph_edge_ablation_ready"] is False
    assert report["release_decision"]["direct_graph_edge_insertion_enabled"] is False
