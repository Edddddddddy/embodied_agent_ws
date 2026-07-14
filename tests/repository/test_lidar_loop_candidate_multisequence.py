from __future__ import annotations

import importlib.util
from pathlib import Path

import json


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "compare_lidar_loop_candidate_sequences",
    ROOT / "scripts" / "compare_lidar_loop_candidate_sequences.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _report(sequence: str, digest: str, recall: float, rerank: float) -> dict:
    def metrics(value: float) -> dict:
        return {
            "top_k_metrics": {
                "at_10": {"query_recall": value, "candidate_precision": 0.05}
            },
            "mean_reciprocal_rank": value / 2.0,
        }
    return {
        "passed": True,
        "quality_checks": {"recall": True, "event": True},
        "sequence": sequence,
        "source": {"corpus_sha256": digest, "graph_sha256": digest[::-1]},
        "association": {"candidate_rows": 200},
        "ground_truth": {
            "eligible_long_revisit_queries": 20,
            "minimum_temporal_separation_s": 60.0,
            "revisit_radius_m": 1.0,
        },
        "retrieval": {
            "ring_key_retrieval": metrics(recall),
            "similarity_rerank_ablation": metrics(rerank),
        },
        "event_recovery": {"event_recall_at_max_k": 1.0},
    }


def test_multisequence_summary_allows_shadow_mode_but_not_direct_edges():
    report = MODULE.build_summary(
        [
            ("one", _report("one", "a" * 64, 0.4, 0.3)),
            ("two", _report("two", "b" * 64, 0.5, 0.2)),
        ]
    )

    assert report["passed"] is True
    assert report["release_decision"]["shadow_scan_match_integration_ready"] is True
    assert report["release_decision"]["direct_graph_edge_insertion_enabled"] is False


def test_duplicate_corpus_or_one_sequence_regression_fails_contract():
    report = MODULE.build_summary(
        [
            ("one", _report("one", "a" * 64, 0.4, 0.3)),
            ("two", _report("two", "a" * 64, 0.2, 0.3)),
        ]
    )

    assert report["passed"] is False
    assert report["checks"]["distinct_scan_corpora"] is False
    assert report["checks"]["ring_key_beats_similarity_rerank_every_sequence"] is False


def test_published_evidence_stays_shadow_only_despite_full_event_recall():
    evidence = json.loads(
        (ROOT / "docs" / "evidence" / "lidar_loop_candidates_multisequence.json").read_text(
            encoding="utf-8"
        )
    )

    assert evidence["passed"] is True
    assert evidence["aggregate"]["all_events_recovered_at_10"] is True
    assert evidence["release_decision"]["shadow_scan_match_integration_ready"] is True
    assert evidence["release_decision"]["direct_graph_edge_insertion_enabled"] is False
