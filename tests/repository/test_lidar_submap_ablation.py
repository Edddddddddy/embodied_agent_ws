from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "compare_lidar_submap_ablation",
    ROOT / "scripts" / "compare_lidar_submap_ablation.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _report(sequence: str, mode: str, precision: float, translation: float) -> dict:
    metrics = {
        "accepted_pairs": 20,
        "precision": precision,
        "conditional_pair_recall": 0.25,
        "relative_translation_error_m": {"median": translation},
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
        "profiles": {"cpp_default": metrics},
        "methodology": {"matching_mode": mode},
        "source": {
            "groundtruth_sha256": f"gt-{sequence}",
            "candidates_sha256": f"candidate-{sequence}",
            "corpus_metadata_sha256": f"corpus-{sequence}",
        },
    }


def test_relative_improvement_does_not_enable_graph_edges():
    baselines = [
        (name, _report(name, "scan_to_scan", 0.20, 1.0)) for name in ("one", "two")
    ]
    submaps = [
        (name, _report(name, "scan_to_submap", 0.40, 0.7)) for name in ("one", "two")
    ]

    report = MODULE.build_summary(baselines, submaps)

    assert report["passed"] is True
    assert report["aggregate"]["mean_precision_delta"] == 0.2
    assert report["release_decision"]["guarded_graph_edge_ablation_ready"] is False
    assert report["release_decision"]["direct_graph_edge_insertion_enabled"] is False


def test_ablation_rejects_different_candidate_source():
    baselines = [
        (name, _report(name, "scan_to_scan", 0.85, 0.3)) for name in ("one", "two")
    ]
    submaps = [
        (name, _report(name, "scan_to_submap", 0.85, 0.3)) for name in ("one", "two")
    ]
    submaps[0][1]["source"]["candidates_sha256"] = "mixed-source"

    report = MODULE.build_summary(baselines, submaps)

    assert report["passed"] is False
    assert report["release_decision"]["direct_graph_edge_insertion_enabled"] is False


def test_ablation_rejects_different_effective_point_sampling():
    baselines = [
        (name, _report(name, "scan_to_scan", 0.85, 0.3)) for name in ("one", "two")
    ]
    submaps = [
        (name, _report(name, "scan_to_submap", 0.85, 0.3)) for name in ("one", "two")
    ]
    baselines[0][1]["methodology"]["geometry_sampling"] = {
        "effective_point_stride_per_scan": 2,
        "matcher_minimum_points": 30,
    }
    submaps[0][1]["methodology"]["geometry_sampling"] = {
        "effective_point_stride_per_scan": 4,
        "matcher_minimum_points": 30,
    }

    report = MODULE.build_summary(baselines, submaps)

    assert report["passed"] is False


def test_published_submap_evidence_remains_shadow_only():
    path = ROOT / "docs" / "evidence" / "lidar_submap_ablation_multisequence.json"
    assert path.exists()
    report = json.loads(path.read_text(encoding="utf-8"))
    assert report["passed"] is True
    assert report["sequence_count"] == 2
    assert all(row["contract"]["same_source"] for row in report["sequences"])
    assert all(
        row["contract"]["same_effective_sampling"] for row in report["sequences"]
    )
    assert (
        report["release_decision"]["status"]
        == "shadow_only_submap_quality_insufficient"
    )
    assert report["release_decision"]["direct_graph_edge_insertion_enabled"] is False
