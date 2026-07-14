from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "evaluate_loop_constraints", ROOT / "scripts" / "evaluate_loop_constraints.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
REVISIT_SPEC = importlib.util.spec_from_file_location(
    "analyze_openloris_revisits", ROOT / "scripts" / "analyze_openloris_revisits.py"
)
assert REVISIT_SPEC and REVISIT_SPEC.loader
REVISIT_MODULE = importlib.util.module_from_spec(REVISIT_SPEC)
REVISIT_SPEC.loader.exec_module(REVISIT_MODULE)


def _reference():
    points = [
        (0.0, 0.0),
        (1.0, 0.0),
        (2.0, 0.0),
        (3.0, 0.0),
        (3.0, 1.0),
        (3.0, 2.0),
        (2.0, 2.0),
        (1.0, 2.0),
        (0.0, 2.0),
        (0.0, 1.0),
        (0.0, 0.0),
        (0.1, 0.0),
    ]
    return [MODULE.EVALUATOR.Pose2(float(index), x, y, 0.0) for index, (x, y) in enumerate(points)]


def test_accepted_constraint_precision_and_event_recall_are_separate_metrics():
    constraints = [
        {
            "constraint_kind": "loop",
            "accepted": True,
            "source_id": 10,
            "target_id": 0,
            "source_stamp_s": 10.0,
            "target_stamp_s": 0.0,
        },
        {
            "constraint_kind": "loop",
            "accepted": True,
            "source_id": 8,
            "target_id": 0,
            "source_stamp_s": 8.0,
            "target_stamp_s": 0.0,
        },
    ]
    config = MODULE.EVALUATOR.EvaluationConfig(
        loop_radius_m=0.25,
        loop_min_separation_s=8.0,
        loop_sample_interval_s=1.0,
        loop_event_gap_s=2.0,
    )

    report = MODULE.evaluate_constraints(_reference(), constraints, config=config)

    assert report["ground_truth"]["revisit_events"] == 1
    assert report["accepted_constraints"]["true_positive"] == 1
    assert report["accepted_constraints"]["false_positive"] == 1
    assert report["accepted_constraints"]["precision"] == 0.5
    assert report["event_recovery"]["event_recall"] == 1.0


def test_loader_ignores_sequential_edges_and_rejects_malformed_rows(tmp_path):
    path = tmp_path / "constraints.jsonl"
    path.write_text(
        json.dumps(
            {
                "constraint_kind": "sequential",
                "accepted": True,
                "source_stamp_s": 1.0,
                "target_stamp_s": 0.0,
            }
        )
        + "\n"
        + json.dumps(
            {
                "constraint_kind": "loop",
                "accepted": True,
                "source_stamp_s": 10.0,
                "target_stamp_s": 0.0,
            }
        )
        + "\n",
        encoding="utf-8",
    )
    assert len(MODULE.load_constraints(path)) == 1

    path.write_text("{}\n", encoding="utf-8")
    with pytest.raises(ValueError, match="invalid loop constraint"):
        MODULE.load_constraints(path)


def test_ground_truth_revisit_catalog_does_not_claim_slam_recovery(tmp_path):
    reference = _reference()
    path = tmp_path / "groundtruth.tum"
    path.write_text(
        "\n".join(
            f"{pose.stamp} {pose.x} {pose.y} 0 0 0 0 1" for pose in reference
        )
        + "\n",
        encoding="utf-8",
    )
    config = REVISIT_MODULE.EVALUATOR.EvaluationConfig(
        loop_radius_m=0.25,
        loop_min_separation_s=8.0,
        loop_sample_interval_s=1.0,
        loop_event_gap_s=2.0,
    )

    report = REVISIT_MODULE.analyze(path, config, min_events=1)

    assert report["passed"] is True
    assert report["revisit_catalog"]["event_count"] == 1
    assert "recovered" not in report["revisit_catalog"]["events"][0]
