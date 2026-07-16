from __future__ import annotations

import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "analyze_slam_degradation", ROOT / "tools" / "evaluation" / "analyze_slam_degradation.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
Pose2 = MODULE.EVALUATOR.Pose2


def _trajectories():
    reference = []
    estimate = []
    for index in range(30):
        stamp = float(index)
        if index < 10:
            x, y, yaw = index * 0.2, 0.0, 0.0
        elif index < 20:
            angle = (index - 10) * 0.3
            x, y, yaw = 2.0 + math.sin(angle), 1.0 - math.cos(angle), angle
        else:
            x, y, yaw = 3.0, 2.0, 2.0
        reference.append(Pose2(stamp, x, y, yaw))
        error = 0.01 * index if 10 <= index < 20 else 0.01
        estimate.append(Pose2(stamp, x + error, y, yaw))
    return reference, estimate


def test_analysis_separates_motion_and_requires_labels_for_dynamic_occlusion():
    reference, estimate = _trajectories()
    report = MODULE.analyze(reference, estimate)
    assert report["passed"] is True
    assert report["motion_classes"]["straight"]["samples"] > 0
    assert report["motion_classes"]["turning"]["samples"] > 0
    assert report["motion_classes"]["stationary"]["samples"] > 0
    total_duration = sum(
        item["duration_s"] for item in report["motion_classes"].values()
    )
    assert total_duration == 29.0
    assert report["dynamic_occlusion_evidence"]["available"] is False


def test_analysis_reports_manually_reviewed_dynamic_interval():
    reference, estimate = _trajectories()
    report = MODULE.analyze(
        reference,
        estimate,
        annotations={
            "intervals": [
                {"label": "dynamic_occlusion", "start_s": 12.0, "end_s": 16.0}
            ]
        },
    )
    assert report["dynamic_occlusion_evidence"]["available"] is True
    assert report["labelled_intervals"][0]["samples"] == 5
    assert report["labelled_intervals"][0]["ate_rmse_m"] is not None
