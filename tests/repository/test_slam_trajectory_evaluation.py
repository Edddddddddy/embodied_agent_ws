from __future__ import annotations

import importlib.util
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "evaluate_slam_trajectory", ROOT / "scripts" / "evaluate_slam_trajectory.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _trajectory(count: int = 80, step: float = 0.1):
    return [MODULE.Pose2(index * step, index * 0.05, math.sin(index * 0.08), 0.1) for index in range(count)]


def _rigid_transform(poses, angle=0.4, tx=2.0, ty=-1.0):
    cosine, sine = math.cos(angle), math.sin(angle)
    return [
        MODULE.Pose2(
            pose.stamp,
            cosine * pose.x - sine * pose.y + tx,
            sine * pose.x + cosine * pose.y + ty,
            pose.yaw + angle,
        )
        for pose in poses
    ]


def test_rigid_alignment_removes_only_global_frame_difference():
    reference = _trajectory()
    report = MODULE.evaluate(reference, _rigid_transform(reference), MODULE.EvaluationConfig())
    assert report["passed"] is True
    assert report["alignment"]["scale"] == 1.0
    assert report["ate_xy_m"]["rmse"] < 1e-6
    assert report["rpe"]["translation_m"]["rmse"] < 1e-6


def test_association_interpolates_a_small_timestamp_offset():
    reference = _trajectory()
    estimate = [
        MODULE.Pose2(pose.stamp + 0.02, pose.x + 1.0, pose.y - 2.0, pose.yaw)
        for pose in reference[:-1]
    ]
    report = MODULE.evaluate(
        reference, estimate, MODULE.EvaluationConfig(max_time_diff_s=0.06)
    )
    assert report["association"]["estimate_match_ratio"] == 1.0
    assert report["ate_xy_m"]["rmse"] < 0.03


def test_scale_drift_is_not_hidden_by_alignment():
    reference = _trajectory()
    scaled = [MODULE.Pose2(p.stamp, p.x * 1.12, p.y * 1.12, p.yaw) for p in reference]
    report = MODULE.evaluate(reference, scaled, MODULE.EvaluationConfig())
    assert report["alignment"]["scale"] == 1.0
    assert report["ate_xy_m"]["rmse"] > 0.1
    assert report["path"]["length_ratio"] > 1.10


def test_rpe_exposes_local_drift_and_threshold_can_fail_gate():
    reference = _trajectory()
    drifted = [
        MODULE.Pose2(p.stamp, p.x + 0.01 * i, p.y + 0.002 * i * i, p.yaw + 0.003 * i)
        for i, p in enumerate(reference)
    ]
    report = MODULE.evaluate(
        reference,
        drifted,
        MODULE.EvaluationConfig(max_rpe_translation_rmse_m=0.01),
    )
    assert report["rpe"]["translation_m"]["rmse"] > 0.01
    assert report["checks"]["rpe_translation_within_limit"] is False
    assert report["passed"] is False


def test_closed_loop_reports_recovered_and_missed_return_events():
    reference = [
        MODULE.Pose2(0.0, 0.0, 0.0, 0.0),
        MODULE.Pose2(5.0, 1.0, 0.0, 0.0),
        MODULE.Pose2(10.0, 1.0, 1.0, math.pi),
        MODULE.Pose2(15.0, 0.0, 1.0, -math.pi / 2),
        MODULE.Pose2(20.0, 0.0, 0.0, 0.0),
    ]
    recovered = _rigid_transform(reference)
    recovered_report = MODULE.evaluate(
        reference,
        recovered,
        MODULE.EvaluationConfig(
            max_time_diff_s=0.1,
            loop_min_separation_s=15.0,
            loop_sample_interval_s=1.0,
        ),
    )
    missed = list(recovered)
    last = missed[-1]
    missed[-1] = MODULE.Pose2(last.stamp, last.x + 1.0, last.y, last.yaw)
    missed_report = MODULE.evaluate(
        reference,
        missed,
        MODULE.EvaluationConfig(
            max_time_diff_s=0.1,
            loop_min_separation_s=15.0,
            loop_sample_interval_s=1.0,
        ),
    )
    assert recovered_report["loop"]["opportunities"] == 1
    assert recovered_report["loop"]["event_count"] == 1
    assert recovered_report["loop"]["event_recall"] == 1.0
    assert recovered_report["loop"]["recall"] == 1.0
    assert missed_report["loop"]["recall"] == 0.0


def test_adjacent_return_samples_are_clustered_into_one_revisit_event():
    reference = [
        MODULE.Pose2(0.0, 0.0, 0.0, 0.0),
        MODULE.Pose2(5.0, 2.0, 0.0, 0.0),
        MODULE.Pose2(10.0, 2.0, 2.0, math.pi),
        MODULE.Pose2(15.0, 0.0, 2.0, math.pi),
        MODULE.Pose2(20.0, 0.1, 0.0, 0.0),
        MODULE.Pose2(21.0, 0.2, 0.0, 0.0),
        MODULE.Pose2(30.0, 2.0, 2.0, math.pi),
        MODULE.Pose2(31.0, 2.1, 2.0, math.pi),
    ]
    report = MODULE.evaluate(
        reference,
        reference,
        MODULE.EvaluationConfig(
            max_time_diff_s=0.1,
            loop_min_separation_s=15.0,
            loop_sample_interval_s=1.0,
            loop_event_gap_s=2.0,
        ),
    )

    assert report["loop"]["opportunities"] == 4
    assert report["loop"]["event_count"] == 2
    assert [item["opportunity_count"] for item in report["loop"]["events"]] == [2, 2]
    assert all(
        item["minimum_reference_distance_m"] <= 0.2
        for item in report["loop"]["events"]
    )
    assert all(
        item["representative_previous_stamp_s"] < item["representative_current_stamp_s"]
        for item in report["loop"]["events"]
    )


def test_association_rejects_interpolation_across_a_large_gap():
    reference = [
        MODULE.Pose2(0.0, 0.0, 0.0, 0.0),
        MODULE.Pose2(1.0, 1.0, 0.0, 0.0),
        MODULE.Pose2(10.0, 2.0, 0.0, 0.0),
    ]
    estimate = [MODULE.Pose2(stamp, stamp, 0.0, 0.0) for stamp in (0.0, 1.0, 5.0, 10.0)]
    matched_ref, _ = MODULE.associate(reference, estimate, max_time_diff_s=0.1)
    assert [pose.stamp for pose in matched_ref] == [0.0, 1.0, 10.0]
