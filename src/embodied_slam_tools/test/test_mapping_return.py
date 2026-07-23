"""未知环境建图返航领域契约测试。"""

from __future__ import annotations

from dataclasses import replace
import math
from pathlib import Path
import sys

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from embodied_slam_tools.mapping_return import (  # noqa: E402
    PlanarPose,
    ReturnActionKind,
    ReturnActionStatus,
    ReturnToStartEvidence,
    ReturnToStartSpec,
    build_return_to_start_report,
    evaluate_return_to_start,
    wrapped_yaw_error_rad,
)


def _passing_evidence() -> ReturnToStartEvidence:
    return ReturnToStartEvidence(
        start_pose=PlanarPose(
            x=1.0,
            y=-2.0,
            yaw=math.pi - 0.05,
            frame_id="map",
            observed_at_ns=1_000_000_000,
        ),
        final_pose=PlanarPose(
            x=1.21,
            y=-1.82,
            yaw=-math.pi + 0.04,
            frame_id="map",
            observed_at_ns=5_100_000_000,
        ),
        action_kind=ReturnActionKind.NAVIGATE_TO_POSE,
        action_status=ReturnActionStatus.SUCCEEDED,
        action_command_id="return-home-1",
        action_started_at_ns=2_000_000_000,
        action_finished_at_ns=5_000_000_000,
        map_saved_at_ns=5_500_000_000,
        cmd_vel_linear_x=0.0,
        cmd_vel_angular_z=0.0,
        cmd_vel_observed_at_ns=5_800_000_000,
        evaluated_at_ns=6_000_000_000,
    )


def test_complete_return_transaction_passes_with_wrapped_yaw():
    decision = evaluate_return_to_start(_passing_evidence())

    assert decision.passed is True
    assert decision.failed_checks == ()
    assert decision.xy_error_m == pytest.approx(0.276586, abs=1.0e-6)
    assert decision.yaw_error_rad == pytest.approx(0.09)
    assert wrapped_yaw_error_rad(math.pi - 0.05, -math.pi + 0.04) == pytest.approx(
        0.09
    )


def test_report_keeps_raw_evidence_thresholds_and_independent_checks():
    report = build_return_to_start_report(_passing_evidence())

    assert report["schema_version"] == 1
    assert report["passed"] is True
    assert all(report["checks"].values())
    assert report["thresholds"]["max_xy_error_m"] == 0.35
    assert report["errors"]["xy_error_m"] == pytest.approx(0.276586, abs=1.0e-6)
    assert report["action"] == {
        "kind": "navigate_to_pose",
        "status": "succeeded",
        "command_id": "return-home-1",
        "started_at_ns": 2_000_000_000,
        "finished_at_ns": 5_000_000_000,
    }


def test_xy_boundary_is_inclusive_at_035_meters():
    evidence = _passing_evidence()
    final_pose = replace(evidence.final_pose, x=1.35, y=-2.0)

    decision = evaluate_return_to_start(replace(evidence, final_pose=final_pose))

    assert decision.xy_error_m == pytest.approx(0.35)
    assert decision.check("xy_within_tolerance") is True
    assert decision.passed is True


@pytest.mark.parametrize(
    "status",
    (
        ReturnActionStatus.REJECTED,
        ReturnActionStatus.ABORTED,
        ReturnActionStatus.CANCELED,
        ReturnActionStatus.TIMED_OUT,
        ReturnActionStatus.ACCEPTED,
        ReturnActionStatus.EXECUTING,
    ),
)
def test_non_success_action_states_fail_closed(status):
    decision = evaluate_return_to_start(
        replace(_passing_evidence(), action_status=status)
    )

    assert decision.passed is False
    assert decision.check("typed_action_succeeded") is False


def test_return_must_finish_before_map_is_saved():
    evidence = _passing_evidence()
    decision = evaluate_return_to_start(
        replace(evidence, map_saved_at_ns=evidence.action_finished_at_ns)
    )

    assert decision.passed is False
    assert decision.check("return_before_map_save") is False


def test_missing_or_out_of_tolerance_pose_fails_closed():
    missing = evaluate_return_to_start(replace(_passing_evidence(), final_pose=None))
    far_pose = replace(_passing_evidence().final_pose, x=1.36, y=-2.0)
    far = evaluate_return_to_start(
        replace(_passing_evidence(), final_pose=far_pose)
    )

    assert missing.passed is False
    assert missing.check("same_pose_frame") is False
    assert missing.check("xy_within_tolerance") is False
    assert far.passed is False
    assert far.check("xy_within_tolerance") is False


def test_pose_from_a_different_frame_cannot_be_compared_directly():
    evidence = _passing_evidence()
    final_pose = replace(evidence.final_pose, frame_id="odom")

    decision = evaluate_return_to_start(replace(evidence, final_pose=final_pose))

    assert decision.passed is False
    assert decision.check("same_pose_frame") is False


@pytest.mark.parametrize(
    ("field", "value", "failed_check"),
    (
        ("cmd_vel_linear_x", 0.01, "final_cmd_vel_zero"),
        ("cmd_vel_angular_z", -0.01, "final_cmd_vel_zero"),
        ("cmd_vel_observed_at_ns", 4_900_000_000, "cmd_vel_after_return"),
        ("cmd_vel_observed_at_ns", 6_100_000_000, "cmd_vel_after_return"),
    ),
)
def test_invalid_final_velocity_evidence_fails_closed(field, value, failed_check):
    decision = evaluate_return_to_start(replace(_passing_evidence(), **{field: value}))

    assert decision.passed is False
    assert decision.check(failed_check) is False


def test_stale_zero_velocity_is_not_accepted():
    evidence = replace(
        _passing_evidence(),
        cmd_vel_observed_at_ns=5_800_000_000,
        evaluated_at_ns=7_000_000_001,
    )

    decision = evaluate_return_to_start(evidence)

    assert decision.passed is False
    assert decision.check("final_cmd_vel_zero") is True
    assert decision.check("final_cmd_vel_fresh") is False


def test_yaw_error_beyond_configured_limit_is_rejected():
    evidence = _passing_evidence()
    final_pose = replace(evidence.final_pose, yaw=0.0)

    decision = evaluate_return_to_start(
        replace(evidence, final_pose=final_pose),
        ReturnToStartSpec(max_yaw_error_rad=0.2),
    )

    assert decision.passed is False
    assert decision.check("yaw_within_tolerance") is False


def test_untyped_action_values_are_rejected_at_the_boundary():
    evidence = _passing_evidence()

    with pytest.raises(TypeError, match="action_status"):
        replace(evidence, action_status="succeeded")
    with pytest.raises(TypeError, match="action_kind"):
        replace(evidence, action_kind="navigate_to_pose")
