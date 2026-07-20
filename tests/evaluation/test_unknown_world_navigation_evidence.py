"""动态采样导航与 frontier 完成证据契约。"""

from __future__ import annotations

import sys
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from tools.acceptance.unknown_world_evidence import (  # noqa: E402
    LocalizationSample,
    MapQualityThresholds,
    MapRegion,
    NavigationGoalObservation,
    UnknownWorldObservation,
    UnknownWorldThresholds,
    build_unknown_world_report,
    evaluate_frontier_completion,
    evaluate_sampled_navigation,
    load_scene_evaluation_context,
)


def _write_map(directory: Path, rows: list[list[int]]) -> Path:
    pgm = directory / "map.pgm"
    body = "\n".join(" ".join(str(value) for value in row) for row in rows)
    pgm.write_text(
        f"P2\n{len(rows[0])} {len(rows)}\n255\n{body}\n",
        encoding="ascii",
    )
    yaml_path = directory / "map.yaml"
    yaml_path.write_text(
        "\n".join(
            (
                "image: map.pgm",
                "mode: trinary",
                "resolution: 1.0",
                "origin: [0.0, 0.0, 0.0]",
                "negate: 0",
                "occupied_thresh: 0.65",
                "free_thresh: 0.196",
                "",
            )
        ),
        encoding="utf-8",
    )
    return yaml_path


def _producer(
    sequence: int,
    x_m: float,
    y_m: float,
    *,
    plan_count: int = 1,
    known_free: bool = True,
) -> dict[str, object]:
    return {
        "sequence": sequence,
        "x": x_m,
        "y": y_m,
        "frame_id": "map",
        "status": 5,
        "nav2_status": 4,
        "nav2_error_code": 0,
        "started_at": float(sequence),
        "finished_at": float(sequence) + 0.5,
        "plan_count": plan_count,
        "max_unknown_cell_count": 0,
        "max_occupied_cell_count": 0,
        "max_outside_map_cell_count": 0,
        "all_plans_known_free": known_free,
    }


def _dynamic_report() -> dict[str, object]:
    check_names = (
        "gazebo_entity_moved",
        "tracker_confident",
        "tracker_estimated_motion",
        "future_cell_marked_lethal",
        "dynamic_path_increased_clearance",
        "nav2_replanned",
        "navigate_to_pose_succeeded",
        "robot_moved",
        "cmd_vel_zero",
    )
    return {
        "passed": True,
        "checks": {name: True for name in check_names},
        "scenario_id": "unit-dynamic-obstacle",
        "published_plan_count": 3,
        "unique_plan_count": 2,
        "navigate_to_pose_status": 4,
    }


def test_three_successful_goals_with_known_free_paths_pass(tmp_path: Path):
    map_yaml = _write_map(tmp_path, [[254] * 8 for _ in range(8)])
    goals = tuple(
        NavigationGoalObservation(
            x_m=float(index + 1),
            y_m=float(index + 1),
            succeeded=True,
            planned_paths=(((0.5, 0.5), (float(index + 1), float(index + 1))),),
            producer_evidence=_producer(
                index + 1, float(index + 1), float(index + 1)
            ),
        )
        for index in range(3)
    )

    report = evaluate_sampled_navigation(
        built_map_yaml=map_yaml,
        goals=goals,
        minimum_goal_count=3,
        minimum_goal_separation_m=1.0,
    )

    assert report["passed"] is True
    assert report["metrics"]["succeeded_goal_count"] == 3


def test_path_crossing_unknown_fails_even_when_endpoints_are_free(tmp_path: Path):
    rows = [[254] * 7 for _ in range(7)]
    # PGM 自上向下；整列 unknown 横切对角线，端点仍是 free。
    for row in rows:
        row[3] = 205
    map_yaml = _write_map(tmp_path, rows)
    observation = NavigationGoalObservation(
        x_m=6.5,
        y_m=6.5,
        succeeded=True,
        planned_paths=(((0.5, 0.5), (6.5, 6.5)),),
        producer_evidence=_producer(1, 6.5, 6.5),
    )

    report = evaluate_sampled_navigation(
        built_map_yaml=map_yaml,
        goals=(observation,),
        minimum_goal_count=1,
    )

    assert report["passed"] is False
    plan = report["goals"][0]["plans"][0]
    assert plan["unknown_sample_count"] > 0


def test_goal_without_observed_plan_or_success_fails(tmp_path: Path):
    map_yaml = _write_map(tmp_path, [[254] * 4 for _ in range(4)])

    report = evaluate_sampled_navigation(
        built_map_yaml=map_yaml,
        goals=(
            NavigationGoalObservation(
                x_m=1.5,
                y_m=1.5,
                succeeded=False,
                planned_paths=(),
                producer_evidence=_producer(
                    1, 1.5, 1.5, plan_count=0, known_free=False
                ),
            ),
        ),
        minimum_goal_count=1,
    )

    assert report["passed"] is False
    assert report["goals"][0]["checks"]["plan_observed"] is False
    assert report["goals"][0]["checks"]["action_succeeded"] is False


def test_typed_success_cannot_hide_nav2_failure(tmp_path: Path):
    map_yaml = _write_map(tmp_path, [[254] * 4 for _ in range(4)])
    producer = _producer(1, 1.5, 1.5)
    producer["nav2_status"] = 6
    producer["nav2_error_code"] = 208

    report = evaluate_sampled_navigation(
        built_map_yaml=map_yaml,
        goals=(
            NavigationGoalObservation(
                x_m=1.5,
                y_m=1.5,
                succeeded=True,
                planned_paths=(((0.5, 0.5), (1.5, 1.5)),),
                producer_evidence=producer,
            ),
        ),
        minimum_goal_count=1,
    )

    assert report["passed"] is False
    assert report["goals"][0]["checks"]["producer_lifecycle_valid"] is False


def test_repeated_goal_or_sequence_cannot_fake_three_navigation_goals(
    tmp_path: Path,
):
    map_yaml = _write_map(tmp_path, [[254] * 8 for _ in range(8)])
    repeated = NavigationGoalObservation(
        x_m=2.5,
        y_m=2.5,
        succeeded=True,
        planned_paths=(((0.5, 0.5), (2.5, 2.5)),),
        producer_evidence=_producer(1, 2.5, 2.5),
    )

    report = evaluate_sampled_navigation(
        built_map_yaml=map_yaml,
        goals=(repeated, repeated, repeated),
        minimum_goal_count=3,
        minimum_goal_separation_m=1.5,
    )

    assert report["passed"] is False
    assert report["checks"]["unique_positive_goal_sequences"] is False
    assert report["checks"]["minimum_goal_separation"] is False


def test_frontier_completion_requires_no_reachable_active_or_blacklisted_goal():
    complete = {
        "valid": True,
        "completion_reason": "no_frontiers",
        "mission_completion_reason": "no_reachable_frontiers",
        "available_frontier_count": 0,
        "active_goal_count": 0,
        "blacklisted_frontier_count": 0,
        "accepted_goal_count": 3,
        "succeeded_goal_count": 2,
        "aborted_goal_count": 1,
        "canceled_goal_count": 0,
    }

    assert evaluate_frontier_completion(complete)["passed"] is True

    for field, value in (
        ("available_frontier_count", 1),
        ("active_goal_count", 1),
        ("blacklisted_frontier_count", 1),
    ):
        invalid = {**complete, field: value}
        assert evaluate_frontier_completion(invalid)["passed"] is False


def test_frontier_completion_accepts_exhaustion_only_with_no_map_gain_pair():
    report = evaluate_frontier_completion(
        {
            "valid": True,
            "provider_completion_reason": (
                "frontier_attempts_exhausted_recoverable"
            ),
            "mission_completion_reason": (
                "frontier_attempts_exhausted_no_map_gain"
            ),
            "available_frontier_count": 0,
            "active_goal_count": 0,
            "blacklisted_frontier_count": 0,
            "accepted_goal_count": 2,
            "succeeded_goal_count": 1,
            "aborted_goal_count": 1,
        }
    )

    assert report["passed"] is True


@pytest.mark.parametrize(
    ("provider_reason", "mission_reason"),
    [
        ("no_frontiers", "frontier_attempts_exhausted_no_map_gain"),
        (
            "frontier_attempts_exhausted_recoverable",
            "no_reachable_frontiers",
        ),
        ("coverage_plateau", "no_reachable_frontiers"),
    ],
)
def test_frontier_completion_rejects_crossed_or_legacy_reason_pairs(
    provider_reason,
    mission_reason,
):
    report = evaluate_frontier_completion(
        {
            "valid": True,
            "provider_completion_reason": provider_reason,
            "mission_completion_reason": mission_reason,
        }
    )

    assert report["passed"] is False
    assert report["checks"]["completion_reason_pair"] is False


def test_frontier_completion_rejects_non_terminal_accepted_goal():
    report = evaluate_frontier_completion(
        {
            "valid": True,
            "completion_reason": "no_frontiers",
            "mission_completion_reason": "no_reachable_frontiers",
            "accepted_goal_count": 2,
            "succeeded_goal_count": 1,
        }
    )

    assert report["passed"] is False
    assert report["checks"]["all_accepted_goals_terminal"] is False


def test_scene_evaluator_derives_regions_relative_to_spawn():
    context = load_scene_evaluation_context(
        ROOT / "src/embodied_simulation/config/showcase_apartment.yaml"
    )

    assert context.world_name == "voice_slam_nav_showcase"
    assert {region.name for region in context.regions} == {
        "living",
        "kitchen",
        "meeting",
        "office",
    }
    living = next(region for region in context.regions if region.name == "living")
    assert living.min_x_m == pytest.approx(-0.79)
    assert living.max_y_m == pytest.approx(4.09)


def test_schema_v4_requires_every_unknown_world_evidence_layer(tmp_path: Path):
    map_yaml = _write_map(tmp_path, [[254] * 8 for _ in range(8)])
    goal_coordinates = ((1.5, 1.5), (3.5, 3.5), (5.5, 5.5))
    goals = tuple(
        NavigationGoalObservation(
            x_m=x_m,
            y_m=y_m,
            succeeded=True,
            planned_paths=(((0.5, 0.5), (x_m, y_m)),),
            producer_evidence=_producer(index, x_m, y_m),
        )
        for index, (x_m, y_m) in enumerate(goal_coordinates, start=1)
    )
    truth = tuple(
        LocalizationSample(float(index), float(index), 0.0)
        for index in range(3)
    )
    observation = UnknownWorldObservation(
        session_id="unit",
        session_start_ns=100,
        mission_profile_unknown=True,
        mission_sequence=1,
        mission_completed=True,
        mission_outcome=2,
        mission_message="mission succeeded",
        map_saved=True,
        built_map_yaml=map_yaml,
        truth_map_yaml=map_yaml,
        robot_start_xy=(0.5, 0.5),
        regions=(MapRegion("all", 0.0, 0.0, 8.0, 8.0),),
        map_provenance={"yaml_mtime_ns": 101, "image_mtime_ns": 101},
        frontier_telemetry={
            "valid": True,
            "completion_reason": "no_frontiers",
            "mission_completion_reason": "no_reachable_frontiers",
            "accepted_goal_count": 1,
            "succeeded_goal_count": 1,
        },
        navigation_goals=goals,
        amcl_samples=truth,
        gazebo_samples=truth,
        gazebo_truth_error="",
        nav2_lifecycle_active=True,
        dynamic_navigation=_dynamic_report(),
        final_linear_x=0.0,
        final_angular_z=0.0,
        cmd_vel_sample_count=10,
        nonzero_cmd_vel_sample_count=5,
        last_cmd_vel_received_at_s=11.0,
        final_stop_boundary_at_s=10.0,
        cmd_vel_samples_after_boundary=4,
        nonzero_cmd_vel_samples_after_boundary=3,
    )
    thresholds = UnknownWorldThresholds(
        map_quality=MapQualityThresholds(0.90, 0.85, 0.10),
        minimum_aligned_samples=3,
    )

    report = build_unknown_world_report(observation, thresholds)

    assert report["schema_version"] == 4
    assert report["passed"] is True
    assert all(report["checks"].values())


def test_passed_bit_without_dynamic_evidence_is_rejected(tmp_path: Path):
    map_yaml = _write_map(tmp_path, [[254] * 8 for _ in range(8)])
    goal_coordinates = ((1.5, 1.5), (3.5, 3.5), (5.5, 5.5))
    truth = tuple(
        LocalizationSample(float(index), float(index), 0.0)
        for index in range(3)
    )
    observation = UnknownWorldObservation(
        session_id="unit",
        session_start_ns=100,
        mission_profile_unknown=True,
        mission_sequence=1,
        mission_completed=True,
        mission_outcome=2,
        mission_message="mission succeeded",
        map_saved=True,
        built_map_yaml=map_yaml,
        truth_map_yaml=map_yaml,
        robot_start_xy=(0.5, 0.5),
        regions=(MapRegion("all", 0.0, 0.0, 8.0, 8.0),),
        map_provenance={"yaml_mtime_ns": 101, "image_mtime_ns": 101},
        frontier_telemetry={
            "valid": True,
            "completion_reason": "no_frontiers",
            "mission_completion_reason": "no_reachable_frontiers",
        },
        navigation_goals=tuple(
            NavigationGoalObservation(
                x_m=x_m,
                y_m=y_m,
                succeeded=True,
                planned_paths=(((0.5, 0.5), (x_m, y_m)),),
                producer_evidence=_producer(index, x_m, y_m),
            )
            for index, (x_m, y_m) in enumerate(goal_coordinates, start=1)
        ),
        amcl_samples=truth,
        gazebo_samples=truth,
        gazebo_truth_error="",
        nav2_lifecycle_active=True,
        dynamic_navigation={"passed": True},
        final_linear_x=0.0,
        final_angular_z=0.0,
        cmd_vel_sample_count=2,
        nonzero_cmd_vel_sample_count=1,
        last_cmd_vel_received_at_s=2.0,
        final_stop_boundary_at_s=1.0,
        cmd_vel_samples_after_boundary=2,
        nonzero_cmd_vel_samples_after_boundary=1,
    )
    thresholds = UnknownWorldThresholds(
        map_quality=MapQualityThresholds(0.90, 0.85, 0.10),
        minimum_aligned_samples=3,
    )

    report = build_unknown_world_report(observation, thresholds)

    assert report["checks"]["dynamic_navigation"] is False
    assert report["passed"] is False


def test_initialized_zero_velocity_is_not_terminal_stop_evidence(tmp_path: Path):
    map_yaml = _write_map(tmp_path, [[254] * 8 for _ in range(8)])
    truth = tuple(
        LocalizationSample(float(index), float(index), 0.0)
        for index in range(3)
    )
    observation = UnknownWorldObservation(
        session_id="unit",
        session_start_ns=100,
        mission_profile_unknown=True,
        mission_sequence=1,
        mission_completed=True,
        mission_outcome=2,
        mission_message="mission succeeded",
        map_saved=True,
        built_map_yaml=map_yaml,
        truth_map_yaml=map_yaml,
        robot_start_xy=(0.5, 0.5),
        regions=(MapRegion("all", 0.0, 0.0, 8.0, 8.0),),
        map_provenance={"yaml_mtime_ns": 101, "image_mtime_ns": 101},
        frontier_telemetry={
            "valid": True,
            "completion_reason": "no_frontiers",
            "mission_completion_reason": "no_reachable_frontiers",
        },
        navigation_goals=(),
        amcl_samples=truth,
        gazebo_samples=truth,
        gazebo_truth_error="",
        nav2_lifecycle_active=True,
        dynamic_navigation=_dynamic_report(),
        final_linear_x=0.0,
        final_angular_z=0.0,
    )
    thresholds = UnknownWorldThresholds(
        map_quality=MapQualityThresholds(0.90, 0.85, 0.10),
        minimum_aligned_samples=3,
    )

    report = build_unknown_world_report(observation, thresholds)

    assert report["checks"]["cmd_vel_observed"] is False
    assert report["checks"]["robot_motion_observed"] is False
    assert report["checks"]["final_cmd_vel_fresh"] is False
