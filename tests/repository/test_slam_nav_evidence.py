"""SLAM/Nav2 验收证据深模块的纯逻辑契约。"""

from tools.acceptance.slam_nav_evidence import (
    AutomaticMissionObservation,
    AutomaticMissionThresholds,
    DynamicNavigationObservation,
    DynamicNavigationThresholds,
    cumulative_distance,
    evaluate_dynamic_navigation,
    path_clearance,
    path_signature,
    build_automatic_mission_report,
)


STATUS_SUCCEEDED = 4


def _thresholds() -> DynamicNavigationThresholds:
    return DynamicNavigationThresholds(
        minimum_track_confidence=0.7,
        minimum_moving_velocity_mps=0.1,
        minimum_predicted_cost=253,
        minimum_clearance_gain_m=0.5,
        minimum_unique_navigation_plans=2,
        minimum_travel_distance_m=1.0,
    )


def _observation(**overrides) -> DynamicNavigationObservation:
    values = {
        "scenario_id": "crossing-person",
        "goal": {"name": "entrance", "x": 3.0, "y": 0.0},
        "route_attempts": ({"name": "entrance", "status": "accepted"},),
        "motion_anchor": {"x": 1.0, "y": 0.0},
        "gazebo_pose_updates": 5,
        "expected_pose_updates": 5,
        "track_id": "track_1",
        "track_confidence": 0.9,
        "track_velocity_x_mps": 0.0,
        "track_velocity_y_mps": 0.35,
        "predicted_x": 1.0,
        "predicted_y": 0.0,
        "predicted_cost": 254,
        "baseline_path": ((0.0, 0.0), (1.0, 0.0), (3.0, 0.0)),
        "dynamic_path": ((0.0, 0.0), (1.0, 1.0), (3.0, 0.0)),
        "published_plans": (
            ((0.0, 0.0), (1.0, 0.0), (3.0, 0.0)),
            ((0.0, 0.0), (1.0, 1.0), (3.0, 0.0)),
        ),
        "odom_traveled_distance_m": 2.0,
        "navigate_to_pose_status": STATUS_SUCCEEDED,
        "navigation_succeeded": True,
        "final_linear_x": 0.0,
        "final_angular_z": 0.0,
    }
    values.update(overrides)
    return DynamicNavigationObservation(**values)


def test_geometry_helpers_are_ros_independent_and_deterministic():
    points = ((0.0, 0.0), (3.0, 4.0), (6.0, 4.0))

    assert cumulative_distance(points) == 8.0
    assert path_clearance(points, 3.0, 3.0) == 1.0
    assert path_signature(points) == ((0.0, 0.0), (3.0, 4.0), (6.0, 4.0))


def test_dynamic_navigation_report_centralizes_all_pass_conditions():
    report = evaluate_dynamic_navigation(_observation(), _thresholds())

    assert report["passed"] is True
    assert all(report["checks"].values())
    assert report["baseline_clearance_m"] == 0.0
    assert report["dynamic_clearance_m"] == 1.0
    assert report["unique_plan_count"] == 2
    assert report["navigate_to_pose_status"] == STATUS_SUCCEEDED


def test_dynamic_navigation_report_rejects_unchanged_path_and_nonzero_stop():
    baseline = ((0.0, 0.0), (1.0, 0.0), (3.0, 0.0))
    report = evaluate_dynamic_navigation(
        _observation(
            dynamic_path=baseline,
            published_plans=(baseline, baseline),
            final_linear_x=0.1,
        ),
        _thresholds(),
    )

    assert report["passed"] is False
    assert report["checks"]["dynamic_path_increased_clearance"] is False
    assert report["checks"]["nav2_replanned"] is False
    assert report["checks"]["cmd_vel_zero"] is False


def _automatic_observation(**overrides) -> AutomaticMissionObservation:
    values = {
        "session_id": "session-1",
        "session_start_ns": 100,
        "state_sequence": (1, 2, 10, 3, 7, 11, 12),
        "map_saved": True,
        "map_yaml_path": "/tmp/map.yaml",
        "final_phase": 12,
        "evidence_kind": "gazebo_frontier_slam_map_saver_amcl_nav2_dynamic_replan",
        "action_candidates": (
            {"name": "navigate_to", "request_id": "nav-1"},
            {"name": "follow_waypoints", "request_id": "patrol-1"},
        ),
        "action_results": (
            {"command_id": "nav-1", "success": True, "message": "nav2:ok"},
            {
                "command_id": "patrol-1",
                "success": True,
                "message": "nav2:follow_waypoints:missed_waypoints=0",
            },
        ),
        "map_stats": {"known_cells": 8000, "occupied_cells": 300},
        "map_provenance": {"yaml_mtime_ns": 120, "image_mtime_ns": 121},
        "mapping_path_m": 12.0,
        "frontier_goal_count": 2,
        "exploration_completion_reason": "coverage_plateau",
        "localization_tf_count": 2,
        "amcl_pose_count": 3,
        "lifecycle_states": {"bt_navigator": 3, "waypoint_follower": 3},
        "nav2_lifecycle_active": True,
        "dynamic_navigation": {"passed": True},
        "dynamic_navigation_required": True,
        "provenance": {"world_sha256": "world"},
        "final_linear_x": 0.0,
        "final_angular_z": 0.0,
    }
    values.update(overrides)
    return AutomaticMissionObservation(**values)


def test_automatic_mission_report_proves_every_runtime_layer():
    report = build_automatic_mission_report(
        _automatic_observation(),
        AutomaticMissionThresholds(
            minimum_mapping_path_m=10.0,
            minimum_known_map_cells=6000,
            minimum_occupied_map_cells=150,
        ),
    )

    assert report["schema_version"] == 3
    assert set(report) == {
        "schema_version",
        "passed",
        "session_id",
        "session_start_ns",
        "state_sequence",
        "map_saved",
        "map_yaml_path",
        "final_phase",
        "evidence_kind",
        "automatic_mission",
        "action_candidates",
        "action_results",
        "map",
        "map_provenance",
        "mapping_path_m",
        "frontier_goal_count",
        "exploration_completion_reason",
        "localization",
        "dynamic_navigation",
        "provenance",
        "checks",
        "final_cmd_vel",
    }
    assert set(report["checks"]) == {
        "map_saved",
        "fresh_session_map",
        "frontier_goal_observed",
        "mapping_path_threshold",
        "known_cells_threshold",
        "occupied_cells_threshold",
        "auditable_exploration_completion",
        "localization_tf",
        "amcl_pose",
        "nav2_lifecycle_active",
        "semantic_navigation_succeeded",
        "dynamic_navigation_succeeded",
        "final_cmd_vel_zero",
    }
    assert report["passed"] is True
    assert all(report["checks"].values())
    assert report["automatic_mission"] is True
    assert report["mapping_path_m"] == 12.0


def test_automatic_mission_report_rejects_stale_map_and_partial_patrol():
    report = build_automatic_mission_report(
        _automatic_observation(
            map_provenance={"yaml_mtime_ns": 90, "image_mtime_ns": 121},
            action_results=(
                {"command_id": "nav-1", "success": True, "message": "ok"},
                {
                    "command_id": "patrol-1",
                    "success": True,
                    "message": "nav2:follow_waypoints:missed_waypoints=1",
                },
            ),
        ),
        AutomaticMissionThresholds(10.0, 6000, 150),
    )

    assert report["passed"] is False
    assert report["checks"]["fresh_session_map"] is False
    assert report["checks"]["semantic_navigation_succeeded"] is False


def test_automatic_mission_dry_run_does_not_claim_runtime_evidence():
    report = build_automatic_mission_report(
        _automatic_observation(
            evidence_kind="dry_run_process_adapter",
            map_provenance=None,
            map_stats=None,
            frontier_goal_count=0,
            localization_tf_count=0,
            amcl_pose_count=0,
            lifecycle_states={},
            nav2_lifecycle_active=False,
            dynamic_navigation=None,
            dynamic_navigation_required=False,
        ),
        AutomaticMissionThresholds(0.0, 0, 0),
    )

    assert report["passed"] is True
    assert report["checks"]["fresh_session_map"] is True
    assert report["checks"]["semantic_navigation_succeeded"] is True
