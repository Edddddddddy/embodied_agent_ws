"""Mapping evidence is deterministic and independent of rclpy."""

import pytest

from embodied_slam_tools.mapping_evidence import MappingEvidenceTracker


def test_map_growth_timestamp_changes_only_after_configured_growth():
    now = [10.0]
    tracker = MappingEvidenceTracker(3, clock=lambda: now[0])

    tracker.record_map([-1, 0, 100])
    first = tracker.snapshot()
    assert first.map_stats == {"known_cells": 2, "occupied_cells": 1}
    assert first.best_known_map_cells == 0
    assert first.last_map_growth_at == 10.0

    now[0] = 12.0
    tracker.record_map([0, 0, 0, 100])
    second = tracker.snapshot()
    assert second.best_known_map_cells == 4
    assert second.last_map_growth_at == 12.0


def test_mapping_path_rejects_pose_jump_and_stops_after_finish():
    tracker = MappingEvidenceTracker(1)
    tracker.begin_mapping_path()

    tracker.record_odom(0.0, 0.0)
    tracker.record_odom(0.3, 0.0)
    tracker.record_odom(2.0, 0.0)  # Gazebo reset / localization jump
    tracker.record_odom(2.2, 0.0)
    tracker.finish_mapping_path()
    tracker.record_odom(2.4, 0.0)

    assert tracker.mapping_path_m == pytest.approx(0.5)


def test_exploration_state_and_scan_readiness_have_explicit_lifecycle():
    tracker = MappingEvidenceTracker(1)
    tracker.record_explore_status("exploration_complete")
    tracker.completion_reason = "coverage_plateau"
    tracker.mark_scan_ready()

    assert tracker.scan_ready.is_set()
    assert tracker.explore_status == "exploration_complete"
    assert tracker.completion_reason == "coverage_plateau"

    tracker.reset_exploration()

    assert tracker.explore_status == ""
    assert tracker.completion_reason == ""
