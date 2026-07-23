"""Mapping evidence is deterministic and independent of rclpy."""

from dataclasses import replace

import pytest

from embodied_slam_tools.mapping_evidence import (
    FrontierTelemetry,
    MappingEvidenceTracker,
    NavigationGoalLedger,
    NavigationGoalStatus,
)


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


def test_latest_odom_remains_available_while_mapping_path_is_paused():
    tracker = MappingEvidenceTracker(1)

    tracker.record_odom(1.0, 2.0)
    first = tracker.snapshot()
    tracker.record_odom(0.75, 2.0)
    second = tracker.snapshot()

    assert first.latest_odom_xy == pytest.approx((1.0, 2.0))
    assert second.latest_odom_xy == pytest.approx((0.75, 2.0))
    assert second.odom_generation == first.odom_generation + 1
    # 恢复动作位移可审计，但不伪装成自主 frontier 探索里程。
    assert tracker.mapping_path_m == 0.0


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


def test_frontier_telemetry_is_recorded_atomically_and_reset_per_attempt():
    tracker = MappingEvidenceTracker(1)
    telemetry = FrontierTelemetry(
        status="exploration_blocked",
        detected_frontier_count=4,
        available_frontier_count=0,
        blacklisted_frontier_count=4,
        active_goal_count=0,
        active_goal_id="",
        accepted_goal_count=3,
        succeeded_goal_count=1,
        aborted_goal_count=1,
        canceled_goal_count=1,
        rejected_goal_count=0,
        last_goal_terminal="canceled",
        completion_reason="all_frontiers_blacklisted",
    )

    tracker.record_frontier_telemetry(telemetry)

    assert tracker.snapshot().frontier_telemetry == telemetry
    assert tracker.explore_status == "exploration_blocked"

    tracker.reset_exploration()

    assert tracker.frontier_telemetry == FrontierTelemetry()


def test_frontier_activity_clock_ignores_heartbeat_but_tracks_real_changes():
    now = [10.0]
    tracker = MappingEvidenceTracker(1, clock=lambda: now[0])
    telemetry = FrontierTelemetry(
        status="exploration_in_progress",
        detected_frontier_count=2,
        available_frontier_count=1,
        active_goal_count=1,
        active_goal_id="goal-a",
        accepted_goal_count=1,
    )

    now[0] = 12.0
    tracker.record_frontier_telemetry(telemetry)
    assert tracker.snapshot().last_frontier_activity_at == 12.0

    now[0] = 40.0
    tracker.record_frontier_telemetry(telemetry)
    # Explore Lite 的周期心跳内容完全相同，不应让停滞计时永远归零。
    assert tracker.snapshot().last_frontier_activity_at == 12.0

    now[0] = 41.0
    tracker.record_frontier_telemetry(
        replace(telemetry, active_goal_id="goal-b")
    )
    # 即使计数相同，goal id 改变也代表真实 Action 交接。
    assert tracker.snapshot().last_frontier_activity_at == 41.0


def test_frontier_recovery_preserves_action_terminal_totals_only():
    tracker = MappingEvidenceTracker(1)
    tracker.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            available_frontier_count=2,
            blacklisted_frontier_count=1,
            accepted_goal_count=2,
            succeeded_goal_count=1,
            aborted_goal_count=1,
        )
    )

    tracker.reset_exploration(preserve_action_totals=True)
    tracker.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_complete",
            available_frontier_count=0,
            accepted_goal_count=1,
            succeeded_goal_count=1,
            completion_reason="no_frontiers",
        )
    )

    telemetry = tracker.frontier_telemetry
    assert telemetry.available_frontier_count == 0
    assert telemetry.blacklisted_frontier_count == 0
    assert telemetry.accepted_goal_count == 3
    assert telemetry.succeeded_goal_count == 2
    assert telemetry.aborted_goal_count == 1


def test_navigation_goal_ledger_preserves_typed_lifecycle_and_plan_evidence():
    ledger = NavigationGoalLedger()
    ledger.reset(7)
    ledger.plan(((1.0, 2.0), (3.0, 4.0)))

    ledger.transition(1, NavigationGoalStatus.REQUESTED, timestamp_ns=100)
    assert ledger.active() == ledger.get(1)
    assert ledger.active().goal_xy == (1.0, 2.0)
    ledger.record_plan(
        1,
        unknown_cell_count=0,
        occupied_cell_count=0,
        outside_map_cell_count=0,
    )
    ledger.transition(1, NavigationGoalStatus.ACCEPTED, timestamp_ns=110)
    ledger.transition(
        1,
        NavigationGoalStatus.SUCCEEDED,
        timestamp_ns=200,
        nav2_status=4,
    )

    first, second = ledger.snapshot()
    assert ledger.mission_sequence == 7
    assert first.status is NavigationGoalStatus.SUCCEEDED
    assert first.started_at_ns == 100
    assert first.finished_at_ns == 200
    assert first.all_plans_known_free is True
    assert ledger.is_terminal(1) is True
    assert ledger.is_terminal(2) is False
    assert second.status is NavigationGoalStatus.PLANNED
    assert ledger.active_sequence() is None
    assert ledger.active() is None


def test_navigation_goal_ledger_retains_worst_plan_counts():
    ledger = NavigationGoalLedger()
    ledger.reset(1)
    ledger.plan(((1.0, 1.0),))
    ledger.transition(1, NavigationGoalStatus.REQUESTED, timestamp_ns=1)
    ledger.record_plan(
        1,
        unknown_cell_count=0,
        occupied_cell_count=0,
        outside_map_cell_count=0,
    )
    ledger.record_plan(
        1,
        unknown_cell_count=2,
        occupied_cell_count=1,
        outside_map_cell_count=3,
    )

    record = ledger.snapshot()[0]
    assert record.plan_count == 2
    assert record.max_unknown_cell_count == 2
    assert record.max_occupied_cell_count == 1
    assert record.max_outside_map_cell_count == 3
    assert record.all_plans_known_free is False
