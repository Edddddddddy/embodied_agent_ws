"""Frontier completion policy tests without ROS runtime."""

from dataclasses import replace

import pytest

from embodied_slam_tools.frontier_monitor import (
    FrontierExplorationMonitor,
    FrontierMonitorConfig,
)
from embodied_slam_tools.mapping_evidence import MappingEvidenceTracker
from embodied_slam_tools.mission_executor import (
    AutomaticMissionCancelled,
    CommandRequest,
)
from embodied_slam_tools.showcase_session import SessionCommand


class _Explorer:
    def __init__(self):
        self.exit = (False, None)

    def explorer_exited_unexpectedly(self):
        return self.exit


def _request():
    return CommandRequest(
        command=SessionCommand.RUN_AUTOMATIC_MISSION,
        source="test",
    )


def _config(**overrides):
    config = FrontierMonitorConfig(
        timeout_s=0.0,
        min_runtime_s=0.0,
        stable_map_s=5.0,
        completion_status="exploration_complete",
        min_known_cells=4,
        min_occupied_cells=1,
        min_mapping_path_m=0.3,
    )
    return replace(config, **overrides)


def _ready_evidence(now):
    evidence = MappingEvidenceTracker(1, clock=lambda: now[0])
    evidence.begin_mapping_path()
    evidence.record_map([0, 0, 0, 0, 100])
    evidence.record_odom(0.0, 0.0)
    evidence.record_odom(0.4, 0.0)
    return evidence


def test_time_budget_accepts_only_ready_coverage():
    now = [10.0]
    evidence = _ready_evidence(now)
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor.wait(_request()) == "time_budget_coverage"
    assert evidence.completion_reason == "time_budget_coverage"


def test_stable_map_coverage_finishes_without_waiting_for_timeout():
    now = [10.0]
    evidence = _ready_evidence(now)
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(timeout_s=30.0, stable_map_s=0.0),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor.wait(_request()) == "coverage_plateau"


def test_explorer_completion_requires_acceptance_evidence():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_explore_status("exploration_complete")
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(timeout_s=30.0, stable_map_s=60.0),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor.wait(_request()) == "no_frontiers"


def test_explorer_completion_below_threshold_is_explicit_failure():
    now = [10.0]
    evidence = MappingEvidenceTracker(1, clock=lambda: now[0])
    evidence.record_map([0, 100])
    evidence.record_explore_status("exploration_complete")
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(timeout_s=1.0),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    with pytest.raises(RuntimeError, match="known-cell threshold"):
        monitor.wait(_request())


def test_unexpected_explorer_exit_propagates_process_code():
    now = [10.0]
    evidence = MappingEvidenceTracker(1, clock=lambda: now[0])
    explorer = _Explorer()
    explorer.exit = (True, 17)
    monitor = FrontierExplorationMonitor(
        evidence,
        explorer,
        _config(timeout_s=1.0),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    with pytest.raises(RuntimeError, match="code=17"):
        monitor.wait(_request())


def test_cancel_publishes_stop_before_propagating():
    now = [10.0]
    evidence = MappingEvidenceTracker(1, clock=lambda: now[0])
    canceled = []
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(timeout_s=1.0),
        cancel_motion=lambda: canceled.append(True),
        clock=lambda: now[0],
    )
    request = _request()
    request.canceled = True

    with pytest.raises(AutomaticMissionCancelled):
        monitor.wait(request)

    assert canceled == [True]


def test_dry_run_returns_without_requiring_explore_messages():
    evidence = MappingEvidenceTracker(1)
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(
            status_available=False,
            dry_run=True,
            dry_run_delay_s=0.0,
        ),
        cancel_motion=lambda: None,
    )

    assert monitor.wait(_request()) == "dry_run"


def test_missing_explore_message_runtime_has_setup_guidance():
    evidence = MappingEvidenceTracker(1)
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(status_available=False),
        cancel_motion=lambda: None,
    )

    with pytest.raises(RuntimeError, match="setup_frontier_exploration"):
        monitor.wait(_request())
