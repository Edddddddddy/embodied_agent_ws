"""Frontier completion policy tests without ROS runtime."""

from dataclasses import replace

import pytest

from embodied_slam_tools.frontier_monitor import (
    FrontierExplorationMonitor,
    FrontierMonitorConfig,
)
from embodied_slam_tools.mapping_evidence import MappingEvidenceTracker
from embodied_slam_tools.mapping_evidence import FrontierTelemetry
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


def test_unknown_world_plateau_with_reachable_frontier_requests_recovery():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_in_progress",
            detected_frontier_count=2,
            available_frontier_count=2,
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(
            timeout_s=30.0,
            stable_map_s=0.0,
            policy_mode="unknown_world",
        ),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor.wait(
        _request(), recovery_attempts_remaining=1
    ) == "recovery_required:reachable_frontiers_stalled"


def test_no_clearance_provider_reason_is_preserved_before_epoch_min_runtime():
    """明确的 provider 阻塞应立即恢复，不能被通用 60s 最短时长吞掉。"""

    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=3,
            accepted_goal_count=2,
            canceled_goal_count=2,
            completion_reason="no_clearance_safe_frontier_approach",
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(
            min_runtime_s=60.0,
            stable_map_s=15.0,
            policy_mode="unknown_world",
        ),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor._unknown_world_reason(
        evidence.snapshot(),
        elapsed_s=1.0,
        time_budget_reached=False,
        recovery_attempts_remaining=1,
    ) == "recovery_required:no_clearance_safe_frontier_approach"


def test_no_clearance_recovery_waits_for_active_goal_terminal():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=3,
            active_goal_count=1,
            accepted_goal_count=2,
            canceled_goal_count=1,
            completion_reason="no_clearance_safe_frontier_approach",
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(min_runtime_s=60.0, policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor._unknown_world_reason(
        evidence.snapshot(),
        elapsed_s=1.0,
        time_budget_reached=False,
        recovery_attempts_remaining=1,
    ) is None


def test_no_clearance_recovery_rejects_undrained_action_ledger():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=3,
            accepted_goal_count=2,
            canceled_goal_count=1,
            completion_reason="no_clearance_safe_frontier_approach",
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(min_runtime_s=60.0, policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    with pytest.raises(RuntimeError, match="Action ledger is not drained"):
        monitor._unknown_world_reason(
            evidence.snapshot(),
            elapsed_s=1.0,
            time_budget_reached=False,
            recovery_attempts_remaining=1,
        )


@pytest.mark.parametrize(
    "telemetry",
    [
        FrontierTelemetry(
            status="exploration_blocked",
            completion_reason="no_clearance_safe_frontier_approach",
        ),
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=3,
            available_frontier_count=1,
            completion_reason="no_clearance_safe_frontier_approach",
        ),
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=3,
            blacklisted_frontier_count=1,
            completion_reason="no_clearance_safe_frontier_approach",
        ),
    ],
)
def test_no_clearance_recovery_rejects_inconsistent_provider_counters(telemetry):
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(telemetry)
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(min_runtime_s=60.0, policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    with pytest.raises(RuntimeError, match="no-clearance telemetry"):
        monitor._unknown_world_reason(
            evidence.snapshot(),
            elapsed_s=1.0,
            time_budget_reached=False,
            recovery_attempts_remaining=1,
        )


def test_unknown_world_blacklist_handoff_waits_for_idle_grace_before_recovery():
    """回放现场：goal terminal 后 28ms 的黑名单快照不能立即结束 epoch。"""

    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=8,
            blacklisted_frontier_count=2,
            completion_reason="all_frontiers_blacklisted",
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(
            timeout_s=30.0,
            frontier_idle_grace_s=20.0,
            policy_mode="unknown_world",
        ),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor._unknown_world_reason(
        evidence.snapshot(),
        elapsed_s=60.0,
        time_budget_reached=False,
        recovery_attempts_remaining=1,
    ) is None

    now[0] += 20.0
    assert monitor._unknown_world_reason(
        evidence.snapshot(),
        elapsed_s=80.0,
        time_budget_reached=False,
        recovery_attempts_remaining=1,
    ) == "recovery_required:blacklisted_frontiers"


def test_unknown_world_completes_only_after_native_no_frontiers_and_quiet_map():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_complete",
            completion_reason="no_frontiers",
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(
            timeout_s=30.0,
            stable_map_s=0.0,
            policy_mode="unknown_world",
        ),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor.wait(_request()) == "no_reachable_frontiers"


def test_unknown_world_attempt_exhaustion_requests_recovery_scan_confirmation():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=11,
            accepted_goal_count=15,
            succeeded_goal_count=4,
            canceled_goal_count=11,
            completion_reason="frontier_attempts_exhausted_recoverable",
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(
            timeout_s=30.0,
            stable_map_s=0.0,
            policy_mode="unknown_world",
        ),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor.wait(_request(), recovery_attempts_remaining=1) == (
        "recovery_required:frontier_attempts_exhausted"
    )


def test_attempt_exhaustion_provider_reason_preempts_epoch_min_runtime():
    """provider 已搜索完本 epoch 时，不应再空等通用最短时长。"""

    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=11,
            accepted_goal_count=15,
            succeeded_goal_count=4,
            canceled_goal_count=11,
            completion_reason="frontier_attempts_exhausted_recoverable",
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(
            min_runtime_s=60.0,
            stable_map_s=0.0,
            policy_mode="unknown_world",
        ),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor._unknown_world_reason(
        evidence.snapshot(),
        elapsed_s=1.0,
        time_budget_reached=False,
        recovery_attempts_remaining=1,
    ) == "recovery_required:frontier_attempts_exhausted"


def test_attempt_exhaustion_rejects_undrained_action_ledger():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=11,
            accepted_goal_count=15,
            succeeded_goal_count=4,
            canceled_goal_count=10,
            completion_reason="frontier_attempts_exhausted_recoverable",
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(stable_map_s=0.0, policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    with pytest.raises(RuntimeError, match="Action ledger is not drained"):
        monitor._unknown_world_reason(
            evidence.snapshot(),
            elapsed_s=1.0,
            time_budget_reached=False,
            recovery_attempts_remaining=1,
        )


def test_attempt_exhaustion_with_epoch_blacklist_still_requests_relocation():
    """现场回放：黑名单是本 epoch 的失败 approach，不是非法终态。"""

    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=8,
            blacklisted_frontier_count=1,
            accepted_goal_count=22,
            succeeded_goal_count=4,
            canceled_goal_count=18,
            completion_reason="frontier_attempts_exhausted_recoverable",
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(stable_map_s=0.0, policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor._unknown_world_reason(
        evidence.snapshot(),
        elapsed_s=1.0,
        time_budget_reached=False,
        recovery_attempts_remaining=2,
    ) == "recovery_required:frontier_attempts_exhausted"


def test_progress_stall_waits_until_active_action_is_terminal():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            completion_reason="frontier_progress_stalled_recoverable",
            active_goal_count=1,
            accepted_goal_count=2,
            canceled_goal_count=1,
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    reason = monitor._unknown_world_reason(
        evidence.snapshot(),
        elapsed_s=1.0,
        time_budget_reached=False,
        recovery_attempts_remaining=1,
    )

    assert reason is None


def test_progress_stall_requests_recovery_after_action_ledger_drains():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            completion_reason="frontier_progress_stalled_recoverable",
            accepted_goal_count=2,
            canceled_goal_count=2,
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor.wait(_request(), recovery_attempts_remaining=1) == (
        "recovery_required:frontier_progress_stalled"
    )


def test_progress_stall_rejects_undrained_terminal_ledger():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            completion_reason="frontier_progress_stalled_recoverable",
            accepted_goal_count=2,
            canceled_goal_count=1,
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    with pytest.raises(RuntimeError, match="Action ledger is not drained"):
        monitor._unknown_world_reason(
            evidence.snapshot(),
            elapsed_s=1.0,
            time_budget_reached=False,
            recovery_attempts_remaining=1,
        )


@pytest.mark.parametrize(
    "telemetry",
    [
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=3,
            completion_reason="frontier_attempts_exhausted_recoverable",
            available_frontier_count=1,
        ),
        FrontierTelemetry(
            status="exploration_blocked",
            completion_reason="frontier_attempts_exhausted_recoverable",
        ),
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=3,
            blacklisted_frontier_count=4,
            completion_reason="frontier_attempts_exhausted_recoverable",
        ),
    ],
)
def test_attempt_exhaustion_rejects_inconsistent_provider_counters(telemetry):
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(telemetry)
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(stable_map_s=0.0, policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    with pytest.raises(RuntimeError, match="attempt-exhaustion telemetry"):
        monitor._unknown_world_reason(
            evidence.snapshot(),
            elapsed_s=1.0,
            time_budget_reached=False,
            recovery_attempts_remaining=1,
        )


def test_attempt_exhaustion_waits_for_active_action_terminal():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=11,
            active_goal_count=1,
            accepted_goal_count=15,
            succeeded_goal_count=4,
            canceled_goal_count=10,
            completion_reason="frontier_attempts_exhausted_recoverable",
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(stable_map_s=0.0, policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor._unknown_world_reason(
        evidence.snapshot(),
        elapsed_s=1.0,
        time_budget_reached=False,
        recovery_attempts_remaining=1,
    ) is None


def test_unknown_world_attempt_exhaustion_waits_for_map_plateau():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_blocked",
            detected_frontier_count=11,
            accepted_goal_count=15,
            succeeded_goal_count=4,
            canceled_goal_count=11,
            completion_reason="frontier_attempts_exhausted_recoverable",
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(stable_map_s=5.0, policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    reason = monitor._unknown_world_reason(
        evidence.snapshot(),
        elapsed_s=1.0,
        time_budget_reached=False,
        recovery_attempts_remaining=1,
    )

    assert reason is None


def test_unknown_world_timeout_requests_bounded_completion_assessment():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_in_progress",
            detected_frontier_count=1,
            available_frontier_count=1,
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(timeout_s=0.0, policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    assert monitor.wait(
        _request(), recovery_attempts_remaining=1
    ) == "assessment_required:time_budget"


def test_unknown_world_absolute_deadline_caps_restarted_monitor_budget():
    now = [10.0]
    evidence = _ready_evidence(now)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_in_progress",
            detected_frontier_count=1,
            available_frontier_count=1,
        )
    )
    monitor = FrontierExplorationMonitor(
        evidence,
        _Explorer(),
        _config(timeout_s=600.0, policy_mode="unknown_world"),
        cancel_motion=lambda: None,
        clock=lambda: now[0],
    )

    # 模拟 recovery action 已把整轮预算耗尽；provider 自己仍配置 600s，
    # 但重启后的 wait 必须立即服从上层绝对 deadline。
    assert monitor.wait(
        _request(),
        recovery_attempts_remaining=1,
        deadline_monotonic=now[0],
    ) == "assessment_required:time_budget"
