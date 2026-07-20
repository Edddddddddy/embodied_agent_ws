"""Unknown-world frontier exploration completion policy contract."""

from dataclasses import replace
import math

import pytest

from embodied_slam_tools.frontier_monitor import (
    ExplorationDecision,
    ExplorationObservation,
    decide_exploration,
)


def _observation(**overrides) -> ExplorationObservation:
    observation = ExplorationObservation(
        native_completion_reported=False,
        reachable_frontier_count=0,
        active_goal_count=0,
        blacklisted_frontier_count=0,
        plateau_detected=False,
        time_budget_reached=False,
        recovery_attempts_remaining=1,
        map_quiet_s=0.0,
        required_quiet_s=10.0,
        frontier_idle_s=0.0,
        required_frontier_idle_s=20.0,
    )
    return replace(observation, **overrides)


def test_plateau_with_reachable_frontier_is_not_complete():
    # 地图暂时不增长可能只是规划器卡住；仍有可达 frontier 时不能把平台期当成建图完成。
    decision = decide_exploration(
        _observation(
            plateau_detected=True,
            reachable_frontier_count=2,
            map_quiet_s=30.0,
        )
    )

    assert decision is not ExplorationDecision.COMPLETE


def test_reachable_frontier_handoff_grace_keeps_monitor_waiting():
    decision = decide_exploration(
        _observation(
            plateau_detected=True,
            reachable_frontier_count=2,
            map_quiet_s=30.0,
            frontier_idle_s=19.9,
        )
    )

    # 成功目标的 cooldown/下一 Action 交接期间 active=0 是正常瞬态，
    # 不能因为地图恰好处于平台期就重启 explorer。
    assert decision is ExplorationDecision.CONTINUE


def test_reachable_frontier_stall_recovers_after_handoff_grace():
    decision = decide_exploration(
        _observation(
            plateau_detected=True,
            reachable_frontier_count=2,
            map_quiet_s=30.0,
            frontier_idle_s=20.0,
            recovery_attempts_remaining=1,
        )
    )

    assert decision is ExplorationDecision.RECOVER


def test_reachable_frontier_stall_fails_after_grace_and_recovery_exhaustion():
    decision = decide_exploration(
        _observation(
            plateau_detected=True,
            reachable_frontier_count=2,
            map_quiet_s=30.0,
            frontier_idle_s=20.0,
            recovery_attempts_remaining=0,
        )
    )

    assert decision is ExplorationDecision.FAIL


def test_active_goal_prevents_completion_even_after_native_complete():
    decision = decide_exploration(
        _observation(
            native_completion_reported=True,
            active_goal_count=1,
            map_quiet_s=30.0,
        )
    )

    assert decision is ExplorationDecision.CONTINUE


def test_time_budget_with_reachable_frontier_fails_instead_of_passing():
    decision = decide_exploration(
        _observation(
            reachable_frontier_count=1,
            time_budget_reached=True,
        )
    )

    assert decision is ExplorationDecision.FAIL


def test_blacklisted_frontier_recovers_while_budget_remains():
    decision = decide_exploration(
        _observation(
            blacklisted_frontier_count=1,
            recovery_attempts_remaining=1,
        )
    )

    assert decision is ExplorationDecision.RECOVER


def test_blacklisted_frontier_fails_after_recovery_budget_is_exhausted():
    decision = decide_exploration(
        _observation(
            blacklisted_frontier_count=1,
            recovery_attempts_remaining=0,
        )
    )

    assert decision is ExplorationDecision.FAIL


def test_native_completion_before_quiet_window_keeps_waiting():
    decision = decide_exploration(
        _observation(
            native_completion_reported=True,
            map_quiet_s=9.9,
        )
    )

    assert decision is ExplorationDecision.CONTINUE


def test_native_completion_needs_no_active_or_reachable_frontier_and_quiet_window():
    decision = decide_exploration(
        _observation(
            native_completion_reported=True,
            reachable_frontier_count=0,
            active_goal_count=0,
            map_quiet_s=10.0,
        )
    )

    assert decision is ExplorationDecision.COMPLETE


@pytest.mark.parametrize("invalid", [-0.1, math.nan, math.inf, -math.inf])
def test_rejects_invalid_frontier_idle_windows(invalid):
    with pytest.raises(ValueError, match="quiet windows"):
        decide_exploration(_observation(frontier_idle_s=invalid))
