"""有界 frontier 饱和判定的领域测试。"""

from dataclasses import replace

import pytest

from embodied_slam_tools.exploration_saturation import (
    ExplorationCounters,
    ExplorationEpochEvidence,
    FinalProbeEvidence,
    SaturationDecision,
    SaturationEvidenceTracker,
    SaturationHistory,
    SaturationRuntimeEvidence,
    SaturationTrigger,
    assess_bounded_frontier_saturation,
)


def _passing_history() -> SaturationHistory:
    tracker = SaturationEvidenceTracker()
    known = 25_000
    terminals = 0
    path_m = 0.0
    tracker.observe(
        current_known_cells=known,
        terminal_goal_count=terminals,
        mapping_path_m=path_m,
    )
    for epoch_id in (1, 2):
        tracker.begin_epoch(epoch_id)
        known += 20
        terminals += 3
        path_m += 10.0
        tracker.observe(
            current_known_cells=known,
            terminal_goal_count=terminals,
            mapping_path_m=path_m,
        )
        tracker.finish_epoch()

    tracker.begin_final_probe(trigger=SaturationTrigger.TIME_BUDGET)
    tracker.observe(
        current_known_cells=known + 10,
        terminal_goal_count=terminals,
        mapping_path_m=path_m,
    )
    tracker.finish_final_probe()
    return tracker.snapshot()


def _passing_evidence() -> SaturationRuntimeEvidence:
    return SaturationRuntimeEvidence(
        history=_passing_history(),
        trigger=SaturationTrigger.TIME_BUDGET,
        recovery_attempts_remaining=0,
        residual_available_frontiers=4,
        active_goal_count=0,
        pending_goal_count=0,
        map_quiet_s=10.0,
        typed_stop_confirmed=True,
        typed_stop_after_final_probe=True,
        final_stop_age_s=1.0,
    )


def _repeated_stall_evidence() -> SaturationRuntimeEvidence:
    """复用相同收益证据，但把最终探测绑定到重复停滞触发源。"""

    evidence = _passing_evidence()
    probe = evidence.history.final_probe
    assert probe is not None
    return replace(
        evidence,
        trigger=SaturationTrigger.REPEATED_REACHABLE_STALL,
        recovery_attempts_remaining=0,
        residual_available_frontiers=0,
        history=replace(
            evidence.history,
            final_probe=replace(
                probe,
                trigger=SaturationTrigger.REPEATED_REACHABLE_STALL,
            ),
        ),
    )


def test_complete_requires_all_bounded_saturation_evidence():
    result = assess_bounded_frontier_saturation(_passing_evidence())

    assert result.decision is SaturationDecision.COMPLETE
    assert result.unmet_requirements == ()


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        (
            {"residual_available_frontiers": 5},
            "too_many_residual_frontiers",
        ),
        ({"active_goal_count": 1}, "action_ledger_not_drained"),
        ({"pending_goal_count": 1}, "action_ledger_not_drained"),
        ({"map_quiet_s": 9.99}, "map_not_quiet"),
        ({"typed_stop_confirmed": False}, "final_stop_missing"),
        (
            {"typed_stop_after_final_probe": False},
            "final_stop_not_after_probe",
        ),
        ({"final_stop_age_s": None}, "final_stop_stale"),
        ({"final_stop_age_s": 5.01}, "final_stop_stale"),
    ],
)
def test_missing_runtime_evidence_cannot_complete(overrides, reason):
    evidence = replace(_passing_evidence(), **overrides)

    result = assess_bounded_frontier_saturation(evidence)

    assert result.decision is SaturationDecision.CONTINUE
    assert reason in result.unmet_requirements


def test_unused_recovery_budget_does_not_block_hard_budget_saturation():
    """未遇到 provider 恢复原因时，不应为“花完恢复次数”制造无意义运动。"""

    result = assess_bounded_frontier_saturation(
        replace(_passing_evidence(), recovery_attempts_remaining=2)
    )

    assert result.decision is SaturationDecision.COMPLETE


def test_repeated_reachable_stall_requires_all_confirmation_evidence():
    result = assess_bounded_frontier_saturation(_repeated_stall_evidence())

    assert result.decision is SaturationDecision.COMPLETE
    assert result.unmet_requirements == ()


@pytest.mark.parametrize(
    ("overrides", "reason"),
    [
        (
            {"recovery_attempts_remaining": 1},
            "stall_confirmation_budget_remaining",
        ),
        (
            {"residual_available_frontiers": 1},
            "reachable_frontiers_remain_after_stall",
        ),
    ],
)
def test_repeated_stall_cannot_skip_confirmation_or_leave_reachable_frontier(
    overrides,
    reason,
):
    result = assess_bounded_frontier_saturation(
        replace(_repeated_stall_evidence(), **overrides)
    )

    assert result.decision is SaturationDecision.CONTINUE
    assert reason in result.unmet_requirements


def test_two_recent_low_yield_epochs_are_required():
    evidence = _passing_evidence()
    history = replace(evidence.history, epochs=evidence.history.epochs[-1:])

    result = assess_bounded_frontier_saturation(
        replace(evidence, history=history)
    )

    assert result.decision is SaturationDecision.CONTINUE
    assert "insufficient_low_yield_epochs" in result.unmet_requirements


def test_each_low_yield_epoch_needs_three_terminal_goals():
    evidence = _passing_evidence()
    epochs = list(evidence.history.epochs)
    last = epochs[-1]
    epochs[-1] = replace(
        last,
        end=replace(
            last.end,
            terminal_goal_count=last.start.terminal_goal_count + 2,
        ),
    )
    history = replace(evidence.history, epochs=tuple(epochs))

    result = assess_bounded_frontier_saturation(
        replace(evidence, history=history)
    )

    assert result.decision is SaturationDecision.CONTINUE
    assert "insufficient_terminal_goals_per_epoch" in result.unmet_requirements


@pytest.mark.parametrize("gain_cells", [151, 300])
def test_epoch_per_terminal_cell_and_ratio_gain_can_be_material(gain_cells):
    evidence = _passing_evidence()
    epochs = list(evidence.history.epochs)
    last = epochs[-1]
    epochs[-1] = replace(
        last,
        end=replace(
            last.end,
            peak_known_cells=last.start.peak_known_cells + gain_cells,
        ),
    )
    history = replace(evidence.history, epochs=tuple(epochs))

    result = assess_bounded_frontier_saturation(
        replace(evidence, history=history)
    )

    assert result.decision is SaturationDecision.CONTINUE
    assert "recent_epochs_still_material" in result.unmet_requirements


def test_epoch_is_low_yield_when_only_one_per_terminal_threshold_is_met():
    start = ExplorationCounters(25_000, 0, 0.0)
    ratio_boundary_epoch = ExplorationEpochEvidence(
        epoch_id=2,
        start=start,
        end=ExplorationCounters(25_120, 3, 10.0),
    )
    assert ratio_boundary_epoch.map_gain_cells_per_terminal == 40
    assert ratio_boundary_epoch.map_gain_ratio_per_terminal < 0.002

    evidence = _passing_evidence()
    history = replace(
        evidence.history,
        epochs=(evidence.history.epochs[0], ratio_boundary_epoch),
    )
    result = assess_bounded_frontier_saturation(
        replace(evidence, history=history)
    )

    assert "recent_epochs_still_material" not in result.unmet_requirements


def test_twenty_metres_of_mapping_path_is_a_hard_floor():
    evidence = _passing_evidence()
    history = replace(
        evidence.history,
        counters=replace(evidence.history.counters, mapping_path_m=19.99),
    )

    result = assess_bounded_frontier_saturation(
        replace(evidence, history=history)
    )

    assert result.decision is SaturationDecision.CONTINUE
    assert "insufficient_mapping_path" in result.unmet_requirements


def test_final_probe_must_exist_and_match_saturation_trigger():
    evidence = _passing_evidence()

    missing = assess_bounded_frontier_saturation(
        replace(evidence, history=replace(evidence.history, final_probe=None))
    )
    mismatched_probe = replace(
        evidence.history.final_probe,
        trigger=SaturationTrigger.REPEATED_REACHABLE_STALL,
    )
    assert isinstance(mismatched_probe, FinalProbeEvidence)
    mismatched = assess_bounded_frontier_saturation(
        replace(
            evidence,
            history=replace(
                evidence.history,
                final_probe=mismatched_probe,
            ),
        )
    )

    assert "final_probe_missing" in missing.unmet_requirements
    assert "final_probe_trigger_mismatch" in mismatched.unmet_requirements


def test_final_probe_gain_must_be_below_both_thresholds():
    evidence = _passing_evidence()
    probe = evidence.history.final_probe
    assert probe is not None
    material_probe = replace(
        probe,
        end=replace(
            probe.end,
            peak_known_cells=probe.start.peak_known_cells + 40,
        ),
    )

    result = assess_bounded_frontier_saturation(
        replace(
            evidence,
            history=replace(evidence.history, final_probe=material_probe),
        )
    )

    assert result.decision is SaturationDecision.CONTINUE
    assert "final_probe_still_material" in result.unmet_requirements


def test_loop_closure_current_shrink_cannot_hide_material_epoch_growth():
    tracker = SaturationEvidenceTracker()
    tracker.observe(
        current_known_cells=10_000,
        terminal_goal_count=0,
        mapping_path_m=0.0,
    )
    tracker.begin_epoch(1)
    tracker.observe(
        current_known_cells=10_120,
        terminal_goal_count=3,
        mapping_path_m=10.0,
    )
    # 回环优化会重画栅格并让当前 known 数缩小。峰值证据必须保留此前真实
    # 出现过的 120-cell 扩图，否则会把高收益 epoch 错判成 0 收益。
    tracker.observe(
        current_known_cells=9_000,
        terminal_goal_count=3,
        mapping_path_m=10.0,
    )
    first = tracker.finish_epoch()
    tracker.begin_epoch(2)
    tracker.observe(
        current_known_cells=10_140,
        terminal_goal_count=6,
        mapping_path_m=20.0,
    )
    tracker.finish_epoch()
    tracker.begin_final_probe(trigger=SaturationTrigger.TIME_BUDGET)
    tracker.observe(
        current_known_cells=10_150,
        terminal_goal_count=6,
        mapping_path_m=20.0,
    )
    tracker.finish_final_probe()

    assert first.map_gain_cells == 120
    evidence = replace(_passing_evidence(), history=tracker.snapshot())
    result = assess_bounded_frontier_saturation(evidence)
    assert result.decision is SaturationDecision.CONTINUE
    assert "recent_epochs_still_material" in result.unmet_requirements


def test_tracker_rejects_reset_task_totals_instead_of_fabricating_low_yield():
    tracker = SaturationEvidenceTracker()
    tracker.observe(
        current_known_cells=10_000,
        terminal_goal_count=4,
        mapping_path_m=21.0,
    )

    with pytest.raises(ValueError, match="terminal goal total must be monotonic"):
        tracker.observe(
            current_known_cells=10_010,
            terminal_goal_count=0,
            mapping_path_m=21.0,
        )
    with pytest.raises(ValueError, match="mapping path total must be monotonic"):
        tracker.observe(
            current_known_cells=10_010,
            terminal_goal_count=4,
            mapping_path_m=0.0,
        )
