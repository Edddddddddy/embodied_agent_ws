"""Automatic mission transaction tests without ROS runtime."""

from dataclasses import replace
from pathlib import Path
import time

import pytest

from embodied_slam_tools.mapping_evidence import (
    FrontierTelemetry,
    MappingEvidenceTracker,
)
from embodied_slam_tools.mission_executor import (
    AutomaticMissionCancelled,
    AutomaticMissionExecutor,
    AutomaticMissionSpec,
    CommandRequest,
    EpochRecoveryDecision,
    UnknownWorldMissionExecutor,
    UnknownWorldMissionSpec,
    decide_epoch_recovery,
)
from embodied_slam_tools.showcase_session import SessionCommand, SessionPhase


class _Manager:
    def __init__(self, *, dry_run=True):
        self.dry_run = dry_run
        self.calls = []

    def start_explorer(self, config_path):
        self.calls.append(("start_explorer", config_path))

    def stop_explorer(self):
        self.calls.append(("stop_explorer",))


class _Runtime:
    def __init__(self, evidence):
        self.evidence = evidence
        self.calls = []
        self.frontier_error = None
        self.action_error_for = None

    def transition(self, phase, **kwargs):
        self.calls.append(("transition", phase, kwargs["detail"]))

    def feedback(self, _request, progress):
        self.calls.append(("feedback", progress))

    def run_agent_action(
        self,
        _request,
        *,
        text,
        expected_action,
        timeout_s,
    ):
        self.calls.append(
            ("action", text, expected_action, timeout_s)
        )
        if expected_action == self.action_error_for:
            raise RuntimeError(f"{expected_action} failed")

    def wait_for_frontier(self, _request):
        self.calls.append(("wait_frontier",))
        if self.frontier_error is not None:
            raise self.frontier_error
        self.evidence.record_map([0] * 8 + [100, 100])
        self.evidence.record_odom(0.0, 0.0)
        self.evidence.record_odom(0.4, 0.0)
        self.evidence.completion_reason = "coverage_plateau"

    def save_map(self, _request):
        self.calls.append(("save_map",))

    def start_navigation(self, _request):
        self.calls.append(("start_navigation",))

    def wait_navigation_ready(self, _request):
        self.calls.append(("wait_navigation_ready",))


def _spec():
    return AutomaticMissionSpec(
        explorer_config_path=Path("/tmp/frontier.yaml"),
        bootstrap_route=(
            ("离开充电角", "前进三秒", "move"),
            ("转向通道", "左转九十度", "turn"),
        ),
        scan_startup_timeout_s=0.1,
        bootstrap_action_timeout_s=4.0,
        navigation_timeout_s=12.0,
        navigate_text="去入口",
        patrol_text="依次去厨房、办公室",
    )


def _request():
    return CommandRequest(
        command=SessionCommand.RUN_AUTOMATIC_MISSION,
        source="test",
    )


def test_successful_transaction_preserves_order_and_final_reason():
    evidence = MappingEvidenceTracker(1)
    manager = _Manager()
    runtime = _Runtime(evidence)
    executor = AutomaticMissionExecutor(runtime, manager, evidence, _spec())

    executor.run(_request())

    named_calls = [call[0] for call in runtime.calls]
    assert named_calls == [
        "transition",
        "feedback",
        "transition",
        "action",
        "feedback",
        "transition",
        "action",
        "feedback",
        "transition",
        "wait_frontier",
        "transition",
        "feedback",
        "save_map",
        "start_navigation",
        "wait_navigation_ready",
        "transition",
        "feedback",
        "action",
        "feedback",
        "action",
        "transition",
        "feedback",
    ]
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    assert runtime.calls[-2][1] == SessionPhase.MISSION_COMPLETED
    assert "coverage_plateau" in runtime.calls[-2][2]
    assert evidence.mapping_path_m == pytest.approx(0.4)


def test_frontier_failure_always_stops_explorer_and_freezes_path():
    evidence = MappingEvidenceTracker(1)
    manager = _Manager()
    runtime = _Runtime(evidence)
    runtime.frontier_error = RuntimeError("frontier failed")
    executor = AutomaticMissionExecutor(runtime, manager, evidence, _spec())

    with pytest.raises(RuntimeError, match="frontier failed"):
        executor.run(_request())

    assert manager.calls[-1] == ("stop_explorer",)
    evidence.record_odom(0.0, 0.0)
    evidence.record_odom(0.3, 0.0)
    assert evidence.mapping_path_m == 0.0


def test_cancel_after_map_save_never_starts_navigation():
    evidence = MappingEvidenceTracker(1)
    manager = _Manager()

    class _CancelAfterSaveRuntime(_Runtime):
        def save_map(self, request):
            super().save_map(request)
            request.canceled = True

    runtime = _CancelAfterSaveRuntime(evidence)
    executor = AutomaticMissionExecutor(runtime, manager, evidence, _spec())

    with pytest.raises(AutomaticMissionCancelled):
        executor.run(_request())

    assert ("start_navigation",) not in runtime.calls


def test_navigation_failure_stops_transaction_before_patrol_completion():
    evidence = MappingEvidenceTracker(1)
    manager = _Manager()
    runtime = _Runtime(evidence)
    runtime.action_error_for = "navigate_to"
    executor = AutomaticMissionExecutor(runtime, manager, evidence, _spec())

    with pytest.raises(RuntimeError, match="navigate_to failed"):
        executor.run(_request())

    action_types = [
        call[2] for call in runtime.calls if call[0] == "action"
    ]
    assert "navigate_to" in action_types
    assert "follow_waypoints" not in action_types
    assert not any(
        call[0] == "transition" and call[1] == SessionPhase.MISSION_COMPLETED
        for call in runtime.calls
    )


def test_live_runtime_requires_scan_before_first_motion():
    evidence = MappingEvidenceTracker(1)
    manager = _Manager(dry_run=False)
    runtime = _Runtime(evidence)
    spec = replace(_spec(), scan_startup_timeout_s=0.001)
    executor = AutomaticMissionExecutor(runtime, manager, evidence, spec)

    with pytest.raises(TimeoutError, match="mapping scan"):
        executor.run(_request())

    assert not any(call[0] == "action" for call in runtime.calls)
    evidence.record_odom(0.0, 0.0)
    evidence.record_odom(0.3, 0.0)
    assert evidence.mapping_path_m == 0.0


class _UnknownWorldRuntime(_Runtime):
    def __init__(
        self,
        evidence,
        frontier_outcomes,
        *,
        recovery_map_sizes=(),
        frontier_telemetries=(),
    ):
        super().__init__(evidence)
        self.frontier_outcomes = list(frontier_outcomes)
        self.recovery_map_sizes = list(recovery_map_sizes)
        self.frontier_telemetries = list(frontier_telemetries)
        self.scan_action_count = 0

    def run_agent_action(
        self,
        request,
        *,
        text,
        expected_action,
        timeout_s,
    ):
        super().run_agent_action(
            request,
            text=text,
            expected_action=expected_action,
            timeout_s=timeout_s,
        )
        self.scan_action_count += 1
        if self.scan_action_count > 1 and self.recovery_map_sizes:
            self.evidence.record_map([0] * self.recovery_map_sizes.pop(0))

    def wait_for_frontier(
        self,
        _request,
        *,
        recovery_attempts_remaining,
        deadline_monotonic,
    ):
        self.calls.append(
            ("wait_frontier", recovery_attempts_remaining, deadline_monotonic)
        )
        if self.frontier_telemetries:
            self.evidence.record_frontier_telemetry(
                self.frontier_telemetries.pop(0)
            )
        return self.frontier_outcomes.pop(0)

    def stop_motion_and_wait(self, _request, *, timeout_s):
        self.calls.append(("stop_motion", timeout_s))

    def select_mapped_navigation_goals(
        self,
        _request,
        *,
        count,
        seed,
        minimum_separation_m,
        clearance_m,
        timeout_s,
    ):
        self.calls.append(
            (
                "select_goals",
                count,
                seed,
                minimum_separation_m,
                clearance_m,
                timeout_s,
            )
        )
        return ((1.0, 1.0), (2.0, 1.0), (2.0, 2.0))

    def run_navigation_goal(self, _request, *, sequence, goal_xy, timeout_s):
        self.calls.append(("navigation_goal", sequence, goal_xy, timeout_s))


def _unknown_spec(max_recovery_attempts=2):
    return UnknownWorldMissionSpec(
        explorer_config_path=Path("/tmp/frontier.yaml"),
        scan_startup_timeout_s=0.1,
        exploration_timeout_s=600.0,
        action_timeout_s=45.0,
        navigation_timeout_s=120.0,
        initial_scan_text="原地转一圈",
        recovery_scan_text="原地转一圈",
        max_recovery_attempts=max_recovery_attempts,
        minimum_epoch_map_gain_cells=40,
        map_settle_s=0.0,
        navigation_goal_count=3,
        navigation_goal_seed=20260719,
        navigation_goal_minimum_separation_m=1.5,
        navigation_goal_clearance_m=0.25,
    )


def test_unknown_world_recovers_without_scene_route_then_samples_map_goals():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        (
            "recovery_required:blacklisted_frontiers",
            "no_reachable_frontiers",
        ),
        recovery_map_sizes=(150,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    executor.run(_request())

    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    actions = [call for call in runtime.calls if call[0] == "action"]
    assert [call[1] for call in actions] == ["原地转一圈", "原地转一圈"]
    assert ("stop_motion", 45.0) in runtime.calls
    frontier_waits = [call for call in runtime.calls if call[0] == "wait_frontier"]
    # recovery 只重启 provider，不得把整轮 exploration deadline 延长 600s。
    assert frontier_waits[0][2] == frontier_waits[1][2]
    decision_details = [
        call[2]
        for call in runtime.calls
        if call[0] == "transition"
        and "frontier epoch recovery evaluated" in call[2]
    ]
    assert decision_details == [
        "frontier epoch recovery evaluated "
        "reason=recovery_required:blacklisted_frontiers "
        "known_before=100 known_after=150 gain=50 threshold=40 "
        "decision=advance_epoch"
    ]
    assert not any("前进" in call[1] for call in actions)
    assert len(
        [call for call in runtime.calls if call[0] == "navigation_goal"]
    ) == 3
    assert runtime.calls[-2][1] == SessionPhase.MISSION_COMPLETED


def test_unknown_world_reports_frontier_progress_instead_of_stale_initialization():
    evidence = MappingEvidenceTracker(1)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(evidence, ("no_reachable_frontiers",))
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    executor.run(_request())

    details = [
        call[2]
        for call in runtime.calls
        if call[0] == "transition"
        and call[1] == SessionPhase.AUTOMATIC_MAPPING
    ]
    assert any(detail.startswith("frontier exploration running") for detail in details)


def test_unknown_world_recovery_budget_exhaustion_is_explicit_failure():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        (
            "recovery_required:blacklisted_frontiers",
            "recovery_required:blacklisted_frontiers",
        ),
        recovery_map_sizes=(150, 200),
    )
    executor = UnknownWorldMissionExecutor(
        runtime,
        manager,
        evidence,
        _unknown_spec(max_recovery_attempts=1),
    )

    with pytest.raises(
        RuntimeError,
        match="gained map but recovery budget is exhausted",
    ):
        executor.run(_request())

    # 第一次高增益扫描消费唯一预算；第二次即使预算为零仍要完成扫描和审计，
    # 但不能启动第三个 Explorer 进程。
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    waits = [call for call in runtime.calls if call[0] == "wait_frontier"]
    assert [call[1] for call in waits] == [1, 0]
    assert len([call for call in runtime.calls if call[0] == "action"]) == 3


def test_exhausted_frontiers_get_final_confirmation_after_restart_budget_used():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        (
            "recovery_required:blacklisted_frontiers",
            "recovery_required:frontier_attempts_exhausted",
        ),
        recovery_map_sizes=(150, 160),
    )
    executor = UnknownWorldMissionExecutor(
        runtime,
        manager,
        evidence,
        _unknown_spec(max_recovery_attempts=1),
    )

    executor.run(_request())

    # 第一次扫描消费的是“重启 Explorer”预算；第二次扫描只确认地图是否仍增长，
    # 因此不能因为 remaining=0 而跳过，也不能偷偷启动第三轮 Explorer。
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    assert evidence.completion_reason == (
        "frontier_attempts_exhausted_no_map_gain"
    )
    assert len([call for call in runtime.calls if call[0] == "action"]) == 3
    assert len(
        [call for call in runtime.calls if call[0] == "navigation_goal"]
    ) == 3


def test_final_confirmation_with_map_gain_fails_without_restart_budget():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        (
            "recovery_required:blacklisted_frontiers",
            "recovery_required:frontier_attempts_exhausted",
        ),
        recovery_map_sizes=(150, 200),
    )
    executor = UnknownWorldMissionExecutor(
        runtime,
        manager,
        evidence,
        _unknown_spec(max_recovery_attempts=1),
    )

    with pytest.raises(
        RuntimeError,
        match="gained map but recovery budget is exhausted",
    ):
        executor.run(_request())

    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    assert evidence.completion_reason != (
        "frontier_attempts_exhausted_no_map_gain"
    )
    assert len([call for call in runtime.calls if call[0] == "action"]) == 3
    assert not any(call[0] == "navigation_goal" for call in runtime.calls)


def test_final_confirmation_settles_after_exploration_deadline_expired(
    monkeypatch,
):
    class _AdvancingClock:
        def __init__(self):
            self.now = 0.0

        def __call__(self):
            current = self.now
            self.now += 0.25
            return current

        def advance(self, seconds):
            self.now += seconds

    clock = _AdvancingClock()
    monkeypatch.setattr(time, "monotonic", clock)
    evidence = MappingEvidenceTracker(1, clock=clock)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)

    class _DeadlineExpiringRuntime(_UnknownWorldRuntime):
        def run_agent_action(self, request, **kwargs):
            if self.scan_action_count > 0:
                # 模拟 Explorer 在 900s 边界返回：确认转圈结束时旧探索 deadline
                # 已过期，但动作自己的 timeout 仍然有界且已经成功完成。
                clock.advance(2.0)
            super().run_agent_action(request, **kwargs)

    runtime = _DeadlineExpiringRuntime(
        evidence,
        ("recovery_required:frontier_attempts_exhausted",),
        recovery_map_sizes=(110,),
    )
    spec = replace(
        _unknown_spec(max_recovery_attempts=0),
        exploration_timeout_s=1.0,
        map_settle_s=0.5,
    )
    executor = UnknownWorldMissionExecutor(runtime, manager, evidence, spec)

    executor.run(_request())

    assert evidence.completion_reason == (
        "frontier_attempts_exhausted_no_map_gain"
    )
    assert len([call for call in runtime.calls if call[0] == "action"]) == 2
    assert len(
        [call for call in runtime.calls if call[0] == "navigation_goal"]
    ) == 3


def test_map_quiet_window_extends_when_map_grows_while_waiting(monkeypatch):
    class _Clock:
        def __init__(self):
            self.now = 0.0

        def __call__(self):
            return self.now

        def advance(self, seconds):
            self.now += seconds

    class _AdvancingCondition:
        """不用真实 sleep，确定性模拟 settle 期间到达一帧新地图。"""

        def __init__(self, clock, on_first_wait):
            self._clock = clock
            self._on_first_wait = on_first_wait
            self._wait_count = 0

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def notify_all(self):
            return None

        def wait(self, timeout):
            self._clock.advance(timeout)
            self._wait_count += 1
            if self._wait_count == 1:
                self._on_first_wait()

    clock = _Clock()
    monkeypatch.setattr(time, "monotonic", clock)
    evidence = MappingEvidenceTracker(1, clock=clock)
    evidence.record_map([0] * 100)
    executor = UnknownWorldMissionExecutor(
        _UnknownWorldRuntime(evidence, ()),
        _Manager(),
        evidence,
        replace(_unknown_spec(), map_settle_s=1.0),
    )
    evidence.condition = _AdvancingCondition(
        clock,
        lambda: evidence.record_map([0] * 120),
    )

    snapshot = executor._wait_for_map_quiet(
        _request(),
        deadline_monotonic=2.0,
    )

    assert snapshot.map_stats == {
        "known_cells": 120,
        "occupied_cells": 0,
    }
    # 新地图在 t=0.1 到达，完整 quiet window 应延展到 t=1.1。
    assert clock.now == pytest.approx(1.1)


def test_map_quiet_wait_times_out_when_map_never_becomes_quiet(monkeypatch):
    class _Clock:
        def __init__(self):
            self.now = 0.0

        def __call__(self):
            return self.now

        def advance(self, seconds):
            self.now += seconds

    class _ContinuouslyGrowingCondition:
        def __init__(self, clock, evidence):
            self._clock = clock
            self._evidence = evidence
            self._known_cells = 100

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def notify_all(self):
            return None

        def wait(self, timeout):
            self._clock.advance(timeout)
            # 持续增长到原硬 deadline 之后；旧实现会不断续期并在增长停止后
            # 错误返回，正确实现应在 t=1.0 直接结束等待并报超时。
            if self._known_cells < 120:
                self._known_cells += 1
                self._evidence.record_map([0] * self._known_cells)

    clock = _Clock()
    monkeypatch.setattr(time, "monotonic", clock)
    evidence = MappingEvidenceTracker(1, clock=clock)
    evidence.record_map([0] * 100)
    executor = UnknownWorldMissionExecutor(
        _UnknownWorldRuntime(evidence, ()),
        _Manager(),
        evidence,
        replace(_unknown_spec(), map_settle_s=0.5),
    )
    evidence.condition = _ContinuouslyGrowingCondition(clock, evidence)

    with pytest.raises(
        TimeoutError,
        match="map did not settle before confirmation deadline",
    ):
        executor._wait_for_map_quiet(
            _request(),
            deadline_monotonic=1.0,
        )

    assert clock.now == pytest.approx(1.0)


def test_confirmation_hard_deadline_uses_action_timeout_budget(monkeypatch):
    class _Clock:
        def __init__(self):
            self.now = 10.0

        def __call__(self):
            return self.now

    clock = _Clock()
    monkeypatch.setattr(time, "monotonic", clock)
    evidence = MappingEvidenceTracker(1, clock=clock)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    runtime = _UnknownWorldRuntime(
        evidence,
        ("recovery_required:frontier_attempts_exhausted",),
        recovery_map_sizes=(110,),
    )

    class _CapturingExecutor(UnknownWorldMissionExecutor):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.hard_deadline = None

        def _wait_for_map_quiet(self, request, *, deadline_monotonic):
            self.hard_deadline = deadline_monotonic
            return self._evidence.snapshot()

    spec = replace(
        _unknown_spec(max_recovery_attempts=0),
        action_timeout_s=3.0,
        map_settle_s=0.5,
    )
    executor = _CapturingExecutor(runtime, _Manager(dry_run=False), evidence, spec)

    executor.run(_request())

    assert executor.hard_deadline == pytest.approx(13.0)


def test_exhausted_frontiers_finish_after_recovery_scan_has_no_map_gain():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        ("recovery_required:frontier_attempts_exhausted",),
        recovery_map_sizes=(110,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    executor.run(_request())

    # 恢复扫描只新增 10 个栅格，小于统一阈值 40；无需再重复一整轮 Explorer。
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    assert evidence.completion_reason == (
        "frontier_attempts_exhausted_no_map_gain"
    )
    assert len([call for call in runtime.calls if call[0] == "action"]) == 2
    assert len(
        [call for call in runtime.calls if call[0] == "navigation_goal"]
    ) == 3


def test_exhausted_frontiers_restart_only_when_recovery_scan_grows_map():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        (
            "recovery_required:frontier_attempts_exhausted",
            "no_reachable_frontiers",
        ),
        recovery_map_sizes=(150,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    executor.run(_request())

    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    assert evidence.completion_reason == "no_reachable_frontiers"


@pytest.mark.parametrize(
    "outcome",
    [
        "recovery_required:blacklisted_frontiers",
        "recovery_required:reachable_frontiers_stalled",
    ],
)
def test_nonterminal_recovery_with_low_gain_fails_without_new_epoch(outcome):
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        (outcome,),
        recovery_map_sizes=(110,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    with pytest.raises(
        RuntimeError,
        match="recovery produced insufficient map gain",
    ):
        executor.run(_request())

    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    assert len([call for call in runtime.calls if call[0] == "action"]) == 2
    assert not any(call[0] == "navigation_goal" for call in runtime.calls)
    detail = next(
        call[2]
        for call in runtime.calls
        if call[0] == "transition"
        and "frontier epoch recovery evaluated" in call[2]
    )
    assert f"reason={outcome}" in detail
    assert "known_before=100 known_after=110 gain=10 threshold=40" in detail
    assert "decision=fail_no_gain" in detail


@pytest.mark.parametrize(
    "telemetry",
    [
        FrontierTelemetry(available_frontier_count=1),
        FrontierTelemetry(active_goal_count=1),
        FrontierTelemetry(blacklisted_frontier_count=1),
    ],
)
def test_attempt_exhaustion_low_gain_requires_all_frontier_counters_zero(
    telemetry,
):
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        ("recovery_required:frontier_attempts_exhausted",),
        recovery_map_sizes=(110,),
        frontier_telemetries=(telemetry,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    with pytest.raises(RuntimeError, match="insufficient map gain"):
        executor.run(_request())

    assert evidence.completion_reason != (
        "frontier_attempts_exhausted_no_map_gain"
    )
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]


def test_epoch_recovery_decision_is_pure_and_budget_is_spent_only_on_advance():
    common = {
        "recovery_reason": "recovery_required:blacklisted_frontiers",
        "minimum_gain_cells": 40,
        "available_frontier_count": 0,
        "active_goal_count": 0,
        "blacklisted_frontier_count": 1,
    }

    assert decide_epoch_recovery(
        **common,
        map_gain_cells=39,
        recovery_attempts_remaining=2,
    ) is EpochRecoveryDecision.FAIL_NO_GAIN
    assert decide_epoch_recovery(
        **common,
        map_gain_cells=40,
        recovery_attempts_remaining=2,
    ) is EpochRecoveryDecision.ADVANCE_EPOCH
    assert decide_epoch_recovery(
        **common,
        map_gain_cells=40,
        recovery_attempts_remaining=0,
    ) is EpochRecoveryDecision.FAIL_BUDGET_EXHAUSTED

    stalled = {
        **common,
        "recovery_reason": "recovery_required:frontier_progress_stalled",
    }
    # 停滞熔断只授权一次传感器恢复，不授权“无增益即完成”；只有扫描确实
    # 增加本次地图，才可消费预算并开启新的 explorer epoch。
    assert decide_epoch_recovery(
        **stalled,
        map_gain_cells=39,
        recovery_attempts_remaining=2,
    ) is EpochRecoveryDecision.FAIL_NO_GAIN
    assert decide_epoch_recovery(
        **stalled,
        map_gain_cells=40,
        recovery_attempts_remaining=2,
    ) is EpochRecoveryDecision.ADVANCE_EPOCH


def test_recovery_scan_exception_never_restarts_explorer():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)

    class _FailingRecoveryScanRuntime(_UnknownWorldRuntime):
        def run_agent_action(self, request, **kwargs):
            if self.scan_action_count == 1:
                raise RuntimeError("recovery scan failed")
            super().run_agent_action(request, **kwargs)

    runtime = _FailingRecoveryScanRuntime(
        evidence,
        ("recovery_required:blacklisted_frontiers",),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    with pytest.raises(RuntimeError, match="recovery scan failed"):
        executor.run(_request())

    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]


def test_recovery_cancellation_never_restarts_explorer():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)

    class _CanceledRecoveryRuntime(_UnknownWorldRuntime):
        def stop_motion_and_wait(self, request, *, timeout_s):
            super().stop_motion_and_wait(request, timeout_s=timeout_s)
            request.canceled = True

    runtime = _CanceledRecoveryRuntime(
        evidence,
        ("recovery_required:blacklisted_frontiers",),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    with pytest.raises(AutomaticMissionCancelled, match="mission canceled"):
        executor.run(_request())

    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    # 只有初始 360° 扫描；取消发生后不得再执行恢复扫描。
    assert len([call for call in runtime.calls if call[0] == "action"]) == 1


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        (
            "minimum_epoch_map_gain_cells",
            -1,
            "minimum epoch map gain cells must be non-negative",
        ),
        ("map_settle_s", -0.1, "map settle time must be finite"),
        ("map_settle_s", float("inf"), "map settle time must be finite"),
    ],
)
def test_unknown_world_spec_rejects_invalid_convergence_values(
    field,
    value,
    message,
):
    with pytest.raises(ValueError, match=message):
        replace(_unknown_spec(), **{field: value})
