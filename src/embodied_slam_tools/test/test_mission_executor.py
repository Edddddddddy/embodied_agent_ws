"""Automatic mission transaction tests without ROS runtime."""

from dataclasses import replace
from pathlib import Path
import time

import pytest

from embodied_slam_tools.exploration_saturation import (
    SaturationPolicy,
    SaturationTrigger,
)
from embodied_slam_tools.mapping_evidence import (
    FrontierTelemetry,
    MappingEvidenceTracker,
)
from embodied_slam_tools.mapping_return import (
    PlanarPose,
    ReturnActionKind,
    ReturnActionStatus,
    ReturnToStartEvidence,
)
from embodied_slam_tools.mission_executor import (
    AutomaticMissionCancelled,
    AutomaticMissionExecutor,
    AutomaticMissionSpec,
    CommandRequest,
    EpochRecoveryDecision,
    RecoveryBackUpSpec,
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

    def quiesce_frontier(self, _request, *, timeout_s):
        self.calls.append(("quiesce_frontier", timeout_s))
        return self.evidence.snapshot().frontier_telemetry

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

    assert ("quiesce_frontier", 4.0) in runtime.calls
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
        recovery_backup_displacements=(),
    ):
        super().__init__(evidence)
        self.frontier_outcomes = list(frontier_outcomes)
        self.recovery_map_sizes = list(recovery_map_sizes)
        self.frontier_telemetries = list(frontier_telemetries)
        self.recovery_backup_displacements = list(
            recovery_backup_displacements
        )
        self.scan_action_count = 0

    def capture_mapping_start_pose(self, _request, *, timeout_s):
        del timeout_s
        return PlanarPose(0.0, 0.0, 0.0, "map", 1)

    def quiesce_frontier(self, _request, *, timeout_s):
        del timeout_s
        return self.evidence.snapshot().frontier_telemetry

    def run_mapping_return_goal(
        self,
        _request,
        *,
        start_pose,
        spec,
        timeout_s,
    ):
        del spec, timeout_s
        return ReturnToStartEvidence(
            start_pose=start_pose,
            final_pose=PlanarPose(0.0, 0.0, 0.0, "map", 4),
            action_kind=ReturnActionKind.NAVIGATE_TO_POSE,
            action_status=ReturnActionStatus.SUCCEEDED,
            action_command_id="test-return-home",
            action_started_at_ns=2,
            action_finished_at_ns=3,
            map_saved_at_ns=5,
            cmd_vel_linear_x=0.0,
            cmd_vel_angular_z=0.0,
            cmd_vel_observed_at_ns=4,
            evaluated_at_ns=5,
        )

    def record_mapping_completion(self, **_kwargs):
        return None

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

    def run_recovery_backup(
        self,
        _request,
        *,
        distance_m,
        speed_mps,
        timeout_s,
    ):
        self.calls.append(
            ("recovery_backup", distance_m, speed_mps, timeout_s)
        )
        return self.recovery_backup_displacements.pop(0)

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
        recovery_backup=RecoveryBackUpSpec(
            distance_m=0.30,
            speed_mps=0.08,
            timeout_s=10.0,
            minimum_displacement_m=0.20,
        ),
        max_recovery_attempts=max_recovery_attempts,
        minimum_epoch_map_gain_cells=40,
        minimum_epoch_map_gain_ratio=0.002,
        map_settle_s=0.0,
        navigation_goal_count=3,
        navigation_goal_seed=20260719,
        navigation_goal_minimum_separation_m=1.5,
        navigation_goal_clearance_m=0.25,
        return_map_settle_s=0.0,
    )


def test_time_budget_saturation_quiesces_returns_home_then_saves_map():
    """复现残余角落场景：安全收口后必须在 SLAM stage 返航再存图。"""

    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 25_000)
    evidence.mark_scan_ready()
    events = []

    class _EventManager(_Manager):
        def start_explorer(self, config_path):
            super().start_explorer(config_path)
            events.append("explorer_started")

        def stop_explorer(self):
            super().stop_explorer()
            events.append("explorer_stopped")

    class _SaturationRuntime(_UnknownWorldRuntime):
        def __init__(self):
            super().__init__(
                evidence,
                (
                    "recovery_required:blacklisted_frontiers",
                    "recovery_required:frontier_attempts_exhausted",
                    "assessment_required:time_budget",
                ),
                recovery_map_sizes=(25_400, 25_420, 25_450),
                recovery_backup_displacements=(0.24,),
            )
            self._epoch = 0
            self._odom_x = 0.0
            self.completion = None

        def capture_mapping_start_pose(self, request, *, timeout_s):
            events.append("start_pose_captured")
            return super().capture_mapping_start_pose(
                request, timeout_s=timeout_s
            )

        def wait_for_frontier(self, request, **kwargs):
            self._epoch += 1
            known_cells = (25_200, 25_420, 25_440)[self._epoch - 1]
            self.evidence.record_map([0] * known_cells)
            # 每个 epoch 形成约 7m 合法连续里程；总里程超过通用 20m 下限。
            for _ in range(21):
                self.evidence.record_odom(self._odom_x, 0.0)
                self._odom_x += 0.35
            self.evidence.record_frontier_telemetry(
                FrontierTelemetry(
                    status="exploration_blocked",
                    detected_frontier_count=8,
                    available_frontier_count=4,
                    blacklisted_frontier_count=1,
                    accepted_goal_count=3,
                    succeeded_goal_count=3,
                )
            )
            return super().wait_for_frontier(request, **kwargs)

        def quiesce_frontier(self, request, *, timeout_s):
            events.append("frontier_quiesced")
            self.evidence.record_frontier_telemetry(
                FrontierTelemetry(
                    status="exploration_paused",
                    detected_frontier_count=11,
                    # 复现真实现场：这是 final 360° probe 之前暂停
                    # Explorer 得到的旧 cluster 快照，只能用于诊断。
                    available_frontier_count=6,
                    blacklisted_frontier_count=0,
                    accepted_goal_count=3,
                    succeeded_goal_count=3,
                )
            )
            return super().quiesce_frontier(request, timeout_s=timeout_s)

        def run_mapping_return_goal(self, request, **kwargs):
            events.append("return_home")
            return super().run_mapping_return_goal(request, **kwargs)

        def record_mapping_completion(self, **kwargs):
            events.append("completion_recorded")
            self.completion = kwargs

        def save_map(self, request):
            events.append("map_saved")
            super().save_map(request)

    manager = _EventManager(dry_run=False)
    runtime = _SaturationRuntime()
    spec = replace(
        _unknown_spec(max_recovery_attempts=2),
        saturation_policy=SaturationPolicy(required_map_quiet_s=0.001),
    )

    UnknownWorldMissionExecutor(runtime, manager, evidence, spec).run(
        _request()
    )

    assert evidence.completion_reason == "time_budget_exhausted"
    assert events.count("explorer_started") == 3
    assert events.count("explorer_stopped") == 3
    assert events.index("frontier_quiesced") < events.index(
        "explorer_stopped", events.index("frontier_quiesced")
    )
    assert events.index("return_home") < events.index("map_saved")
    assert runtime.completion is not None
    assert runtime.completion["saturation_assessment"].complete
    assert runtime.completion["return_to_start"].action_status is (
        ReturnActionStatus.SUCCEEDED
    )
    assert len(
        [call for call in runtime.calls if call[0] == "navigation_goal"]
    ) == 3


def test_time_budget_adds_one_confirmation_epoch_when_history_is_material():
    """硬预算撞上高收益历史时，再观察一轮，不能立即失败或放宽完成门槛。"""

    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 25_000)
    evidence.mark_scan_ready()

    class _MaterialHistoryRuntime(_UnknownWorldRuntime):
        def __init__(self):
            super().__init__(
                evidence,
                (
                    "recovery_required:frontier_attempts_exhausted",
                    "assessment_required:time_budget",
                    "assessment_required:time_budget",
                ),
                # 第一次恢复仍显著扩图，final probe 只产生少量边缘细化。
                recovery_map_sizes=(25_500, 25_535),
                recovery_backup_displacements=(0.24,),
            )
            self._epoch = 0
            self._odom_x = 0.0

        def wait_for_frontier(self, request, **kwargs):
            self._epoch += 1
            known_cells = (25_400, 25_520, 25_530)[self._epoch - 1]
            self.evidence.record_map([0] * known_cells)
            for _ in range(21):
                self.evidence.record_odom(self._odom_x, 0.0)
                self._odom_x += 0.35
            self.evidence.record_frontier_telemetry(
                FrontierTelemetry(
                    status="exploration_blocked",
                    detected_frontier_count=8,
                    available_frontier_count=4,
                    blacklisted_frontier_count=1,
                    accepted_goal_count=3,
                    succeeded_goal_count=3,
                )
            )
            return super().wait_for_frontier(request, **kwargs)

    manager = _Manager(dry_run=False)
    runtime = _MaterialHistoryRuntime()
    spec = replace(
        _unknown_spec(max_recovery_attempts=2),
        exploration_timeout_s=0.1,
        final_confirmation_timeout_s=240.0,
        saturation_policy=SaturationPolicy(required_map_quiet_s=0.001),
    )

    UnknownWorldMissionExecutor(runtime, manager, evidence, spec).run(
        _request()
    )

    # 第三轮是独立的低收益确认 epoch；它复用同一地图和 Action 总账，
    # 不追加 BackUp，也不把原 900 秒预算重新赠送给普通探索。
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    waits = [call for call in runtime.calls if call[0] == "wait_frontier"]
    assert waits[0][2] == waits[1][2]
    assert waits[2][2] >= waits[1][2] + 239.0
    assert len(
        [call for call in runtime.calls if call[0] == "recovery_backup"]
    ) == 1
    assert evidence.completion_reason == "time_budget_exhausted"


def test_frontier_wait_failure_uses_independent_typed_stop_request():
    """Explorer 超时后，即使用户请求已取消也必须真正发出安全停车。"""

    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    events = []
    manager = _Manager(dry_run=False)

    class _FailingFrontierRuntime(_UnknownWorldRuntime):
        def wait_for_frontier(self, request, **_kwargs):
            events.append("wait_frontier")
            request.canceled = True
            raise TimeoutError("frontier deadline expired")

        def stop_motion_and_wait(self, request, *, timeout_s):
            events.append("typed_stop")
            assert request.command == SessionCommand.STOP_SESSION
            assert request.source == "frontier_failure_cleanup"
            assert request.canceled is False
            assert timeout_s == 45.0

    runtime = _FailingFrontierRuntime(evidence, ())
    request = _request()
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    with pytest.raises(TimeoutError, match="frontier deadline expired"):
        executor.run(request)

    assert request.canceled is True
    assert events == ["wait_frontier", "typed_stop"]
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]


def test_frontier_cleanup_failures_never_mask_primary_wait_error():
    """清理失败只能附注在主异常上，不能覆盖真正的 Frontier 根因。"""

    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    events = []

    class _FailingManager(_Manager):
        def stop_explorer(self):
            super().stop_explorer()
            events.append("stop_explorer_failed")
            raise RuntimeError("explorer cleanup failed")

    class _DoubleFailRuntime(_UnknownWorldRuntime):
        def wait_for_frontier(self, _request, **_kwargs):
            events.append("wait_frontier_failed")
            raise TimeoutError("frontier deadline expired")

        def stop_motion_and_wait(self, request, *, timeout_s):
            events.append("typed_stop_failed")
            assert request is not original_request
            raise RuntimeError("typed stop failed")

    manager = _FailingManager(dry_run=False)
    runtime = _DoubleFailRuntime(evidence, ())
    original_request = _request()
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    with pytest.raises(TimeoutError, match="frontier deadline expired") as error:
        executor.run(original_request)

    assert events == [
        "wait_frontier_failed",
        "stop_explorer_failed",
        "typed_stop_failed",
    ]
    notes = getattr(error.value, "__notes__", ())
    assert any("explorer cleanup failed" in note for note in notes)
    assert any("typed stop failed" in note for note in notes)


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
            "gain_ratio=0.500000 ratio_threshold=0.002000 "
            "detected=0 available=0 active=0 blacklisted=0 "
            "displacement=none "
            "completed_recoveries=0 "
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
        match="material map growth but recovery budget is exhausted",
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
        "frontier_attempts_exhausted_below_material_gain"
    )
    assert len([call for call in runtime.calls if call[0] == "action"]) == 3
    assert len(
        [call for call in runtime.calls if call[0] == "navigation_goal"]
    ) == 3


def test_material_gain_at_budget_boundary_runs_one_post_scan_confirmation_epoch():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        (
            "recovery_required:frontier_attempts_exhausted",
            "recovery_required:frontier_attempts_exhausted",
            "recovery_required:frontier_attempts_exhausted",
        ),
        # 第一轮只靠真实 BackUp 位移开启新 epoch；预算边界上的第二次
        # 扫描出现显著增长后，必须让 Explorer 消费这张新图再判收敛。
        recovery_map_sizes=(102, 150),
        recovery_backup_displacements=(0.24,),
    )
    spec = replace(
        _unknown_spec(max_recovery_attempts=1),
        # 复现现场：最终确认启动时，共享 exploration deadline 只剩极短时间。
        exploration_timeout_s=0.1,
        final_confirmation_timeout_s=240.0,
    )
    executor = UnknownWorldMissionExecutor(
        runtime,
        manager,
        evidence,
        spec,
    )

    executor.run(_request())

    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    assert evidence.completion_reason == (
        "frontier_attempts_exhausted_after_final_confirmation"
    )
    # 最终确认 epoch 不再 BackUp，也不再做一遍无人消费的恢复扫描。
    assert len([call for call in runtime.calls if call[0] == "action"]) == 3
    assert len(
        [call for call in runtime.calls if call[0] == "recovery_backup"]
    ) == 1
    assert len(
        [call for call in runtime.calls if call[0] == "navigation_goal"]
    ) == 3
    details = [
        call[2]
        for call in runtime.calls
        if call[0] == "transition"
    ]
    assert any("decision=run_final_confirmation_epoch" in item for item in details)
    waits = [call for call in runtime.calls if call[0] == "wait_frontier"]
    assert waits[0][2] == waits[1][2]
    assert waits[2][2] >= waits[1][2] + 239.0


def test_final_confirmation_rejects_a_second_nonterminal_recovery_reason():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        (
            "recovery_required:frontier_attempts_exhausted",
            "recovery_required:frontier_attempts_exhausted",
            "recovery_required:frontier_progress_stalled",
        ),
        recovery_map_sizes=(102, 150),
        recovery_backup_displacements=(0.24,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime,
        manager,
        evidence,
        _unknown_spec(max_recovery_attempts=1),
    )

    with pytest.raises(
        RuntimeError,
        match="final frontier confirmation did not converge",
    ):
        executor.run(_request())

    # 确认轮只能观察 provider；失败时也不得偷偷追加 BackUp 或恢复扫描。
    assert len([call for call in runtime.calls if call[0] == "action"]) == 3
    assert len(
        [call for call in runtime.calls if call[0] == "recovery_backup"]
    ) == 1
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
        (
            "recovery_required:blacklisted_frontiers",
            "recovery_required:frontier_attempts_exhausted",
        ),
        recovery_map_sizes=(150, 160),
    )
    spec = replace(
        _unknown_spec(max_recovery_attempts=1),
        exploration_timeout_s=1.0,
        map_settle_s=0.5,
    )
    executor = UnknownWorldMissionExecutor(runtime, manager, evidence, spec)

    executor.run(_request())

    assert evidence.completion_reason == (
        "frontier_attempts_exhausted_below_material_gain"
    )
    assert len([call for call in runtime.calls if call[0] == "action"]) == 3
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
        (
            "recovery_required:blacklisted_frontiers",
            "recovery_required:frontier_attempts_exhausted",
        ),
        recovery_map_sizes=(150, 160),
    )

    class _CapturingExecutor(UnknownWorldMissionExecutor):
        def __init__(self, *args, **kwargs):
            super().__init__(*args, **kwargs)
            self.hard_deadline = None

        def _wait_for_map_quiet(
            self,
            request,
            *,
            deadline_monotonic,
            required_quiet_s=None,
        ):
            del required_quiet_s
            self.hard_deadline = deadline_monotonic
            return self._evidence.snapshot()

    spec = replace(
        _unknown_spec(max_recovery_attempts=1),
        action_timeout_s=3.0,
        map_settle_s=0.5,
    )
    executor = _CapturingExecutor(runtime, _Manager(dry_run=False), evidence, spec)

    executor.run(_request())

    assert executor.hard_deadline == pytest.approx(13.0)


def test_attempt_exhaustion_completes_only_after_verified_recovery_budget_used():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        (
            "recovery_required:frontier_attempts_exhausted",
            "recovery_required:frontier_attempts_exhausted",
        ),
        recovery_map_sizes=(102, 104),
        recovery_backup_displacements=(0.24,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime,
        manager,
        evidence,
        _unknown_spec(max_recovery_attempts=1),
    )

    executor.run(_request())

    # 第一轮必须由 BackUp 的真实位移授权新 epoch；只有该恢复
    # 预算已用尽，第二轮仍尝试耗尽且地图无增益时才能收敛。
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    assert evidence.completion_reason == (
        "frontier_attempts_exhausted_below_material_gain"
    )
    assert len([call for call in runtime.calls if call[0] == "action"]) == 3
    assert len(
        [call for call in runtime.calls if call[0] == "recovery_backup"]
    ) == 1
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
        recovery_backup_displacements=(0.0,),
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


def test_blacklisted_recovery_with_low_gain_fails_without_new_epoch():
    outcome = "recovery_required:blacklisted_frontiers"
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
        match="below material map gain",
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
    assert "decision=fail_below_material_gain" in detail


def test_reachable_stall_decision_requires_bounded_confirmation_epochs():
    common = {
        "recovery_reason": (
            "recovery_required:reachable_frontiers_stalled"
        ),
        "map_gain_cells": 28,
        "minimum_gain_cells": 40,
        "map_gain_ratio": 28 / 25_746,
        "minimum_gain_ratio": 0.002,
        "available_frontier_count": 0,
        "active_goal_count": 0,
        # 现场允许残留不可达黑名单 frontier；门槛约束的是可达/活动账本。
        "blacklisted_frontier_count": 2,
        "detected_frontier_count": 13,
    }

    assert decide_epoch_recovery(
        **common,
        recovery_attempts_remaining=2,
        completed_recovery_epochs=0,
    ) is EpochRecoveryDecision.RUN_STALL_CONFIRMATION_EPOCH
    assert decide_epoch_recovery(
        **common,
        recovery_attempts_remaining=0,
        completed_recovery_epochs=2,
    ) is EpochRecoveryDecision.ASSESS_STALL_SATURATION


def test_repeated_reachable_stall_saturates_then_returns_saves_and_navigates():
    """现场假阴性回放：两轮 fresh 低收益后才允许进入原收口链。"""

    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 25_000)
    evidence.mark_scan_ready()
    events: list[str] = []

    class _RepeatedStallRuntime(_UnknownWorldRuntime):
        def __init__(self):
            super().__init__(
                evidence,
                (
                    "recovery_required:reachable_frontiers_stalled",
                    "recovery_required:reachable_frontiers_stalled",
                    "recovery_required:reachable_frontiers_stalled",
                ),
                recovery_map_sizes=(25_774, 25_790, 25_800),
            )
            self._epoch = 0
            self._odom_x = 0.0
            self.completion = None

        def wait_for_frontier(self, request, **kwargs):
            self._epoch += 1
            known_cells = (25_746, 25_784, 25_798)[self._epoch - 1]
            terminal_goals = (21, 3, 3)[self._epoch - 1]
            self.evidence.record_map([0] * known_cells)
            # 每轮约 7m；reset 只切断跨恢复段连线，不得清零任务级累计里程。
            for _ in range(21):
                self.evidence.record_odom(self._odom_x, 0.0)
                self._odom_x += 0.35
            self.evidence.record_frontier_telemetry(
                FrontierTelemetry(
                    status="exploration_in_progress",
                    detected_frontier_count=13,
                    available_frontier_count=0,
                    active_goal_count=0,
                    blacklisted_frontier_count=2,
                    accepted_goal_count=terminal_goals,
                    succeeded_goal_count=terminal_goals,
                )
            )
            return super().wait_for_frontier(request, **kwargs)

        def run_mapping_return_goal(self, request, **kwargs):
            events.append("return_home")
            return super().run_mapping_return_goal(request, **kwargs)

        def record_mapping_completion(self, **kwargs):
            events.append("record_completion")
            self.completion = kwargs

        def save_map(self, request):
            events.append("save_map")
            super().save_map(request)

        def start_navigation(self, request):
            events.append("start_navigation")
            super().start_navigation(request)

    manager = _Manager(dry_run=False)
    runtime = _RepeatedStallRuntime()
    spec = replace(
        _unknown_spec(max_recovery_attempts=2),
        saturation_policy=SaturationPolicy(required_map_quiet_s=0.001),
    )

    UnknownWorldMissionExecutor(runtime, manager, evidence, spec).run(
        _request()
    )

    assert evidence.completion_reason == (
        "reachable_frontiers_stalled_bounded_saturation"
    )
    assert manager.calls.count(
        ("start_explorer", Path("/tmp/frontier.yaml"))
    ) == 3
    assert manager.calls.count(("stop_explorer",)) == 3
    frontier_waits = [
        call for call in runtime.calls if call[0] == "wait_frontier"
    ]
    assert [call[1] for call in frontier_waits] == [2, 1, 0]
    # 三个 Explorer epoch 必须共享最初的绝对 deadline，不能因确认重启续期。
    assert len({call[2] for call in frontier_waits}) == 1
    assert events == [
        "return_home",
        "record_completion",
        "save_map",
        "start_navigation",
    ]
    assert runtime.completion is not None
    saturation = runtime.completion["saturation_evidence"]
    assessment = runtime.completion["saturation_assessment"]
    assert saturation.trigger is SaturationTrigger.REPEATED_REACHABLE_STALL
    assert saturation.history.counters.mapping_path_m >= 20.0
    assert len(saturation.history.epochs) == 3
    assert assessment.complete
    assert len(
        [call for call in runtime.calls if call[0] == "navigation_goal"]
    ) == 3


@pytest.mark.parametrize(
    "telemetry",
    [
        FrontierTelemetry(available_frontier_count=1),
        FrontierTelemetry(active_goal_count=1),
    ],
)
def test_attempt_exhaustion_low_gain_requires_no_available_or_active_frontier(
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
        recovery_backup_displacements=(0.0,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    with pytest.raises(RuntimeError, match="below material map gain"):
        executor.run(_request())

    assert evidence.completion_reason != (
        "frontier_attempts_exhausted_below_material_gain"
    )
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]


def test_epoch_recovery_decision_is_pure_and_budget_is_spent_only_on_advance():
    common = {
        "recovery_reason": "recovery_required:blacklisted_frontiers",
        "minimum_gain_cells": 40,
        "map_gain_ratio": 1.0,
        "minimum_gain_ratio": 0.002,
        "available_frontier_count": 0,
        "active_goal_count": 0,
        "blacklisted_frontier_count": 1,
        "detected_frontier_count": 1,
    }

    assert decide_epoch_recovery(
        **common,
        map_gain_cells=39,
        recovery_attempts_remaining=2,
    ) is EpochRecoveryDecision.FAIL_BELOW_MATERIAL_GAIN
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
    ) is EpochRecoveryDecision.FAIL_BELOW_MATERIAL_GAIN
    assert decide_epoch_recovery(
        **stalled,
        map_gain_cells=40,
        recovery_attempts_remaining=2,
    ) is EpochRecoveryDecision.ADVANCE_EPOCH


def test_final_epoch_treats_tiny_relative_map_gain_as_sensor_refinement():
    """现场回放：完整大图新增 41 cells 不应被绝对阈值误判成新区域。"""

    decision = decide_epoch_recovery(
        recovery_reason="recovery_required:frontier_attempts_exhausted",
        map_gain_cells=41,
        minimum_gain_cells=40,
        map_gain_ratio=41 / 26099,
        minimum_gain_ratio=0.002,
        recovery_attempts_remaining=0,
        available_frontier_count=0,
        active_goal_count=0,
        blacklisted_frontier_count=1,
        detected_frontier_count=8,
        completed_recovery_epochs=2,
    )

    assert decision is EpochRecoveryDecision.COMPLETE_BELOW_MATERIAL_GAIN


def test_final_epoch_material_gain_requires_one_post_scan_confirmation():
    """现场回放：53 cells 刚越线时不能失败，也不能未经 Explorer 直接完成。"""

    common = {
        "recovery_reason": "recovery_required:frontier_attempts_exhausted",
        "map_gain_cells": 53,
        "minimum_gain_cells": 40,
        "map_gain_ratio": 53 / 25460,
        "minimum_gain_ratio": 0.002,
        "recovery_attempts_remaining": 0,
        "available_frontier_count": 0,
        "active_goal_count": 0,
        "blacklisted_frontier_count": 0,
        "detected_frontier_count": 9,
        "completed_recovery_epochs": 2,
    }

    assert decide_epoch_recovery(
        **common,
        final_confirmation_used=False,
    ) is EpochRecoveryDecision.RUN_FINAL_CONFIRMATION_EPOCH
    assert decide_epoch_recovery(
        **common,
        final_confirmation_used=True,
    ) is EpochRecoveryDecision.FAIL_BUDGET_EXHAUSTED
    assert decide_epoch_recovery(
        **{**common, "completed_recovery_epochs": 0},
        final_confirmation_used=False,
    ) is EpochRecoveryDecision.FAIL_BUDGET_EXHAUSTED


def test_no_clearance_relocation_requires_verified_displacement_and_budget():
    common = {
        "recovery_reason": (
            "recovery_required:no_clearance_safe_frontier_approach"
        ),
        "map_gain_cells": 2,
        "minimum_gain_cells": 40,
        "map_gain_ratio": 0.02,
        "minimum_gain_ratio": 0.002,
        "available_frontier_count": 0,
        "active_goal_count": 0,
        "blacklisted_frontier_count": 0,
        "detected_frontier_count": 1,
        "minimum_recovery_displacement_m": 0.20,
    }

    assert decide_epoch_recovery(
        **common,
        recovery_attempts_remaining=1,
        recovery_displacement_m=0.20,
    ) is EpochRecoveryDecision.ADVANCE_EPOCH
    assert decide_epoch_recovery(
        **common,
        recovery_attempts_remaining=1,
        recovery_displacement_m=0.199,
    ) is EpochRecoveryDecision.FAIL_BELOW_MATERIAL_GAIN
    assert decide_epoch_recovery(
        **common,
        recovery_attempts_remaining=0,
        recovery_displacement_m=0.20,
    ) is EpochRecoveryDecision.FAIL_BUDGET_EXHAUSTED


def test_no_clearance_recovery_backs_up_before_scan_and_restarts_on_displacement():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        (
            "recovery_required:no_clearance_safe_frontier_approach",
            "no_reachable_frontiers",
        ),
        # 扫描地图增益不足，必须由独立里程计位移授权下一 epoch。
        recovery_map_sizes=(102,),
        recovery_backup_displacements=(0.24,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    executor.run(_request())

    names = [call[0] for call in runtime.calls]
    stop_index = names.index("stop_motion")
    backup_index = names.index("recovery_backup")
    recovery_scan_index = names.index("action", names.index("action") + 1)
    assert stop_index < backup_index < recovery_scan_index
    assert runtime.calls[backup_index] == (
        "recovery_backup",
        0.30,
        0.08,
        10.0,
    )
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    decision_detail = next(
        call[2]
        for call in runtime.calls
        if call[0] == "transition"
        and "frontier epoch recovery evaluated" in call[2]
    )
    assert "displacement=0.240" in decision_detail
    assert "decision=advance_epoch" in decision_detail


def test_attempt_exhaustion_with_budget_relocates_before_starting_next_epoch():
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
        # 原地扫描只增加 2 个栅格；只有 Nav2 BackUp 的真实位移可以授权新 epoch。
        recovery_map_sizes=(102,),
        recovery_backup_displacements=(0.24,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    executor.run(_request())

    names = [call[0] for call in runtime.calls]
    stop_index = names.index("stop_motion")
    backup_index = names.index("recovery_backup")
    recovery_scan_index = names.index("action", names.index("action") + 1)
    second_frontier_wait = names.index("wait_frontier", backup_index)
    assert stop_index < backup_index < recovery_scan_index < second_frontier_wait
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    assert evidence.completion_reason == "no_reachable_frontiers"


def test_attempt_exhaustion_cannot_complete_on_first_low_gain_recovery():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        ("recovery_required:frontier_attempts_exhausted",),
        recovery_map_sizes=(102,),
        recovery_backup_displacements=(0.19,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    with pytest.raises(RuntimeError, match="below material map gain"):
        executor.run(_request())

    assert evidence.completion_reason != (
        "frontier_attempts_exhausted_below_material_gain"
    )
    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    assert not any(call[0] == "navigation_goal" for call in runtime.calls)


def test_zero_recovery_budget_never_turns_attempt_exhaustion_into_completion():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)
    runtime = _UnknownWorldRuntime(
        evidence,
        ("recovery_required:frontier_attempts_exhausted",),
        recovery_map_sizes=(102,),
    )
    executor = UnknownWorldMissionExecutor(
        runtime,
        manager,
        evidence,
        _unknown_spec(max_recovery_attempts=0),
    )

    with pytest.raises(RuntimeError, match="below material map gain"):
        executor.run(_request())

    assert not any(call[0] == "recovery_backup" for call in runtime.calls)
    assert not any(call[0] == "navigation_goal" for call in runtime.calls)
    assert evidence.completion_reason != (
        "frontier_attempts_exhausted_below_material_gain"
    )


def test_no_clearance_backup_failure_never_scans_or_restarts_explorer():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)

    class _FailingBackupRuntime(_UnknownWorldRuntime):
        def run_recovery_backup(self, _request, **_kwargs):
            self.calls.append(("recovery_backup_failed",))
            raise RuntimeError("backup collision ahead")

    runtime = _FailingBackupRuntime(
        evidence,
        ("recovery_required:no_clearance_safe_frontier_approach",),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    with pytest.raises(RuntimeError, match="backup collision ahead"):
        executor.run(_request())

    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    # 初始扫描之外，故障后不得转圈，更不得通过重启清空 attempt memory。
    assert len([call for call in runtime.calls if call[0] == "action"]) == 1


def test_cancellation_during_recovery_backup_never_scans_or_restarts():
    evidence = MappingEvidenceTracker(1)
    evidence.record_map([0] * 100)
    evidence.mark_scan_ready()
    manager = _Manager(dry_run=False)

    class _CanceledBackupRuntime(_UnknownWorldRuntime):
        def run_recovery_backup(self, request, **_kwargs):
            self.calls.append(("recovery_backup_canceled",))
            request.canceled = True
            return 0.30

    runtime = _CanceledBackupRuntime(
        evidence,
        ("recovery_required:no_clearance_safe_frontier_approach",),
    )
    executor = UnknownWorldMissionExecutor(
        runtime, manager, evidence, _unknown_spec()
    )

    with pytest.raises(AutomaticMissionCancelled, match="recovery backup"):
        executor.run(_request())

    assert manager.calls == [
        ("start_explorer", Path("/tmp/frontier.yaml")),
        ("stop_explorer",),
    ]
    assert len([call for call in runtime.calls if call[0] == "action"]) == 1


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
        (
            "minimum_epoch_map_gain_ratio",
            0.0,
            "minimum epoch map gain ratio must be in",
        ),
        (
            "minimum_epoch_map_gain_ratio",
            float("nan"),
            "minimum epoch map gain ratio must be in",
        ),
        ("map_settle_s", -0.1, "map settle time must be finite"),
        ("map_settle_s", float("inf"), "map settle time must be finite"),
        (
            "final_confirmation_timeout_s",
            0.0,
            "unknown-world mission timeouts must be positive",
        ),
        (
            "final_confirmation_timeout_s",
            float("nan"),
            "unknown-world mission timeouts must be positive",
        ),
        (
            "exploration_timeout_s",
            float("nan"),
            "unknown-world mission timeouts must be positive",
        ),
    ],
)
def test_unknown_world_spec_rejects_invalid_convergence_values(
    field,
    value,
    message,
):
    with pytest.raises(ValueError, match=message):
        replace(_unknown_spec(), **{field: value})


@pytest.mark.parametrize(
    ("field", "value", "message"),
    [
        ("distance_m", 0.0, "backup distance"),
        ("distance_m", 0.51, "backup distance"),
        ("speed_mps", 0.0, "backup speed"),
        ("speed_mps", 0.21, "backup speed"),
        ("timeout_s", 4.0, "travel time and margin"),
        ("minimum_displacement_m", 0.31, "minimum recovery displacement"),
        ("distance_m", float("nan"), "must be finite"),
    ],
)
def test_recovery_backup_spec_rejects_unsafe_values(field, value, message):
    backup = _unknown_spec().recovery_backup

    with pytest.raises(ValueError, match=message):
        replace(backup, **{field: value})
