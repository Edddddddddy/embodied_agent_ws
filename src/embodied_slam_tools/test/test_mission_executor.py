"""Automatic mission transaction tests without ROS runtime."""

from dataclasses import replace
from pathlib import Path

import pytest

from embodied_slam_tools.mapping_evidence import MappingEvidenceTracker
from embodied_slam_tools.mission_executor import (
    AutomaticMissionCancelled,
    AutomaticMissionExecutor,
    AutomaticMissionSpec,
    CommandRequest,
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
