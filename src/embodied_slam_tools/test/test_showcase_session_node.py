"""Focused tests for ROS-facing showcase orchestrator helpers."""

import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from embodied_agent_interfaces.msg import RobotCommand, WakeEvent
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, NavigateToPose
import pytest
from std_msgs.msg import String

import embodied_slam_tools.showcase_session_node as showcase_session_node_module
from embodied_slam_tools.mission_executor import (
    AutomaticMissionCancelled,
    CommandRequest,
    wait_for_required_event,
)
from embodied_slam_tools.showcase_session_node import (
    SessionOrchestratorNode,
    _GoalCandidateRejected,
    _admit_goal_candidates,
    _frontier_evidence_message,
    _frontier_telemetry_from_message,
    _get_lifecycle_state,
    _navigation_goal_evidence_message,
    _occupancy_snapshot_from_message,
    _path_points_for_goal,
)
from embodied_slam_tools.showcase_session import (
    SessionCommand,
    SessionPhase,
    ShowcaseSessionStateMachine,
)
from embodied_slam_tools.mapping_evidence import (
    FrontierTelemetry,
    MappingEvidenceTracker,
    NavigationGoalEvidence,
    NavigationGoalStatus,
)
from embodied_slam_tools.mapping_return import (
    PlanarPose,
    ReturnActionKind,
    ReturnActionStatus,
    ReturnToStartEvidence,
    ReturnToStartSpec,
    evaluate_return_to_start,
)


def _session_command_receiver(input_source="raw_asr"):
    queued = []
    logger = SimpleNamespace(
        info=lambda _message: None,
        warning=lambda _message: None,
    )
    receiver = SimpleNamespace(
        _command_input_source=input_source,
        _active_request=None,
        _intent_lock=threading.Lock(),
        _last_intent=(None, 0.0),
        _enqueue=lambda request: queued.append(request) or True,
        get_logger=lambda: logger,
    )
    return receiver, queued


def test_wake_event_mode_does_not_accept_ungated_raw_asr():
    receiver, queued = _session_command_receiver("wake_event")

    SessionOrchestratorNode._on_asr_final(
        receiver, String(data="开始自动建图")
    )

    assert queued == []


def test_raw_asr_remains_the_compatible_default_command_source():
    receiver, queued = _session_command_receiver()
    del receiver._command_input_source

    SessionOrchestratorNode._on_asr_final(
        receiver, String(data="开始自动建图")
    )

    assert [request.command for request in queued] == [
        SessionCommand.RUN_AUTOMATIC_MISSION
    ]


def test_wake_event_mode_accepts_known_command_after_wake():
    receiver, queued = _session_command_receiver("wake_event")
    event = WakeEvent()
    event.kind = WakeEvent.KIND_WAKE
    event.command_known = True
    event.command = "开始自动建图"

    SessionOrchestratorNode._on_wake_event(receiver, event)

    assert [request.command for request in queued] == [
        SessionCommand.RUN_AUTOMATIC_MISSION
    ]


def test_wake_event_mode_accepts_known_command_in_active_session():
    receiver, queued = _session_command_receiver("wake_event")
    event = WakeEvent()
    event.kind = WakeEvent.KIND_CONTINUE
    event.command_known = True
    event.command = "开始自动建图"

    SessionOrchestratorNode._on_wake_event(receiver, event)

    assert [request.command for request in queued] == [
        SessionCommand.RUN_AUTOMATIC_MISSION
    ]


@pytest.mark.parametrize(
    ("kind", "command_known", "command"),
    [
        (WakeEvent.KIND_REJECTED, True, "开始自动建图"),
        (WakeEvent.KIND_SLEEP, True, "开始自动建图"),
        (WakeEvent.KIND_WAKE, False, ""),
        (WakeEvent.KIND_CONTINUE, True, "   "),
    ],
)
def test_wake_event_mode_ignores_events_without_accepted_command(
    kind, command_known, command
):
    receiver, queued = _session_command_receiver("wake_event")
    event = WakeEvent()
    event.kind = kind
    event.command_known = command_known
    event.command = command

    SessionOrchestratorNode._on_wake_event(receiver, event)

    assert queued == []


class _LifecycleClient:
    def __init__(self, response):
        self.response = response
        self.calls = []

    def call(self, request, timeout_sec=None):
        self.calls.append((request, timeout_sec))
        return self.response


def test_get_lifecycle_state_uses_bounded_synchronous_call():
    response = SimpleNamespace(
        current_state=SimpleNamespace(id=State.PRIMARY_STATE_ACTIVE)
    )
    client = _LifecycleClient(response)

    state = _get_lifecycle_state(client, 0.5)

    assert state == State.PRIMARY_STATE_ACTIVE
    assert len(client.calls) == 1
    request, timeout_s = client.calls[0]
    assert isinstance(request, GetState.Request)
    assert timeout_s == 0.5


def test_get_lifecycle_state_preserves_timeout_as_missing_state():
    client = _LifecycleClient(None)

    assert _get_lifecycle_state(client, 0.2) is None


def test_wait_for_required_event_accepts_ready_dependency():
    event = threading.Event()
    event.set()

    assert wait_for_required_event(event, 0.1, lambda: False)


def test_wait_for_required_event_reports_timeout():
    assert not wait_for_required_event(
        threading.Event(), 0.001, lambda: False, poll_s=0.001
    )


def test_wait_for_required_event_honors_cancel_before_readiness():
    with pytest.raises(AutomaticMissionCancelled):
        wait_for_required_event(threading.Event(), 1.0, lambda: True)


def test_explore_status_adapter_preserves_frontier_goal_lifecycle():
    telemetry = _frontier_telemetry_from_message(
        SimpleNamespace(
            status="exploration_blocked",
            detected_frontier_count=5,
            available_frontier_count=0,
            blacklisted_frontier_count=5,
            active_goal_count=0,
            active_goal_id="",
            accepted_goal_count=4,
            succeeded_goal_count=1,
            aborted_goal_count=1,
            canceled_goal_count=1,
            rejected_goal_count=1,
            last_goal_terminal="rejected",
            completion_reason="all_frontiers_blacklisted",
        )
    )

    assert telemetry.status == "exploration_blocked"
    assert telemetry.detected_frontier_count == 5
    assert telemetry.blacklisted_frontier_count == 5
    assert telemetry.accepted_goal_count == 4
    assert telemetry.rejected_goal_count == 1
    assert telemetry.completion_reason == "all_frontiers_blacklisted"


def test_explore_status_adapter_keeps_old_message_compatible():
    telemetry = _frontier_telemetry_from_message(
        SimpleNamespace(status="exploration_started")
    )

    assert telemetry.status == "exploration_started"
    assert telemetry.detected_frontier_count == 0


def test_occupancy_adapter_preserves_ros_cell_order_and_origin():
    message = SimpleNamespace(
        info=SimpleNamespace(
            width=3,
            height=2,
            resolution=0.5,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-1.0, y=-2.0),
                orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            ),
        ),
        data=[0, 100, -1, 0, 0, 100],
    )

    snapshot = _occupancy_snapshot_from_message(message)

    assert snapshot.width == 3
    assert snapshot.height == 2
    assert snapshot.origin_xy == (-1.0, -2.0)
    assert snapshot.cells == (0, 100, -1, 0, 0, 100)


def test_frontier_evidence_keeps_provider_and_mission_reasons_separate():
    message = _frontier_evidence_message(
        FrontierTelemetry(
            status="exploration_complete",
            completion_reason="no_frontiers",
            accepted_goal_count=2,
            succeeded_goal_count=2,
        ),
        mission_completion_reason="no_reachable_frontiers",
    )

    assert message.valid is True
    assert message.provider_completion_reason == "no_frontiers"
    assert message.mission_completion_reason == "no_reachable_frontiers"


def test_frontier_quiesce_publishes_pause_and_requires_drained_ledger():
    evidence = MappingEvidenceTracker(1)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_paused",
            active_goal_count=0,
            accepted_goal_count=3,
            succeeded_goal_count=2,
            canceled_goal_count=1,
        )
    )
    published = []
    fake = SimpleNamespace(
        _dry_run=False,
        _mapping_evidence=evidence,
        _explore_control_pub=SimpleNamespace(
            publish=lambda message: published.append(message.data)
        ),
    )

    telemetry = SessionOrchestratorNode.quiesce_frontier(
        fake,
        CommandRequest(SessionCommand.RUN_AUTOMATIC_MISSION, "test"),
        timeout_s=0.01,
    )

    assert published == [False]
    assert telemetry.active_goal_count == 0
    assert telemetry.accepted_goal_count == 3


def test_frontier_quiesce_fails_closed_when_owner_goal_is_still_active():
    evidence = MappingEvidenceTracker(1)
    evidence.record_frontier_telemetry(
        FrontierTelemetry(
            status="exploration_running",
            active_goal_count=1,
            accepted_goal_count=1,
        )
    )
    fake = SimpleNamespace(
        _dry_run=False,
        _mapping_evidence=evidence,
        _explore_control_pub=SimpleNamespace(publish=lambda _message: None),
    )

    with pytest.raises(TimeoutError, match="did not drain Action ledger"):
        SessionOrchestratorNode.quiesce_frontier(
            fake,
            CommandRequest(SessionCommand.RUN_AUTOMATIC_MISSION, "test"),
            timeout_s=0.0,
        )


def test_new_mission_clears_previous_mapping_completion_evidence():
    fake = SimpleNamespace(
        _mapping_saturation_evidence=object(),
        _mapping_saturation_assessment=object(),
        _return_to_start_evidence=object(),
    )

    SessionOrchestratorNode._reset_mapping_completion_evidence(fake)

    assert fake._mapping_saturation_evidence is None
    assert fake._mapping_saturation_assessment is None
    assert fake._return_to_start_evidence is None


def test_navigation_goal_evidence_is_a_compact_typed_snapshot():
    message = _navigation_goal_evidence_message(
        NavigationGoalEvidence(
            sequence=3,
            goal_xy=(1.25, -0.75),
            status=NavigationGoalStatus.SUCCEEDED,
            started_at_ns=1_500_000_000,
            finished_at_ns=2_000_000_000,
            nav2_status=4,
            plan_count=2,
        )
    )

    assert message.sequence == 3
    assert message.goal.header.frame_id == "map"
    assert message.goal.pose.position.x == pytest.approx(1.25)
    assert message.status == message.STATUS_SUCCEEDED
    assert message.started_at.sec == 1
    assert message.started_at.nanosec == 500_000_000
    assert message.all_plans_known_free is True


def test_completed_phase_and_success_outcome_are_published_atomically():
    observed_during_transition = []
    fake = SimpleNamespace(
        _state_lock=threading.RLock(),
        _mission_outcome=0,
        _mission_message="",
    )
    fake._fsm = SimpleNamespace(
        transition=lambda *_args, **_kwargs: observed_during_transition.append(
            fake._mission_outcome
        )
    )
    fake._publish_state = lambda: None

    SessionOrchestratorNode.transition(
        fake,
        SessionPhase.MISSION_COMPLETED,
        detail="mission completed",
    )

    assert observed_during_transition == [2]
    assert fake._mission_outcome == 2
    assert fake._mission_message == "automatic mission completed"


def test_navigation_switch_clears_mapping_inputs_before_stage_start():
    events = []
    fake = SimpleNamespace(
        _fsm=SimpleNamespace(
            snapshot=SimpleNamespace(phase=SessionPhase.MAP_SAVED)
        ),
        _map_pose_condition=threading.Condition(),
        _navigation_input_generation=4,
        _latest_occupancy=object(),
        _latest_map_pose_xy=(1.0, 2.0),
        _latest_occupancy_generation=4,
        _latest_map_pose_generation=4,
        transition=lambda phase, **_kwargs: events.append(
            ("transition", phase)
        ),
        feedback=lambda _request, _progress: None,
        _readiness_generation=lambda: 9,
        _wait_for_new_ready=lambda generation: events.append(
            ("ready", generation)
        ),
    )

    class _Manager:
        def stop(self):
            events.append(("stop",))

        def start(self, stage):
            events.append(
                (
                    "start",
                    stage,
                    fake._navigation_input_generation,
                    fake._latest_occupancy,
                    fake._latest_map_pose_xy,
                )
            )

    fake._manager = _Manager()

    SessionOrchestratorNode.start_navigation(
        fake, SimpleNamespace(message="")
    )

    stop_index = events.index(("stop",))
    start_event = next(item for item in events if item[0] == "start")
    start_index = events.index(start_event)
    assert stop_index < start_index
    assert start_event == ("start", "navigation", 5, None, None)
    assert fake._latest_occupancy_generation == -1
    assert fake._latest_map_pose_generation == -1


def _automatic_navigation_failure_fixture(manager, executor_callback):
    """构造覆盖 request worker -> stage switch 的最小真实状态机夹具。"""

    fsm = ShowcaseSessionStateMachine()
    fsm.transition(SessionPhase.MAPPING, detail="mapping ready")
    transitions = []
    fake = SimpleNamespace(
        _fsm=fsm,
        _state_lock=threading.RLock(),
        _operation_active=threading.Event(),
        _active_request=None,
        _mission_sequence=0,
        _mission_outcome=0,
        _mission_message="",
        _navigation_goal_ledger=SimpleNamespace(reset=lambda _sequence: None),
        _publish_state=lambda: None,
        _manager=manager,
        _map_pose_condition=threading.Condition(),
        _navigation_input_generation=0,
        _latest_occupancy=object(),
        _latest_map_pose_xy=(0.0, 0.0),
        _latest_occupancy_generation=0,
        _latest_map_pose_generation=0,
        _readiness_generation=lambda: 0,
        _wait_for_new_ready=lambda _generation: None,
        feedback=lambda _request, _progress: None,
        get_logger=lambda: SimpleNamespace(
            error=lambda _message: None,
            warning=lambda _message: None,
        ),
    )

    def transition(phase, *, detail, **kwargs):
        transitions.append(phase)
        return fsm.transition(phase, detail=detail, **kwargs)

    fake.transition = transition
    fake._cleanup_failed_navigation_startup = lambda error: (
        SessionOrchestratorNode._cleanup_failed_navigation_startup(fake, error)
    )
    fake._wait_navigation_ready_impl = lambda request: (
        SessionOrchestratorNode._wait_navigation_ready_impl(fake, request)
    )
    fake._automatic_mission_executor = SimpleNamespace(
        run=lambda request: executor_callback(fake, request)
    )
    return fake, transitions


def test_navigation_stage_start_failure_stops_partial_stage_and_fails_request():
    class _Manager:
        stage = "mapping"
        stop_calls = 0

        def stop(self):
            self.stop_calls += 1
            self.stage = ""

        def start(self, stage):
            assert stage == "navigation"
            raise RuntimeError("navigation launch failed")

    manager = _Manager()
    fake, transitions = _automatic_navigation_failure_fixture(
        manager,
        lambda node, request: SessionOrchestratorNode.start_navigation(
            node, request
        ),
    )
    request = CommandRequest(
        command=SessionCommand.RUN_AUTOMATIC_MISSION,
        source="test",
    )

    SessionOrchestratorNode._execute_request(fake, request)

    # 第一次 stop 切换 mapping，第二次 stop 回收可能已部分拉起的 navigation。
    assert manager.stop_calls == 2
    assert fake._fsm.snapshot.phase == SessionPhase.FAILED
    assert transitions[-1] == SessionPhase.FAILED
    assert request.message == "navigation launch failed"
    assert request.success is False


def test_navigation_readiness_timeout_stops_partial_stage_and_fails_request():
    class _Manager:
        stage = "navigation"
        stop_calls = 0

        def stop(self):
            self.stop_calls += 1
            self.stage = ""

    manager = _Manager()

    def wait_until_timeout(node, request):
        node.transition(
            SessionPhase.STARTING_NAVIGATION,
            detail="navigation process started",
        )
        SessionOrchestratorNode.wait_navigation_ready(node, request)

    fake, transitions = _automatic_navigation_failure_fixture(
        manager, wait_until_timeout
    )
    fake._dry_run = False
    fake._startup_timeout_s = 0.0
    unavailable = SimpleNamespace(
        server_is_ready=lambda: False,
        wait_for_server=lambda timeout_sec: False,
    )
    fake._navigate_to_pose_client = unavailable
    fake._follow_waypoints_client = unavailable
    fake._cancel_automatic_motion = lambda: None
    request = CommandRequest(
        command=SessionCommand.RUN_AUTOMATIC_MISSION,
        source="test",
    )

    SessionOrchestratorNode._execute_request(fake, request)

    assert manager.stop_calls == 1
    assert fake._fsm.snapshot.phase == SessionPhase.FAILED
    assert transitions[-1] == SessionPhase.FAILED
    assert "Nav2 Action server unavailable" in request.message


def test_navigation_start_cleanup_failure_does_not_mask_primary_exception():
    primary = RuntimeError("navigation launch failed")
    cleanup = RuntimeError("navigation cleanup failed")

    class _Manager:
        stop_calls = 0

        def stop(self):
            self.stop_calls += 1
            if self.stop_calls == 2:
                raise cleanup

        def start(self, _stage):
            raise primary

    manager = _Manager()
    fake, _ = _automatic_navigation_failure_fixture(
        manager, lambda _node, _request: None
    )

    with pytest.raises(RuntimeError) as raised:
        SessionOrchestratorNode.start_navigation(
            fake, SimpleNamespace(message="")
        )

    assert raised.value is primary
    assert manager.stop_calls == 2
    assert any(
        "navigation cleanup failed" in note
        for note in getattr(raised.value, "__notes__", ())
    )


def test_navigation_inputs_are_tagged_with_current_generation():
    map_records = []
    fake = SimpleNamespace(
        _mapping_evidence=SimpleNamespace(
            record_map=lambda cells: map_records.append(tuple(cells))
        ),
        _map_pose_condition=threading.Condition(),
        _navigation_input_generation=7,
        _latest_occupancy=None,
        _latest_map_pose_xy=None,
        _latest_occupancy_generation=-1,
        _latest_map_pose_generation=-1,
        get_logger=lambda: SimpleNamespace(error=lambda _message: None),
    )
    map_message = SimpleNamespace(
        info=SimpleNamespace(
            width=2,
            height=2,
            resolution=1.0,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=0.0, y=0.0),
                orientation=SimpleNamespace(
                    x=0.0, y=0.0, z=0.0, w=1.0
                ),
            ),
        ),
        data=[0, 0, 0, 0],
    )
    pose_message = SimpleNamespace(
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=SimpleNamespace(x=1.25, y=-0.75)
            )
        )
    )

    SessionOrchestratorNode._on_map(fake, map_message)
    SessionOrchestratorNode._on_amcl_pose(fake, pose_message)

    assert map_records == [(0, 0, 0, 0)]
    assert fake._latest_occupancy_generation == 7
    assert fake._latest_map_pose_generation == 7
    assert fake._latest_map_pose_xy == (1.25, -0.75)


def _path(frame_id, points):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=frame_id),
        poses=[
            SimpleNamespace(
                pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y))
            )
            for x, y in points
        ],
    )


class _DoneFuture:
    def __init__(self, value):
        self._value = value

    def done(self):
        return True

    def result(self):
        return self._value


class _RaisingResultFuture:
    @staticmethod
    def done():
        return True

    @staticmethod
    def result():
        raise RuntimeError("synthetic result read failure")


class _CancelableNavigationResult:
    def __init__(self, *, polls_after_cancel=3):
        self.terminal = False
        self.cancel_requested = False
        self.polls_after_cancel = polls_after_cancel
        self.cancel_poll_count = 0

    def done(self):
        if self.cancel_requested and not self.terminal:
            self.cancel_poll_count += 1
            if self.cancel_poll_count >= self.polls_after_cancel:
                self.terminal = True
        return self.terminal

    def result(self):
        return SimpleNamespace(
            status=GoalStatus.STATUS_CANCELED,
            result=SimpleNamespace(error_code=0),
        )


class _AcceptedNavigationHandle:
    def __init__(self, result_future):
        self.accepted = True
        self.result_future = result_future
        self.cancel_count = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        self.result_future.cancel_requested = True
        return _DoneFuture(SimpleNamespace())


class _AcceptedHandleWithoutResultFuture:
    def __init__(self):
        self.accepted = True
        self.cancel_count = 0

    @staticmethod
    def get_result_async():
        raise RuntimeError("synthetic get_result_async failure")

    def cancel_goal_async(self):
        self.cancel_count += 1
        return _DoneFuture(SimpleNamespace())


class _DelayedGoalResponse:
    def __init__(self, handle, *, pending_polls=2):
        self.handle = handle
        self.pending_polls = pending_polls
        self.poll_count = 0

    def done(self):
        self.poll_count += 1
        return self.poll_count > self.pending_polls

    def result(self):
        return self.handle


class _NeverGoalResponse:
    @staticmethod
    def done():
        return False


def test_nav2_backup_goal_uses_positive_local_magnitudes_and_strong_success():
    captured = []
    result = BackUp.Result()
    result.error_code = BackUp.Result.NONE
    handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: _DoneFuture(
            SimpleNamespace(
                status=GoalStatus.STATUS_SUCCEEDED,
                result=result,
            )
        ),
    )
    fake = SimpleNamespace(
        _backup_client=SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=lambda goal: captured.append(goal)
            or _DoneFuture(handle),
        )
    )

    SessionOrchestratorNode._execute_nav2_backup(
        fake,
        SimpleNamespace(canceled=False),
        distance_m=0.30,
        speed_mps=0.08,
        time_allowance_s=10.0,
        deadline_monotonic=10**12,
    )

    assert len(captured) == 1
    goal = captured[0]
    # Nav2 BackUp 的客户端契约使用正幅值，Behavior Server 内部负责负向运动。
    assert goal.target.x == pytest.approx(0.30)
    assert goal.target.y == 0.0
    assert goal.target.z == 0.0
    assert goal.speed == pytest.approx(0.08)
    assert goal.time_allowance.sec == 10
    assert goal.time_allowance.nanosec == 0


def test_nav2_backup_collision_is_failure_and_forces_typed_stop():
    result = BackUp.Result()
    result.error_code = BackUp.Result.COLLISION_AHEAD
    result.error_msg = "collision ahead"
    handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: _DoneFuture(
            SimpleNamespace(
                status=GoalStatus.STATUS_ABORTED,
                result=result,
            )
        ),
    )
    stop_calls = []
    fake = SimpleNamespace(
        _backup_client=SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=lambda _goal: _DoneFuture(handle),
        ),
        stop_motion_and_wait=lambda request, *, timeout_s: stop_calls.append(
            (request, timeout_s)
        ),
    )

    with pytest.raises(RuntimeError, match="collision ahead"):
        SessionOrchestratorNode._execute_nav2_backup(
            fake,
            SimpleNamespace(canceled=False),
            distance_m=0.30,
            speed_mps=0.08,
            time_allowance_s=10.0,
            deadline_monotonic=10**12,
        )

    assert len(stop_calls) == 1
    assert stop_calls[0][0].source == "navigation_safety_stop"
    assert stop_calls[0][0].canceled is False


def test_recovery_backup_returns_independent_odom_displacement():
    from embodied_slam_tools.mapping_evidence import MappingEvidenceTracker

    evidence = MappingEvidenceTracker(1)
    evidence.record_odom(1.0, 2.0)

    class _Runtime:
        _dry_run = False
        _mapping_evidence = evidence

        def _wait_for_recovery_odom(self, *args, **kwargs):
            return SessionOrchestratorNode._wait_for_recovery_odom(
                self, *args, **kwargs
            )

        @staticmethod
        def _execute_nav2_backup(*_args, **_kwargs):
            evidence.record_odom(0.76, 2.0)

        @staticmethod
        def get_logger():
            return SimpleNamespace(info=lambda _message: None)

    displacement = SessionOrchestratorNode.run_recovery_backup(
        _Runtime(),
        SimpleNamespace(canceled=False),
        distance_m=0.30,
        speed_mps=0.08,
        timeout_s=10.0,
    )

    assert displacement == pytest.approx(0.24)
    assert evidence.mapping_path_m == 0.0


def _navigation_execution_fake(*, unsafe_runtime_path):
    result_future = _CancelableNavigationResult()
    handle = _AcceptedNavigationHandle(result_future)
    stop_calls = []
    status_calls = []
    stage_shutdowns = []

    class _Fake:
        _navigate_to_pose_client = SimpleNamespace(
            wait_for_server=lambda timeout_sec: True,
            send_goal_async=lambda _goal: _DoneFuture(handle),
        )
        _navigation_goal_ledger = SimpleNamespace(
            get=lambda _sequence: SimpleNamespace(
                plan_count=1,
                all_plans_known_free=not unsafe_runtime_path,
            )
        )

        def _set_navigation_goal_status(self, sequence, status, **kwargs):
            status_calls.append((sequence, status, kwargs))

        def _cancel_nav2_goal(self, goal_handle, future, wait_s=3.0):
            return SessionOrchestratorNode._cancel_nav2_goal(
                goal_handle,
                future,
                wait_s=min(wait_s, 0.01),
            )

        def _cancel_nav2_goal_and_force_stop(self, *args, **kwargs):
            return SessionOrchestratorNode._cancel_nav2_goal_and_force_stop(
                self, *args, **kwargs
            )

        def _resolve_pending_nav2_goal_safely(self, *args, **kwargs):
            # 测试预算刻意缩短；生产默认仍使用独立 10s 安全预算。
            kwargs.setdefault("timeout_s", 0.001)
            return SessionOrchestratorNode._resolve_pending_nav2_goal_safely(
                self, *args, **kwargs
            )

        def stop_motion_and_wait(self, request, *, timeout_s):
            stop_calls.append((request, timeout_s, result_future.done()))

        _manager = SimpleNamespace(
            stop=lambda: stage_shutdowns.append("stop")
        )

        @staticmethod
        def _cancel_automatic_motion():
            raise AssertionError("fault cleanup must use the typed forced stop")

    return (
        _Fake(),
        handle,
        result_future,
        stop_calls,
        status_calls,
        stage_shutdowns,
    )


def test_nav2_succeeded_wrapper_with_result_error_is_typed_failure(monkeypatch):
    """Action 协议成功不能覆盖 NavigateToPose 业务错误。"""

    monkeypatch.setattr(showcase_session_node_module.time, "sleep", lambda _s: None)
    calls = []
    terminal = set()

    def set_status(sequence, status, **kwargs):
        calls.append((sequence, status, kwargs))
        if status in {
            NavigationGoalStatus.SUCCEEDED,
            NavigationGoalStatus.REJECTED,
            NavigationGoalStatus.ABORTED,
            NavigationGoalStatus.CANCELED,
            NavigationGoalStatus.TIMED_OUT,
        }:
            terminal.add(sequence)

    def set_if_open(sequence, status, **kwargs):
        if sequence not in terminal:
            set_status(sequence, status, **kwargs)

    error_code = NavigateToPose.Result.NONE + 42
    fake = SimpleNamespace(
        _dry_run=False,
        _navigation_goal_ledger=SimpleNamespace(
            get=lambda _sequence: SimpleNamespace(
                plan_count=2,
                all_plans_known_free=True,
            )
        ),
        _request_preflight_path=lambda *_args, **_kwargs: object(),
        _record_navigation_plan=lambda _sequence, _path: True,
        _execute_sampled_nav2_goal=lambda *_args, **_kwargs: SimpleNamespace(
            status=GoalStatus.STATUS_SUCCEEDED,
            result=SimpleNamespace(error_code=error_code),
        ),
        _set_navigation_goal_status=set_status,
        _set_navigation_goal_status_if_open=set_if_open,
    )

    with pytest.raises(RuntimeError, match=f"error={error_code}"):
        SessionOrchestratorNode.run_navigation_goal(
            fake,
            SimpleNamespace(canceled=False),
            sequence=1,
            goal_xy=(1.0, 2.0),
            timeout_s=1.0,
        )

    final = calls[-1]
    assert final[1] == NavigationGoalStatus.ABORTED
    assert final[2]["nav2_status"] == GoalStatus.STATUS_SUCCEEDED
    assert final[2]["nav2_error_code"] == error_code


@pytest.mark.parametrize(
    ("unsafe_runtime_path", "deadline", "error_type", "message"),
    [
        (True, 10**12, RuntimeError, "unsafe runtime navigation plan"),
        (False, 0.0, TimeoutError, "navigation goal result timeout"),
    ],
)
def test_runtime_navigation_fault_cancels_forces_stop_and_waits_terminal(
    unsafe_runtime_path,
    deadline,
    error_type,
    message,
):
    fake, handle, result_future, stop_calls, _statuses, stage_shutdowns = (
        _navigation_execution_fake(
            unsafe_runtime_path=unsafe_runtime_path,
        )
    )
    request = SimpleNamespace(canceled=False)

    with pytest.raises(error_type, match=message):
        SessionOrchestratorNode._execute_sampled_nav2_goal(
            fake,
            request,
            sequence=1,
            goal_xy=(1.0, 2.0),
            deadline=deadline,
        )

    assert handle.cancel_count == 1
    assert result_future.done() is True
    assert len(stop_calls) == 1
    # 安全 STOP 必须使用未被上层取消的内部 request，否则 gateway 会直接拒绝。
    assert stop_calls[0][0] is not request
    assert stop_calls[0][0].canceled is False
    assert stop_calls[0][0].source == "navigation_safety_stop"
    assert stop_calls[0][1] > 0.0
    # typed STOP 可以先于 Nav2 terminal 返回；helper 必须继续等待迟到终态。
    assert stop_calls[0][2] is False
    assert result_future.cancel_poll_count >= 3
    assert stage_shutdowns == []


def test_accepted_goal_cancel_uses_same_stop_and_terminal_transaction(
    monkeypatch,
):
    """用户取消已 accepted goal 时，也不能绕过 typed STOP 与终态等待。"""

    monkeypatch.setattr(showcase_session_node_module.time, "sleep", lambda _s: None)
    fake, handle, result_future, stop_calls, _statuses, stage_shutdowns = (
        _navigation_execution_fake(unsafe_runtime_path=False)
    )
    request = SimpleNamespace(
        canceled=True,
        command=SessionCommand.RUN_AUTOMATIC_MISSION,
    )

    with pytest.raises(AutomaticMissionCancelled):
        SessionOrchestratorNode._execute_sampled_nav2_goal(
            fake,
            request,
            sequence=1,
            goal_xy=(1.0, 2.0),
            deadline=10**12,
        )

    assert handle.cancel_count == 1
    assert result_future.done() is True
    assert len(stop_calls) == 1
    assert stop_calls[0][0] is not request
    assert stop_calls[0][0].canceled is False
    assert stage_shutdowns == []


@pytest.mark.parametrize(
    ("request_canceled", "deadline", "expected_error"),
    [
        (True, 10**12, AutomaticMissionCancelled),
        (False, 0.0, TimeoutError),
    ],
)
def test_pending_late_accepted_goal_waits_and_closes_transaction(
    monkeypatch,
    request_canceled,
    deadline,
    expected_error,
):
    """取消或超时遇到迟到 accepted 时，都必须等停车事实后再释放锁。"""

    monkeypatch.setattr(showcase_session_node_module.time, "sleep", lambda _s: None)
    fake, handle, result_future, stop_calls, _statuses, stage_shutdowns = (
        _navigation_execution_fake(unsafe_runtime_path=False)
    )
    response = _DelayedGoalResponse(handle, pending_polls=2)
    fake._navigate_to_pose_client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: response,
    )
    request = SimpleNamespace(
        canceled=request_canceled,
        command=SessionCommand.RUN_AUTOMATIC_MISSION,
    )

    with pytest.raises(expected_error):
        SessionOrchestratorNode._execute_sampled_nav2_goal(
            fake,
            request,
            sequence=1,
            goal_xy=(1.0, 2.0),
            deadline=deadline,
        )

    assert response.poll_count >= 3
    assert handle.cancel_count == 1
    assert result_future.done() is True
    assert len(stop_calls) == 1
    assert stop_calls[0][0] is not request
    assert stop_calls[0][0].canceled is False
    assert stage_shutdowns == []


def test_pending_goal_response_timeout_stops_navigation_stage(monkeypatch):
    """response 永不到达时只能停止 stage，不能假装 cleanup 已完成。"""

    monkeypatch.setattr(showcase_session_node_module.time, "sleep", lambda _s: None)
    fake, _handle, _result, stop_calls, _statuses, stage_shutdowns = (
        _navigation_execution_fake(unsafe_runtime_path=False)
    )
    fake._navigate_to_pose_client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: _NeverGoalResponse(),
    )
    request = SimpleNamespace(
        canceled=True,
        command=SessionCommand.RUN_AUTOMATIC_MISSION,
    )

    with pytest.raises(
        RuntimeError,
        match="goal response did not arrive within safety budget",
    ) as raised:
        SessionOrchestratorNode._execute_sampled_nav2_goal(
            fake,
            request,
            sequence=1,
            goal_xy=(1.0, 2.0),
            deadline=10**12,
        )

    assert stop_calls == []
    assert stage_shutdowns == ["stop"]
    assert "navigation stage stopped by fail-safe" in str(raised.value)


def test_completed_goal_response_result_error_stops_navigation_stage():
    """done response 读取失败时无法证明 goal 未 accepted，必须停整个 stage。"""

    fake, _handle, _result, stop_calls, _statuses, stage_shutdowns = (
        _navigation_execution_fake(unsafe_runtime_path=False)
    )
    fake._navigate_to_pose_client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: _RaisingResultFuture(),
    )

    with pytest.raises(RuntimeError, match="navigation safety cleanup failed"):
        SessionOrchestratorNode._execute_sampled_nav2_goal(
            fake,
            SimpleNamespace(canceled=False),
            sequence=1,
            goal_xy=(1.0, 2.0),
            deadline=10**12,
        )

    assert stop_calls == []
    assert stage_shutdowns == ["stop"]


def test_accepted_handle_result_future_error_cancels_stops_and_shuts_stage():
    """accepted handle 丢失 result future 时，仍先停车再按不可证终态失败。"""

    fake, _handle, _result, stop_calls, _statuses, stage_shutdowns = (
        _navigation_execution_fake(unsafe_runtime_path=False)
    )
    handle = _AcceptedHandleWithoutResultFuture()
    fake._navigate_to_pose_client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: _DoneFuture(handle),
    )

    with pytest.raises(RuntimeError, match="navigation safety cleanup failed"):
        SessionOrchestratorNode._execute_sampled_nav2_goal(
            fake,
            SimpleNamespace(canceled=False),
            sequence=1,
            goal_xy=(1.0, 2.0),
            deadline=10**12,
        )

    assert handle.cancel_count == 1
    assert len(stop_calls) == 1
    assert stop_calls[0][0].canceled is False
    assert stop_calls[0][0].source == "navigation_safety_stop"
    assert stage_shutdowns == ["stop"]


@pytest.mark.parametrize(
    ("result_future", "expected_detail"),
    [
        (_RaisingResultFuture(), "result read failed"),
        (_DoneFuture(None), "returned no wrapper"),
        (
            _DoneFuture(
                SimpleNamespace(
                    status=GoalStatus.STATUS_EXECUTING,
                    result=SimpleNamespace(error_code=0),
                )
            ),
            "non-terminal status",
        ),
        (
            _DoneFuture(
                SimpleNamespace(
                    status=GoalStatus.STATUS_SUCCEEDED,
                    result=None,
                )
            ),
            "result payload is missing",
        ),
    ],
)
def test_navigation_cleanup_requires_readable_terminal_wrapper(
    result_future,
    expected_detail,
):
    """future.done 不是终态证据；wrapper status 与 payload 也必须可验证。"""

    cancel_calls = []
    stop_calls = []
    stage_shutdowns = []
    fake = SimpleNamespace(
        stop_motion_and_wait=lambda request, *, timeout_s: stop_calls.append(
            (request, timeout_s)
        ),
        _manager=SimpleNamespace(stop=lambda: stage_shutdowns.append("stop")),
    )
    handle = SimpleNamespace(
        cancel_goal_async=lambda: cancel_calls.append(True),
    )

    with pytest.raises(RuntimeError, match=expected_detail):
        SessionOrchestratorNode._cancel_nav2_goal_and_force_stop(
            fake,
            SimpleNamespace(canceled=True),
            handle=handle,
            result_future=result_future,
            reason="invalid Nav2 terminal evidence",
            timeout_s=0.001,
        )

    assert cancel_calls == [True]
    assert len(stop_calls) == 1
    assert stop_calls[0][0].canceled is False
    assert stage_shutdowns == ["stop"]


def test_navigation_safety_request_publishes_priority_typed_stop():
    """独立安全 request 最终必须发布 priority RobotCommand.STOP。"""

    published = []
    gateway_calls = []

    class _Gateway:
        @staticmethod
        def run_typed(
            request,
            *,
            command_id,
            publish_command,
            expected_action_name,
            timeout_s,
        ):
            gateway_calls.append(
                (request, command_id, expected_action_name, timeout_s)
            )
            publish_command()

    fake = SimpleNamespace(
        _internal_command_sequence=0,
        _mission_sequence=7,
        _agent_action_gateway=_Gateway(),
        _internal_action_pub=SimpleNamespace(publish=published.append),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())
        ),
    )
    request = showcase_session_node_module._navigation_safety_stop_request()

    SessionOrchestratorNode.stop_motion_and_wait(
        fake,
        request,
        timeout_s=0.5,
    )

    assert request.canceled is False
    assert request.source == "navigation_safety_stop"
    assert len(gateway_calls) == 1
    assert gateway_calls[0][0] is request
    assert gateway_calls[0][2:] == ("stop", 0.5)
    assert len(published) == 1
    assert isinstance(published[0], RobotCommand)
    assert published[0].action_type == RobotCommand.STOP
    assert published[0].priority is True


class _ManualNanosecondClock:
    """让 map_saver 延迟与零速采样时刻在测试中完全可控。"""

    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        return SimpleNamespace(nanoseconds=self.nanoseconds)


def _return_evidence_before_slow_map_save():
    """构造返航已成功、但其零速会被慢 map_saver 拖旧的证据。"""

    return ReturnToStartEvidence(
        start_pose=PlanarPose(0.0, 0.0, 0.0, "map", 1_000_000_000),
        final_pose=PlanarPose(0.02, 0.01, 0.01, "map", 5_050_000_000),
        action_kind=ReturnActionKind.NAVIGATE_TO_POSE,
        action_status=ReturnActionStatus.SUCCEEDED,
        action_command_id="slam-return-home-1",
        action_started_at_ns=2_000_000_000,
        action_finished_at_ns=5_000_000_000,
        map_saved_at_ns=0,
        cmd_vel_linear_x=0.0,
        cmd_vel_angular_z=0.0,
        cmd_vel_observed_at_ns=5_100_000_000,
        evaluated_at_ns=5_100_000_000,
    )


def test_save_map_refreshes_zero_velocity_after_slow_map_saver():
    """耗时保存后必须重新停车取证，不能拿返航时的陈旧零速硬凑 PASS。"""

    clock = _ManualNanosecondClock(5_100_000_000)
    events = []
    transitions = []
    draft = _return_evidence_before_slow_map_save()

    def save_map():
        events.append("map_saved")
        # 超过 max_cmd_vel_age_s=1.0，精确复放现场 final_cmd_vel_fresh=false。
        clock.nanoseconds = 6_300_000_001
        return "/tmp/unknown_world_map.yaml"

    fake = SimpleNamespace(
        _fsm=SimpleNamespace(snapshot=SimpleNamespace(map_saved=False)),
        _manager=SimpleNamespace(save_map=save_map),
        _return_to_start_evidence=draft,
        _return_to_start_spec=ReturnToStartSpec(max_cmd_vel_age_s=1.0),
        _cmd_vel_condition=threading.Condition(),
        _last_cmd_vel=(0.0, 0.0),
        _last_cmd_vel_observed_at_ns=draft.cmd_vel_observed_at_ns,
        get_clock=lambda: clock,
        transition=lambda phase, **kwargs: transitions.append((phase, kwargs)),
        feedback=lambda *_args, **_kwargs: None,
    )

    def stop_motion_and_wait(request, *, timeout_s):
        del request
        assert timeout_s > 0.0
        # 只有 map_saver 已经真实返回后，才允许刷新最终安全零速。
        assert events == ["map_saved"]
        events.append("post_save_stop")
        clock.nanoseconds = 6_350_000_000
        with fake._cmd_vel_condition:
            fake._last_cmd_vel = (0.0, 0.0)
            fake._last_cmd_vel_observed_at_ns = clock.nanoseconds

    fake.stop_motion_and_wait = stop_motion_and_wait

    SessionOrchestratorNode.save_map(fake, SimpleNamespace(message=""))

    finalized = fake._return_to_start_evidence
    decision = evaluate_return_to_start(finalized, fake._return_to_start_spec)
    assert events == ["map_saved", "post_save_stop"]
    assert finalized.action_finished_at_ns < finalized.map_saved_at_ns
    assert finalized.map_saved_at_ns <= finalized.cmd_vel_observed_at_ns
    assert finalized.cmd_vel_observed_at_ns <= finalized.evaluated_at_ns
    assert decision.passed is True
    assert transitions[-1][0] == SessionPhase.MAP_SAVED


def test_save_map_fails_closed_without_post_save_fresh_zero_velocity():
    """保存后 typed STOP 没有新零速时，不得发布 MAP_SAVED 或改写证据。"""

    clock = _ManualNanosecondClock(5_100_000_000)
    transitions = []
    stop_calls = []
    draft = _return_evidence_before_slow_map_save()

    def save_map():
        clock.nanoseconds = 6_300_000_001
        return "/tmp/unknown_world_map.yaml"

    def stop_motion_and_wait(request, *, timeout_s):
        stop_calls.append((request, timeout_s))
        raise TimeoutError("typed STOP completed without a fresh zero /cmd_vel")

    fake = SimpleNamespace(
        _fsm=SimpleNamespace(snapshot=SimpleNamespace(map_saved=False)),
        _manager=SimpleNamespace(save_map=save_map),
        _return_to_start_evidence=draft,
        _return_to_start_spec=ReturnToStartSpec(max_cmd_vel_age_s=1.0),
        _cmd_vel_condition=threading.Condition(),
        _last_cmd_vel=(0.0, 0.0),
        _last_cmd_vel_observed_at_ns=draft.cmd_vel_observed_at_ns,
        get_clock=lambda: clock,
        transition=lambda phase, **kwargs: transitions.append((phase, kwargs)),
        feedback=lambda *_args, **_kwargs: None,
        stop_motion_and_wait=stop_motion_and_wait,
    )

    with pytest.raises(TimeoutError, match="fresh zero /cmd_vel"):
        SessionOrchestratorNode.save_map(fake, SimpleNamespace(message=""))

    assert len(stop_calls) == 1
    assert fake._return_to_start_evidence is draft
    assert all(phase != SessionPhase.MAP_SAVED for phase, _ in transitions)


def test_navigation_safety_cleanup_refuses_missing_nav2_terminal(monkeypatch):
    """停车调用返回也不能掩盖仍处于活动态的直接 Nav2 goal。"""

    monkeypatch.setattr(showcase_session_node_module.time, "sleep", lambda _s: None)
    result_future = SimpleNamespace(done=lambda: False)
    cancel_calls = []
    stop_calls = []
    handle = SimpleNamespace(
        cancel_goal_async=lambda: cancel_calls.append(True),
    )
    stage_shutdowns = []
    fake = SimpleNamespace(
        stop_motion_and_wait=lambda request, *, timeout_s: stop_calls.append(
            (request, timeout_s)
        ),
        _manager=SimpleNamespace(stop=lambda: stage_shutdowns.append("stop")),
    )
    request = SimpleNamespace(
        canceled=False,
        command=SessionCommand.RUN_AUTOMATIC_MISSION,
    )

    with pytest.raises(RuntimeError, match="did not reach terminal"):
        SessionOrchestratorNode._cancel_nav2_goal_and_force_stop(
            fake,
            request,
            handle=handle,
            result_future=result_future,
            reason="unsafe runtime path",
            timeout_s=0.001,
        )

    assert cancel_calls == [True]
    assert len(stop_calls) == 1
    assert stop_calls[0][0] is not request
    assert stop_calls[0][0].canceled is False
    assert stop_calls[0][1] == 0.001
    assert stage_shutdowns == ["stop"]


def test_path_adapter_requires_map_frame_and_matching_goal_endpoint():
    points = _path_points_for_goal(
        _path("map", ((0.0, 0.0), (1.1, 2.1))),
        (1.0, 2.0),
    )

    assert points[-1] == pytest.approx((1.1, 2.1))
    with pytest.raises(ValueError, match="frame must be map"):
        _path_points_for_goal(_path("", ((1.0, 2.0),)), (1.0, 2.0))
    with pytest.raises(ValueError, match="endpoint does not match"):
        _path_points_for_goal(_path("map", ((4.0, 4.0),)), (1.0, 2.0))


def test_goal_admission_skips_two_no_path_candidates_then_selects_three():
    candidates = tuple((float(index * 2), 0.0) for index in range(5))
    attempts = []

    def admit(goal_xy):
        attempts.append(goal_xy)
        if len(attempts) <= 2:
            raise _GoalCandidateRejected("planner_error_208")

    selected = _admit_goal_candidates(
        candidates,
        count=3,
        minimum_separation_m=1.0,
        admit=admit,
    )

    assert selected == candidates[2:]
    assert attempts == list(candidates)


def test_goal_admission_reports_unsafe_and_no_path_when_batch_is_partial():
    candidates = tuple((float(index * 2), 0.0) for index in range(4))

    def admit(goal_xy):
        if goal_xy == candidates[1]:
            raise _GoalCandidateRejected("unsafe_path")
        if goal_xy == candidates[2]:
            raise _GoalCandidateRejected("planner_error_208")

    with pytest.raises(RuntimeError) as raised:
        _admit_goal_candidates(
            candidates,
            count=3,
            minimum_separation_m=1.0,
            admit=admit,
        )

    message = str(raised.value)
    assert "required=3 admitted=2 attempted=4" in message
    assert "planner_error_208:1" in message
    assert "unsafe_path:1" in message


def test_goal_admission_does_not_register_a_partial_batch(monkeypatch):
    occupancy = _occupancy_snapshot_from_message(
        SimpleNamespace(
            info=SimpleNamespace(
                width=6,
                height=6,
                resolution=1.0,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0),
                    orientation=SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=1.0
                    ),
                ),
            ),
            data=[0] * 36,
        )
    )
    candidates = ((1.5, 1.5), (3.5, 1.5))
    monkeypatch.setattr(
        showcase_session_node_module,
        "rank_mapped_goal_candidates",
        lambda *_args, **_kwargs: candidates,
        raising=False,
    )

    class _Ledger:
        def __init__(self):
            self.calls = []

        def plan(self, goals):
            self.calls.append(tuple(goals))

    ledger = _Ledger()
    fake = SimpleNamespace(
        _dry_run=False,
        _map_pose_condition=threading.Condition(),
        _latest_occupancy=occupancy,
        _latest_map_pose_xy=(0.5, 0.5),
        _navigation_input_generation=1,
        _latest_occupancy_generation=1,
        _latest_map_pose_generation=1,
        _navigation_goal_ledger=ledger,
        _publish_state=lambda: None,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )
    fake._request_preflight_path = lambda _request, *, goal_xy, deadline: _path(
        "map", ((0.5, 0.5), goal_xy)
    )

    with pytest.raises(RuntimeError, match="required=3 admitted=2"):
        SessionOrchestratorNode.select_mapped_navigation_goals(
            fake,
            SimpleNamespace(canceled=False),
            count=3,
            seed=7,
            minimum_separation_m=1.0,
            clearance_m=0.0,
            timeout_s=1.0,
        )

    assert ledger.calls == []


def test_goal_admission_rejects_nonempty_inputs_from_stale_generation():
    ledger = SimpleNamespace(plan=lambda _goals: pytest.fail("must not plan"))
    fake = SimpleNamespace(
        _dry_run=False,
        _map_pose_condition=threading.Condition(),
        _latest_occupancy=object(),
        _latest_map_pose_xy=(0.5, 0.5),
        _navigation_input_generation=2,
        _latest_occupancy_generation=1,
        _latest_map_pose_generation=1,
        _navigation_goal_ledger=ledger,
    )

    with pytest.raises(TimeoutError, match="fresh saved /map"):
        SessionOrchestratorNode.select_mapped_navigation_goals(
            fake,
            SimpleNamespace(canceled=False),
            count=3,
            seed=7,
            minimum_separation_m=1.0,
            clearance_m=0.0,
            timeout_s=0.001,
        )


def test_goal_admission_cancel_stops_before_compute_path(monkeypatch):
    candidates = ((1.0, 0.0), (2.0, 0.0), (3.0, 0.0))
    monkeypatch.setattr(
        showcase_session_node_module,
        "rank_mapped_goal_candidates",
        lambda *_args, **_kwargs: candidates,
    )
    fake = SimpleNamespace(
        _dry_run=False,
        _map_pose_condition=threading.Condition(),
        _latest_occupancy=object(),
        _latest_map_pose_xy=(0.0, 0.0),
        _navigation_input_generation=1,
        _latest_occupancy_generation=1,
        _latest_map_pose_generation=1,
        _navigation_goal_ledger=SimpleNamespace(
            plan=lambda _goals: pytest.fail("must not plan")
        ),
        _request_preflight_path=lambda *_args, **_kwargs: pytest.fail(
            "must not request ComputePathToPose"
        ),
    )

    with pytest.raises(AutomaticMissionCancelled):
        SessionOrchestratorNode.select_mapped_navigation_goals(
            fake,
            SimpleNamespace(canceled=True),
            count=3,
            seed=7,
            minimum_separation_m=1.0,
            clearance_m=0.0,
            timeout_s=1.0,
        )


def test_goal_admission_uses_one_deadline_and_registers_complete_batch(
    monkeypatch,
):
    occupancy = _occupancy_snapshot_from_message(
        SimpleNamespace(
            info=SimpleNamespace(
                width=7,
                height=3,
                resolution=1.0,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0),
                    orientation=SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=1.0
                    ),
                ),
            ),
            data=[0] * 21,
        )
    )
    candidates = ((1.5, 0.5), (3.5, 0.5), (5.5, 0.5))
    rank_calls = []

    def rank_candidates(snapshot, **kwargs):
        rank_calls.append((snapshot, kwargs))
        return candidates

    monkeypatch.setattr(
        showcase_session_node_module,
        "rank_mapped_goal_candidates",
        rank_candidates,
    )
    deadlines = []
    ledger_calls = []

    def preflight(_request, *, goal_xy, deadline):
        deadlines.append(deadline)
        return _path("map", ((0.5, 0.5), goal_xy))

    fake = SimpleNamespace(
        _dry_run=False,
        _map_pose_condition=threading.Condition(),
        _latest_occupancy=occupancy,
        _latest_map_pose_xy=(0.5, 0.5),
        _navigation_input_generation=3,
        _latest_occupancy_generation=3,
        _latest_map_pose_generation=3,
        _navigation_goal_ledger=SimpleNamespace(
            plan=lambda goals: ledger_calls.append(tuple(goals))
        ),
        _publish_state=lambda: None,
        _request_preflight_path=preflight,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )

    selected = SessionOrchestratorNode.select_mapped_navigation_goals(
        fake,
        SimpleNamespace(canceled=False),
        count=3,
        seed=7,
        minimum_separation_m=1.0,
        clearance_m=0.0,
        timeout_s=1.0,
    )

    assert selected == candidates
    assert ledger_calls == [candidates]
    assert len(deadlines) == 3
    assert deadlines[0] == deadlines[1] == deadlines[2]
    assert rank_calls == [
        (
            occupancy,
            {
                "start_xy": (0.5, 0.5),
                "seed": 7,
                "minimum_separation_m": 0.0,
                "clearance_m": 0.0,
                "candidate_limit": 24,
            },
        )
    ]


def test_goal_admission_skips_compute_path_that_crosses_unknown(monkeypatch):
    cells = [0] * 49
    cells[1 * 7 + 1] = -1
    occupancy = _occupancy_snapshot_from_message(
        SimpleNamespace(
            info=SimpleNamespace(
                width=7,
                height=7,
                resolution=1.0,
                origin=SimpleNamespace(
                    position=SimpleNamespace(x=0.0, y=0.0),
                    orientation=SimpleNamespace(
                        x=0.0, y=0.0, z=0.0, w=1.0
                    ),
                ),
            ),
            data=cells,
        )
    )
    unsafe = (2.5, 2.5)
    accepted = ((2.5, 0.5), (4.5, 0.5), (0.5, 4.5))
    candidates = (unsafe, *accepted)
    monkeypatch.setattr(
        showcase_session_node_module,
        "rank_mapped_goal_candidates",
        lambda *_args, **_kwargs: candidates,
    )
    ledger_calls = []

    def preflight(_request, *, goal_xy, deadline):
        del deadline
        if goal_xy == unsafe:
            return _path("map", ((0.5, 0.5), (1.5, 1.5), unsafe))
        return _path("map", ((0.5, 0.5), goal_xy))

    fake = SimpleNamespace(
        _dry_run=False,
        _map_pose_condition=threading.Condition(),
        _latest_occupancy=occupancy,
        _latest_map_pose_xy=(0.5, 0.5),
        _navigation_input_generation=1,
        _latest_occupancy_generation=1,
        _latest_map_pose_generation=1,
        _navigation_goal_ledger=SimpleNamespace(
            plan=lambda goals: ledger_calls.append(tuple(goals))
        ),
        _publish_state=lambda: None,
        _request_preflight_path=preflight,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )

    selected = SessionOrchestratorNode.select_mapped_navigation_goals(
        fake,
        SimpleNamespace(canceled=False),
        count=3,
        seed=7,
        minimum_separation_m=1.5,
        clearance_m=0.0,
        timeout_s=1.0,
    )

    assert selected == accepted
    assert ledger_calls == [accepted]


def test_compute_path_no_valid_path_is_a_skippable_candidate_rejection():
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(error_code=208, error_msg="no valid path"),
    )
    handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: _DoneFuture(wrapped),
    )
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: _DoneFuture(handle),
    )
    fake = SimpleNamespace(_compute_path_client=client)

    with pytest.raises(_GoalCandidateRejected, match="planner_error_208"):
        SessionOrchestratorNode._request_preflight_path(
            fake,
            SimpleNamespace(canceled=False),
            goal_xy=(1.0, 2.0),
            deadline=10**12,
        )


@pytest.mark.parametrize("error_code", [200, 201, 202, 203, 205, 207])
def test_compute_path_infrastructure_errors_are_fatal(error_code):
    wrapped = SimpleNamespace(
        status=GoalStatus.STATUS_ABORTED,
        result=SimpleNamespace(error_code=error_code, error_msg="fatal"),
    )
    handle = SimpleNamespace(
        accepted=True,
        get_result_async=lambda: _DoneFuture(wrapped),
    )
    client = SimpleNamespace(
        wait_for_server=lambda timeout_sec: True,
        send_goal_async=lambda _goal: _DoneFuture(handle),
    )
    fake = SimpleNamespace(_compute_path_client=client)

    with pytest.raises(RuntimeError, match="path preflight failed") as raised:
        SessionOrchestratorNode._request_preflight_path(
            fake,
            SimpleNamespace(canceled=False),
            goal_xy=(1.0, 2.0),
            deadline=10**12,
        )

    assert not isinstance(raised.value, _GoalCandidateRejected)


def test_compute_path_server_unavailable_is_a_fatal_timeout():
    fake = SimpleNamespace(
        _compute_path_client=SimpleNamespace(
            wait_for_server=lambda timeout_sec: False
        )
    )

    with pytest.raises(TimeoutError, match="Action server unavailable"):
        SessionOrchestratorNode._request_preflight_path(
            fake,
            SimpleNamespace(canceled=False),
            goal_xy=(1.0, 2.0),
            deadline=10**12,
        )


def test_goal_admission_timeout_does_not_try_later_candidates():
    candidates = ((1.0, 0.0), (2.0, 0.0), (3.0, 0.0))
    attempts = []

    def admit(goal_xy):
        attempts.append(goal_xy)
        raise TimeoutError("shared admission deadline exhausted")

    with pytest.raises(TimeoutError, match="shared admission deadline"):
        _admit_goal_candidates(
            candidates,
            count=2,
            minimum_separation_m=0.5,
            admit=admit,
        )

    assert attempts == [candidates[0]]
