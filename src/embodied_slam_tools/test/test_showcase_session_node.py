"""Focused tests for ROS-facing showcase orchestrator helpers."""

import math
import queue
import threading
import time
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from embodied_agent_interfaces.msg import (
    ControlAuthorityState,
    RobotCommand,
    SlamSessionState,
    WakeEvent,
)
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
import pytest
from std_msgs.msg import String

import embodied_slam_tools.showcase_session_node as showcase_session_node_module
from embodied_slam_tools.autonomy_quiescence import (
    AutonomyQuiescenceBarrier,
    QuiescenceError,
    QuiescenceIdentity,
    QuiescenceState,
)
from embodied_slam_tools.control_authority_lease import (
    AUTONOMY,
    AuthoritySnapshot,
    ControlAuthorityLease,
)
from embodied_slam_tools.mission_executor import (
    AutomaticMissionCancelled,
    CommandRequest,
    wait_for_required_event,
)
from embodied_slam_tools.nav2_motion_transaction import (
    MappingReturn,
    RecoveryBackup,
    SampledNavigate,
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


class _MessagePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _authority_receiver(active_request=None):
    publisher = _MessagePublisher()
    warnings = []
    errors = []
    receiver = SimpleNamespace(
        _authority_gate_enabled=True,
        _authority_lease=ControlAuthorityLease(lease_s=1.0),
        _autonomy_quiescence=AutonomyQuiescenceBarrier(),
        _state_lock=threading.RLock(),
        _active_request=active_request,
        _explore_control_pub=publisher,
        get_logger=lambda: SimpleNamespace(
            warning=warnings.append,
            error=errors.append,
        ),
        _fail_autonomy_quiescence=lambda error: errors.append(str(error)),
    )
    return receiver, publisher, warnings, errors


def _authority_state(
    authority,
    sequence=1,
    epoch=1,
    *,
    pending_revocation=0,
    quiescence_acknowledged=None,
):
    message = ControlAuthorityState()
    message.authority = authority
    message.estop_latched = authority == ControlAuthorityState.ESTOP
    message.manager_epoch = epoch
    message.transition_sequence = sequence
    message.active_source = {
        ControlAuthorityState.HOLD: "",
        ControlAuthorityState.AUTONOMY: "autonomy",
        ControlAuthorityState.KEYBOARD: "keyboard_teleop",
        ControlAuthorityState.ESTOP: "safety_panel",
    }[authority]
    message.reason = (
        "initialized"
        if authority == ControlAuthorityState.HOLD and sequence == 0
        else "test"
    )
    if quiescence_acknowledged is None:
        quiescence_acknowledged = authority != ControlAuthorityState.AUTONOMY
    if hasattr(message, "pending_autonomy_revocation_sequence"):
        message.pending_autonomy_revocation_sequence = pending_revocation
    if hasattr(message, "autonomy_quiescence_acknowledged"):
        message.autonomy_quiescence_acknowledged = quiescence_acknowledged
    return message


@pytest.mark.parametrize(
    "authority",
    [
        ControlAuthorityState.HOLD,
        ControlAuthorityState.KEYBOARD,
        ControlAuthorityState.ESTOP,
    ],
)
def test_authority_takeover_cancels_active_automatic_mission_once(authority):
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )
    receiver, publisher, warnings, _errors = _authority_receiver(request)

    SessionOrchestratorNode._on_control_authority_state(
        receiver, _authority_state(authority)
    )
    SessionOrchestratorNode._on_control_authority_state(
        receiver, _authority_state(authority, sequence=2)
    )

    assert request.canceled
    assert [message.data for message in publisher.messages] == [False]
    assert len(warnings) == 1


def test_returning_to_autonomy_never_resumes_canceled_mission():
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )
    request.canceled = True
    receiver, publisher, _warnings, _errors = _authority_receiver(request)

    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(ControlAuthorityState.AUTONOMY),
    )

    assert request.canceled
    assert publisher.messages == []


def test_authority_takeover_does_not_cancel_non_automatic_request():
    request = CommandRequest(SessionCommand.SAVE_MAP, "test")
    receiver, publisher, _warnings, _errors = _authority_receiver(request)

    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(ControlAuthorityState.KEYBOARD),
    )

    assert not request.canceled
    assert publisher.messages == []


def test_reordered_or_conflicting_authority_cannot_cancel_or_replace_autonomy():
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )
    receiver, publisher, _warnings, errors = _authority_receiver(request)

    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(ControlAuthorityState.AUTONOMY, sequence=2),
    )
    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(ControlAuthorityState.HOLD, sequence=1),
    )
    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(ControlAuthorityState.KEYBOARD, sequence=2),
    )

    assert not request.canceled
    assert receiver._authority_lease.authority == AUTONOMY
    assert publisher.messages == []
    assert len(errors) == 2


def test_authority_lease_expiry_cancels_and_fails_closed(monkeypatch):
    clock = [1.0]
    monkeypatch.setattr(
        showcase_session_node_module.time, "monotonic", lambda: clock[0]
    )
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )
    receiver, publisher, warnings, errors = _authority_receiver(request)
    receiver._authority_lease = ControlAuthorityLease(lease_s=0.1)
    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(ControlAuthorityState.AUTONOMY),
    )
    assert not request.canceled

    clock[0] = 1.101
    SessionOrchestratorNode._check_authority_lease(receiver)

    assert request.canceled
    assert [message.data for message in publisher.messages] == [False]
    assert warnings == [
        "canceling active automatic mission: authority_lease_expired"
    ]
    assert errors == [
        "authority manager lease expired before typed revocation"
    ]


def test_stale_autonomy_heartbeat_cannot_bridge_a_lease_gap(monkeypatch):
    clock = [1.0]
    monkeypatch.setattr(
        showcase_session_node_module.time, "monotonic", lambda: clock[0]
    )
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )
    receiver, publisher, warnings, errors = _authority_receiver(request)
    receiver._authority_lease = ControlAuthorityLease(lease_s=0.1)
    autonomy = _authority_state(
        ControlAuthorityState.AUTONOMY,
        sequence=3,
        epoch=8,
        quiescence_acknowledged=False,
    )
    SessionOrchestratorNode._on_control_authority_state(receiver, autonomy)

    # timer 尚未来得及运行时迟到 heartbeat 也不能把同 epoch 租约“续活”。
    clock[0] = 1.101
    SessionOrchestratorNode._on_control_authority_state(receiver, autonomy)

    assert request.canceled
    assert [message.data for message in publisher.messages] == [False]
    assert warnings == [
        "canceling active automatic mission: authority_lease_discontinuity"
    ]
    assert any("lease_discontinuity" in message for message in errors)


def test_authority_revocation_opens_exact_quiescence_before_worker_start():
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )
    receiver, _publisher, _warnings, _errors = _authority_receiver(request)
    started = []
    receiver._start_autonomy_quiescence_worker = started.append

    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(
            ControlAuthorityState.KEYBOARD,
            sequence=4,
            epoch=9,
            pending_revocation=4,
            quiescence_acknowledged=False,
        ),
    )

    identity = QuiescenceIdentity(9, 4)
    assert request.canceled
    assert receiver._autonomy_quiescence.identity == identity
    assert (
        receiver._autonomy_quiescence.snapshot().state
        is QuiescenceState.WAITING
    )
    assert started == [identity]


def test_internal_stage_switch_hold_quiesces_without_canceling_outer_mission():
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )
    receiver, publisher, warnings, errors = _authority_receiver(request)
    started = []
    receiver._start_autonomy_quiescence_worker = started.append
    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(
            ControlAuthorityState.AUTONOMY,
            sequence=3,
            epoch=12,
            quiescence_acknowledged=False,
        ),
    )
    token = "internal_stage_switch:12:3:1"
    receiver._internal_stage_switch = SimpleNamespace(
        reason=token,
        manager_epoch=12,
        transition_sequence_floor=3,
        identity=None,
    )
    hold = _authority_state(
        ControlAuthorityState.HOLD,
        sequence=4,
        epoch=12,
        pending_revocation=4,
        quiescence_acknowledged=False,
    )
    hold.reason = token

    SessionOrchestratorNode._on_control_authority_state(receiver, hold)

    identity = QuiescenceIdentity(12, 4)
    assert not request.canceled
    assert request.authority_revocation_identity is None
    assert receiver._internal_stage_switch.identity == identity
    assert receiver._autonomy_quiescence.identity == identity
    assert started == [identity]
    assert [message.data for message in publisher.messages] == [False]
    assert warnings == []
    assert errors == []


def test_internal_stage_switch_resume_does_not_cancel_outer_mission():
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )
    receiver, publisher, warnings, errors = _authority_receiver(request)
    receiver._start_autonomy_quiescence_worker = lambda _identity: None
    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(
            ControlAuthorityState.AUTONOMY,
            sequence=3,
            epoch=12,
            quiescence_acknowledged=False,
        ),
    )
    identity = QuiescenceIdentity(12, 4)
    receiver._internal_stage_switch = SimpleNamespace(
        reason="internal_stage_switch:12:3:1",
        manager_epoch=12,
        transition_sequence_floor=3,
        identity=None,
    )
    hold = _authority_state(
        ControlAuthorityState.HOLD,
        sequence=4,
        epoch=12,
        pending_revocation=4,
        quiescence_acknowledged=False,
    )
    hold.reason = receiver._internal_stage_switch.reason
    SessionOrchestratorNode._on_control_authority_state(receiver, hold)
    barrier = receiver._autonomy_quiescence
    barrier.mark_priority_stop_requested("stop-12-4", cmd_vel_generation=1)
    barrier.observe_priority_stop_result("stop-12-4", success=True)
    barrier.observe_cmd_vel(generation=2, linear_x=0.0, angular_z=0.0)
    barrier.begin_acknowledgement(identity)
    barrier.complete_acknowledgement(identity)

    resumed = _authority_state(
        ControlAuthorityState.AUTONOMY,
        sequence=6,
        epoch=12,
        quiescence_acknowledged=False,
    )
    resumed.reason = f"{receiver._internal_stage_switch.reason}:resume"
    SessionOrchestratorNode._on_control_authority_state(receiver, resumed)

    assert not request.canceled
    assert [message.data for message in publisher.messages] == [False]
    assert warnings == []
    assert errors == []


@pytest.mark.parametrize(
    "external_authority",
    [ControlAuthorityState.KEYBOARD, ControlAuthorityState.ESTOP],
)
def test_user_takeover_supersedes_internal_stage_switch_hold(
    external_authority,
):
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )
    receiver, _publisher, warnings, errors = _authority_receiver(request)
    receiver._start_autonomy_quiescence_worker = lambda _identity: None
    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(
            ControlAuthorityState.AUTONOMY,
            sequence=3,
            epoch=12,
            quiescence_acknowledged=False,
        ),
    )
    token = "internal_stage_switch:12:3:1"
    receiver._internal_stage_switch = SimpleNamespace(
        reason=token,
        manager_epoch=12,
        transition_sequence_floor=3,
        identity=None,
    )
    hold = _authority_state(
        ControlAuthorityState.HOLD,
        sequence=4,
        epoch=12,
        pending_revocation=4,
        quiescence_acknowledged=False,
    )
    hold.reason = token
    SessionOrchestratorNode._on_control_authority_state(receiver, hold)

    takeover = _authority_state(
        external_authority,
        sequence=5,
        epoch=12,
        pending_revocation=4,
        quiescence_acknowledged=False,
    )
    SessionOrchestratorNode._on_control_authority_state(receiver, takeover)

    assert request.canceled
    assert request.authority_revocation_identity == (12, 4)
    assert len(warnings) == 1
    assert errors == []


def test_idle_authority_revocation_does_not_leave_session_quiescing():
    receiver, _publisher, _warnings, _errors = _authority_receiver()
    started = []
    transitions = []
    receiver._start_autonomy_quiescence_worker = started.append
    receiver.transition = lambda phase, **kwargs: transitions.append(
        (phase, kwargs)
    )

    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(
            ControlAuthorityState.KEYBOARD,
            sequence=4,
            epoch=9,
            pending_revocation=4,
            quiescence_acknowledged=False,
        ),
    )

    # 没有 active automatic request 时屏障仍会收口并门控新命令，但 phase
    # 不应进入无人负责恢复的 QUIESCING。
    assert transitions == []
    assert receiver._autonomy_quiescence.active
    assert started == [QuiescenceIdentity(9, 4)]


def test_missing_quiescence_ack_field_defaults_to_fail_closed():
    receiver, _publisher, _warnings, errors = _authority_receiver()
    started = []
    receiver._start_autonomy_quiescence_worker = started.append
    legacy_state = SimpleNamespace(
        authority=ControlAuthorityState.HOLD,
        estop_latched=False,
        manager_epoch=11,
        transition_sequence=0,
        active_source="",
        reason="initialized",
    )

    SessionOrchestratorNode._on_control_authority_state(
        receiver, legacy_state
    )

    assert errors == []
    assert receiver._autonomy_quiescence.active
    assert started == [QuiescenceIdentity(11, 0)]


def test_authority_callbacks_allow_fast_resume_then_open_next_revocation():
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )
    receiver, _publisher, _warnings, errors = _authority_receiver(request)
    started = []
    receiver._start_autonomy_quiescence_worker = started.append

    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(
            ControlAuthorityState.AUTONOMY,
            sequence=1,
            epoch=9,
            quiescence_acknowledged=False,
        ),
    )
    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(
            ControlAuthorityState.KEYBOARD,
            sequence=2,
            epoch=9,
            pending_revocation=2,
            quiescence_acknowledged=False,
        ),
    )
    first = QuiescenceIdentity(9, 2)
    barrier = receiver._autonomy_quiescence
    barrier.mark_priority_stop_requested("stop-9-2", cmd_vel_generation=1)
    barrier.observe_priority_stop_result("stop-9-2", success=True)
    barrier.observe_cmd_vel(generation=2, linear_x=0.0, angular_z=0.0)
    barrier.begin_acknowledgement(first)

    # 合法 ACK topic 可能被 executor 跳过，直接先看到 RESUME。
    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(
            ControlAuthorityState.AUTONOMY,
            sequence=4,
            epoch=9,
            quiescence_acknowledged=False,
        ),
    )
    SessionOrchestratorNode._on_control_authority_state(
        receiver,
        _authority_state(
            ControlAuthorityState.HOLD,
            sequence=5,
            epoch=9,
            pending_revocation=5,
            quiescence_acknowledged=False,
        ),
    )

    assert errors == []
    assert barrier.identity == QuiescenceIdentity(9, 5)
    assert barrier.snapshot().state is QuiescenceState.WAITING
    assert started == [first, QuiescenceIdentity(9, 5)]


def test_quiescence_worker_acknowledges_only_after_all_evidence():
    identity = QuiescenceIdentity(7, 3)
    barrier = AutonomyQuiescenceBarrier()
    barrier.observe_authority(identity)
    barrier.begin(identity)
    calls = []

    def request_stop(observed_identity, *, timeout_s):
        calls.append(("stop", observed_identity, timeout_s))
        barrier.mark_priority_stop_requested(
            "quiescence-stop-7-3",
            cmd_vel_generation=10,
        )
        barrier.observe_priority_stop_result(
            "quiescence-stop-7-3",
            success=True,
        )
        barrier.observe_cmd_vel(
            generation=11,
            linear_x=0.0,
            angular_z=0.0,
        )

    fake = SimpleNamespace(
        _autonomy_quiescence=barrier,
        _autonomy_quiescence_timeout_s=1.0,
        _request_quiescence_priority_stop=request_stop,
        _acknowledge_autonomy_quiescence=lambda observed, *, timeout_s: (
            calls.append(("ack", observed, timeout_s))
        ),
        _fail_autonomy_quiescence=lambda error: pytest.fail(str(error)),
        get_logger=lambda: SimpleNamespace(info=lambda message: calls.append(
            ("log", message)
        )),
    )

    SessionOrchestratorNode._complete_autonomy_quiescence(
        fake, identity
    )

    assert calls[0][0] == "stop"
    assert calls[1][0] == "ack"
    assert barrier.snapshot().state is QuiescenceState.ACKNOWLEDGED


def test_quiescence_worker_accepts_resume_callback_before_service_returns():
    identity = QuiescenceIdentity(7, 3)
    barrier = AutonomyQuiescenceBarrier()
    barrier.observe_authority(identity)
    barrier.begin(identity)

    def request_stop(_observed, *, timeout_s):
        assert timeout_s > 0.0
        barrier.mark_priority_stop_requested(
            "quiescence-stop-7-3", cmd_vel_generation=10
        )
        barrier.observe_priority_stop_result(
            "quiescence-stop-7-3", success=True
        )
        barrier.observe_cmd_vel(
            generation=11, linear_x=0.0, angular_z=0.0
        )

    def acknowledge(_observed, *, timeout_s):
        assert timeout_s > 0.0
        # 模拟 MultiThreadedExecutor 先调度 authority topic，后唤醒 service future。
        barrier.observe_authority(
            QuiescenceIdentity(identity.manager_epoch, 0),
            autonomy_active=True,
        )

    errors = []
    fake = SimpleNamespace(
        _autonomy_quiescence=barrier,
        _autonomy_quiescence_timeout_s=1.0,
        _request_quiescence_priority_stop=request_stop,
        _acknowledge_autonomy_quiescence=acknowledge,
        _fail_autonomy_quiescence=errors.append,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )

    SessionOrchestratorNode._complete_autonomy_quiescence(fake, identity)

    assert errors == []
    assert barrier.snapshot().state is QuiescenceState.ACKNOWLEDGED


def test_quiescence_worker_rejects_evidence_invalidated_during_ack_future():
    identity = QuiescenceIdentity(7, 3)
    barrier = AutonomyQuiescenceBarrier()
    barrier.observe_authority(identity)
    barrier.begin(identity)

    def request_stop(_observed, *, timeout_s):
        assert timeout_s > 0.0
        barrier.mark_priority_stop_requested("stop-7-3", cmd_vel_generation=10)
        barrier.observe_priority_stop_result("stop-7-3", success=True)
        barrier.observe_cmd_vel(
            generation=11, linear_x=0.0, angular_z=0.0
        )

    def acknowledge(_observed, *, timeout_s):
        assert timeout_s > 0.0
        barrier.observe_cmd_vel(
            generation=12, linear_x=0.1, angular_z=0.0
        )

    errors = []
    fake = SimpleNamespace(
        _autonomy_quiescence=barrier,
        _autonomy_quiescence_timeout_s=1.0,
        _request_quiescence_priority_stop=request_stop,
        _acknowledge_autonomy_quiescence=acknowledge,
        _fail_autonomy_quiescence=errors.append,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )

    SessionOrchestratorNode._complete_autonomy_quiescence(fake, identity)

    assert len(errors) == 1
    assert "evidence invalidated" in str(errors[0])
    assert barrier.snapshot().state is QuiescenceState.FAILED


def test_quiescence_worker_hands_off_second_identity_without_losing_it():
    first = QuiescenceIdentity(7, 3)
    second = QuiescenceIdentity(7, 6)
    barrier = AutonomyQuiescenceBarrier()
    barrier.observe_authority(first)
    barrier.begin(first)
    barrier.mark_priority_stop_requested("stop-1", cmd_vel_generation=1)
    barrier.observe_priority_stop_result("stop-1", success=True)
    barrier.observe_cmd_vel(generation=2, linear_x=0.0, angular_z=0.0)
    barrier.begin_acknowledgement(first)

    first_started = threading.Event()
    release_first = threading.Event()
    second_completed = threading.Event()
    calls = []

    def complete(identity):
        calls.append(identity)
        if identity == first:
            first_started.set()
            assert release_first.wait(timeout=1.0)
            return
        barrier.fail("second identity observed")
        second_completed.set()

    fake = SimpleNamespace(
        _autonomy_quiescence=barrier,
        _quiescence_worker_lock=threading.Lock(),
        _quiescence_worker=None,
        _complete_autonomy_quiescence=complete,
    )
    SessionOrchestratorNode._start_autonomy_quiescence_worker(fake, first)
    assert first_started.wait(timeout=1.0)

    barrier.observe_authority(second)
    barrier.begin(second)
    # 旧实现会因第一条线程仍 alive 而永久丢掉这次启动。
    SessionOrchestratorNode._start_autonomy_quiescence_worker(fake, second)
    release_first.set()

    assert second_completed.wait(timeout=1.0)
    assert calls == [first, second]


class _ImmediateServiceFuture:
    def __init__(self, response):
        self._response = response

    def add_done_callback(self, callback):
        callback(self)

    def result(self):
        return self._response


def test_internal_stage_switch_enters_exact_hold_and_waits_for_ack(
    monkeypatch,
):
    class _ServiceType:
        class Request:
            ENTER_HOLD = 3
            RESUME_AUTONOMY = 4

            def __init__(self):
                self.command = 0
                self.requester = ""
                self.reason = ""

    barrier = AutonomyQuiescenceBarrier()
    requests = []
    fake = SimpleNamespace(
        _state_lock=threading.RLock(),
        _authority_lease=ControlAuthorityLease(lease_s=2.0),
        _authority_snapshot=AuthoritySnapshot(
            AUTONOMY,
            False,
            44,
            10,
            "autonomy",
            "ready",
        ),
        _internal_stage_switch=None,
        _internal_stage_switch_sequence=0,
        _authority_control_client=None,
        _autonomy_quiescence=barrier,
        _autonomy_quiescence_timeout_s=0.5,
    )
    fake._authority_lease.update(
        fake._authority_snapshot,
        showcase_session_node_module.time.monotonic(),
    )

    def call_async(service_request):
        requests.append(service_request)
        identity = QuiescenceIdentity(44, 11)
        barrier.observe_authority(identity)
        barrier.begin(identity)
        barrier.mark_priority_stop_requested(
            "quiescence-stop-44-11", cmd_vel_generation=1
        )
        barrier.observe_priority_stop_result(
            "quiescence-stop-44-11", success=True
        )
        barrier.observe_cmd_vel(
            generation=2, linear_x=0.0, angular_z=0.0
        )
        barrier.begin_acknowledgement(identity)
        barrier.complete_acknowledgement(identity)
        return _ImmediateServiceFuture(
            SimpleNamespace(
                accepted=True,
                changed=True,
                stop_requested=True,
                message="entered_hold",
                state=SimpleNamespace(
                    authority=ControlAuthorityState.HOLD,
                    estop_latched=False,
                    manager_epoch=44,
                    transition_sequence=11,
                    pending_autonomy_revocation_sequence=11,
                    autonomy_quiescence_acknowledged=False,
                    active_source="",
                    reason=service_request.reason,
                ),
            )
        )

    fake._authority_control_client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: timeout_sec > 0.0,
        call_async=call_async,
    )
    monkeypatch.setattr(
        showcase_session_node_module, "SetControlAuthority", _ServiceType
    )
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )

    identity = SessionOrchestratorNode._enter_internal_stage_switch_hold(
        fake, request
    )

    assert identity == QuiescenceIdentity(44, 11)
    assert len(requests) == 1
    assert requests[0].command == _ServiceType.Request.ENTER_HOLD
    assert requests[0].requester == "slam_session_stage_switch"
    assert requests[0].reason.startswith("internal_stage_switch:44:10:")
    assert not request.canceled


def test_internal_stage_switch_resumes_only_the_acknowledged_identity(
    monkeypatch,
):
    class _ServiceType:
        class Request:
            ENTER_HOLD = 3
            RESUME_AUTONOMY = 4

            def __init__(self):
                self.command = 0
                self.requester = ""
                self.reason = ""

    identity = QuiescenceIdentity(44, 11)
    lock = threading.RLock()
    requests = []
    fake = SimpleNamespace(
        _state_lock=lock,
        _authority_condition=threading.Condition(lock),
        _authority_snapshot=AuthoritySnapshot(
            ControlAuthorityState.HOLD,
            False,
            44,
            12,
            "",
            "quiescence acknowledged",
            pending_autonomy_revocation_sequence=11,
            autonomy_quiescence_acknowledged=True,
        ),
        _internal_stage_switch=SimpleNamespace(
            reason="internal_stage_switch:44:10:1",
            manager_epoch=44,
            transition_sequence_floor=10,
            identity=identity,
        ),
        _authority_control_client=None,
        _autonomy_quiescence_timeout_s=0.5,
    )

    def call_async(service_request):
        requests.append(service_request)
        resumed = AuthoritySnapshot(
            AUTONOMY,
            False,
            44,
            13,
            "autonomy",
            service_request.reason,
        )
        with fake._authority_condition:
            fake._authority_snapshot = resumed
            fake._authority_condition.notify_all()
        return _ImmediateServiceFuture(
            SimpleNamespace(
                accepted=True,
                changed=True,
                stop_requested=False,
                message="autonomy_resumed",
                state=resumed,
            )
        )

    fake._authority_control_client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: timeout_sec > 0.0,
        call_async=call_async,
    )
    monkeypatch.setattr(
        showcase_session_node_module, "SetControlAuthority", _ServiceType
    )

    SessionOrchestratorNode._resume_internal_stage_switch(
        fake, identity
    )

    assert len(requests) == 1
    assert requests[0].command == _ServiceType.Request.RESUME_AUTONOMY
    assert requests[0].requester == "slam_session_stage_switch"
    assert requests[0].reason.endswith(":resume")


def test_quiescence_ack_service_is_bound_to_exact_epoch_and_revocation(
    monkeypatch,
):
    class _ServiceType:
        class Request:
            def __init__(self):
                self.manager_epoch = 0
                self.autonomy_revocation_sequence = 0
                self.requester = ""
                self.detail = ""

    identity = QuiescenceIdentity(12, 8)
    requests = []
    response = SimpleNamespace(
        accepted=True,
        message="acknowledged",
        state=SimpleNamespace(
            manager_epoch=12,
            pending_autonomy_revocation_sequence=8,
            autonomy_quiescence_acknowledged=True,
        ),
    )
    client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: timeout_sec > 0.0,
        call_async=lambda request: requests.append(request)
        or _ImmediateServiceFuture(response),
    )
    fake = SimpleNamespace(_quiescence_ack_client=client)
    monkeypatch.setattr(
        showcase_session_node_module,
        "AcknowledgeAutonomyQuiescence",
        _ServiceType,
    )

    SessionOrchestratorNode._acknowledge_autonomy_quiescence(
        fake,
        identity,
        timeout_s=0.1,
    )

    assert len(requests) == 1
    assert requests[0].manager_epoch == 12
    assert requests[0].autonomy_revocation_sequence == 8
    assert requests[0].requester == "slam_session_orchestrator"


def test_quiescence_timeout_marks_session_failed_and_never_recovers():
    barrier = AutonomyQuiescenceBarrier()
    identity = QuiescenceIdentity(5, 2)
    barrier.observe_authority(identity)
    barrier.begin(identity)
    transitions = []
    errors = []
    fake = SimpleNamespace(
        _autonomy_quiescence=barrier,
        _state_lock=threading.RLock(),
        _mission_outcome=SlamSessionState.MISSION_RUNNING,
        _mission_message="",
        transition=lambda phase, **kwargs: transitions.append(
            (phase, kwargs["detail"])
        ),
        get_logger=lambda: SimpleNamespace(error=errors.append),
    )

    SessionOrchestratorNode._fail_autonomy_quiescence(
        fake,
        TimeoutError("Nav2 terminal missing"),
    )

    assert fake._mission_outcome == SlamSessionState.MISSION_FAILED
    assert transitions[-1][0] is SessionPhase.FAILED
    assert "Nav2 terminal missing" in transitions[-1][1]
    assert barrier.snapshot().state is QuiescenceState.FAILED
    assert errors == [transitions[-1][1]]


def _authority_enqueue_receiver(tracker):
    return SimpleNamespace(
        _operation_active=threading.Event(),
        _state_lock=threading.RLock(),
        _authority_gate_enabled=True,
        _authority_lease=tracker,
        _autonomy_quiescence=AutonomyQuiescenceBarrier(),
        _fsm=SimpleNamespace(validate=lambda _command: (True, "")),
        _requests=queue.Queue(maxsize=2),
    )


def test_quiescence_rejects_save_and_stage_switch_before_ack():
    tracker = ControlAuthorityLease(lease_s=1.0)
    receiver = _authority_enqueue_receiver(tracker)
    identity = QuiescenceIdentity(9, 4)
    receiver._autonomy_quiescence.observe_authority(identity)
    receiver._autonomy_quiescence.begin(identity)

    for command in (
        SessionCommand.SAVE_MAP,
        SessionCommand.START_NAVIGATION,
        SessionCommand.SAVE_AND_START_NAVIGATION,
    ):
        request = CommandRequest(command, "test")
        assert not SessionOrchestratorNode._enqueue(receiver, request)
        assert request.completed.is_set()
        assert request.message == "autonomy quiescence is still in progress"

    assert receiver._requests.empty()


def test_automatic_mission_enqueue_requires_fresh_autonomy(monkeypatch):
    clock = [1.0]
    monkeypatch.setattr(
        showcase_session_node_module.time, "monotonic", lambda: clock[0]
    )
    tracker = ControlAuthorityLease(lease_s=0.1)
    tracker.update(
        AuthoritySnapshot(AUTONOMY, False, 1, 1, "autonomy", "test"),
        1.0,
    )
    receiver = _authority_enqueue_receiver(tracker)
    clock[0] = 1.101
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )

    assert not SessionOrchestratorNode._enqueue(receiver, request)
    assert request.completed.is_set()
    assert request.message == (
        "automatic mission requires fresh AUTONOMY authority"
    )


def test_worker_rejects_request_from_previous_authority_generation():
    tracker = ControlAuthorityLease(lease_s=1.0)
    tracker.update(
        AuthoritySnapshot(AUTONOMY, False, 1, 1, "autonomy", "test"),
        showcase_session_node_module.time.monotonic(),
    )
    receiver = _authority_enqueue_receiver(tracker)
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION,
        "test",
        authority_generation=tracker.generation - 1,
    )

    SessionOrchestratorNode._execute_request(receiver, request)

    assert request.completed.is_set()
    assert not receiver._operation_active.is_set()
    assert request.message == (
        "automatic mission authority generation expired before execution"
    )


def test_canceled_mission_keeps_operation_locked_until_exact_quiescence_ack():
    tracker = ControlAuthorityLease(lease_s=10.0)
    tracker.update(
        AuthoritySnapshot(AUTONOMY, False, 1, 1, "autonomy", "test"),
        showcase_session_node_module.time.monotonic(),
    )
    barrier = AutonomyQuiescenceBarrier()
    identity = QuiescenceIdentity(1, 2)
    entered_cancel = threading.Event()
    transitions = []

    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION,
        "test",
        authority_generation=tracker.generation,
    )

    def run_and_cancel(active_request):
        barrier.observe_authority(identity)
        barrier.begin(identity)
        active_request.authority_revocation_identity = (
            identity.manager_epoch,
            identity.revocation_sequence,
        )
        active_request.canceled = True
        entered_cancel.set()
        raise AutomaticMissionCancelled("authority revoked")

    fake = SimpleNamespace(
        _state_lock=threading.RLock(),
        _fsm=SimpleNamespace(validate=lambda _command: (True, "")),
        _authority_gate_enabled=True,
        _authority_lease=tracker,
        _operation_active=threading.Event(),
        _active_request=None,
        _navigation_startup_failure_pending=False,
        _mission_sequence=0,
        _mission_outcome=SlamSessionState.MISSION_IDLE,
        _mission_message="",
        _navigation_goal_ledger=SimpleNamespace(reset=lambda _sequence: None),
        _mapping_saturation_evidence=None,
        _mapping_saturation_assessment=None,
        _return_to_start_evidence=None,
        _publish_state=lambda: None,
        _automatic_mission_executor=SimpleNamespace(run=run_and_cancel),
        _autonomy_quiescence=barrier,
        _autonomy_quiescence_timeout_s=1.0,
        _manager=SimpleNamespace(stage="mapping"),
        transition=lambda phase, **kwargs: transitions.append(
            (phase, kwargs["detail"])
        ),
        get_logger=lambda: SimpleNamespace(
            warning=lambda _message: None,
            error=lambda _message: None,
        ),
    )

    worker = threading.Thread(
        target=SessionOrchestratorNode._execute_request,
        args=(fake, request),
    )
    worker.start()
    assert entered_cancel.wait(timeout=1.0)
    # 没有 exact ACK 时事务必须仍占用，SAVE/START_NAV 才不能切走 stage。
    assert fake._operation_active.is_set()
    assert worker.is_alive()

    barrier.mark_priority_stop_requested("stop-1-2", cmd_vel_generation=1)
    barrier.observe_priority_stop_result("stop-1-2", success=True)
    barrier.observe_cmd_vel(generation=2, linear_x=0.0, angular_z=0.0)
    barrier.begin_acknowledgement(identity)
    barrier.complete_acknowledgement(identity)

    worker.join(timeout=1.0)
    assert not worker.is_alive()
    assert not fake._operation_active.is_set()
    assert transitions[-1][0] is SessionPhase.MAPPING


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


@pytest.mark.parametrize(
    ("persistent_runtime_enabled", "expected_calls"),
    [
        (False, [("start", "mapping")]),
        (
            True,
            [
                ("start_base",),
                ("base_control_ready",),
                ("start_mapping",),
                ("stage_executor_ready",),
            ],
        ),
    ],
)
def test_mapping_start_selects_compatible_or_persistent_runtime(
    persistent_runtime_enabled,
    expected_calls,
):
    calls = []
    manager = SimpleNamespace(
        start=lambda stage: calls.append(("start", stage)),
        start_base=lambda: calls.append(("start_base",)),
        start_mapping=lambda: calls.append(("start_mapping",)),
    )
    fake = SimpleNamespace(
        _persistent_runtime_enabled=persistent_runtime_enabled,
        _manager=manager,
        _readiness_generation=lambda: 7,
        _arm_readiness_profile=lambda profile: (
            calls.append(("arm", profile)) or 7
        ),
        _wait_for_new_ready=lambda generation: calls.append(
            ("ready", generation)
        ),
        _wait_persistent_velocity_pipeline_ready=lambda: calls.append(
            ("velocity_pipeline_ready",)
        ),
        _wait_persistent_base_control_ready=lambda: calls.append(
            ("base_control_ready",)
        ),
        _wait_persistent_stage_executor_ready=lambda: calls.append(
            ("stage_executor_ready",)
        ),
        transition=lambda phase, **_kwargs: calls.append(
            ("transition", phase)
        ),
    )

    SessionOrchestratorNode._start_mapping(fake)

    runtime_calls = [
        call
        for call in calls
        if call[0]
        in {
            "start",
            "start_base",
            "base_control_ready",
            "start_mapping",
            "stage_executor_ready",
        }
    ]
    assert runtime_calls == expected_calls
    assert ("ready", 7) in calls
    if persistent_runtime_enabled:
        assert ("velocity_pipeline_ready",) in calls
        assert calls.index(("ready", 7)) < calls.index(
            ("velocity_pipeline_ready",)
        )
        assert calls.index(("velocity_pipeline_ready",)) < calls.index(
            ("transition", SessionPhase.MAPPING)
        )
        assert calls.index(("arm", "persistent_mapping_stage")) < calls.index(
            ("start_base",)
        )
        assert calls.index(("start_base",)) < calls.index(
            ("base_control_ready",)
        )
        assert calls.index(("base_control_ready",)) < calls.index(
            ("start_mapping",)
        )
        assert calls.index(("start_mapping",)) < calls.index(
            ("stage_executor_ready",)
        )
        assert calls.index(("stage_executor_ready",)) < calls.index(
            ("ready", 7)
        )
    else:
        assert not any(call[0] == "arm" for call in calls)


class _ActiveLifecycleClient:
    def wait_for_service(self, *, timeout_sec):
        del timeout_sec
        return True

    def call(self, _request, *, timeout_sec):
        del timeout_sec
        return SimpleNamespace(
            current_state=SimpleNamespace(id=State.PRIMARY_STATE_ACTIVE)
        )


class _LifecycleStateSequenceClient:
    def __init__(self, states):
        self._states = iter(states)

    def wait_for_service(self, *, timeout_sec):
        del timeout_sec
        return True

    def call(self, _request, *, timeout_sec):
        del timeout_sec
        return SimpleNamespace(
            current_state=SimpleNamespace(id=next(self._states))
        )


class _RecordingLifecycleChangeClient:
    def __init__(self, responses=None):
        self.transitions = []
        self._responses = iter(responses) if responses is not None else None

    def wait_for_service(self, *, timeout_sec):
        del timeout_sec
        return True

    def call(self, request, *, timeout_sec):
        del timeout_sec
        assert isinstance(request, ChangeState.Request)
        self.transitions.append(request.transition.id)
        if self._responses is not None:
            return next(self._responses)
        return SimpleNamespace(success=True)


def test_persistent_base_control_must_be_active_before_stage_start():
    fake = SimpleNamespace(
        _dry_run=False,
        _startup_timeout_s=0.2,
        _typed_action_bridge_state_client=_ActiveLifecycleClient(),
        _typed_action_bridge_change_client=_RecordingLifecycleChangeClient(),
        _manager=SimpleNamespace(unexpected_exit=lambda: (None, None)),
    )

    SessionOrchestratorNode._wait_persistent_base_control_ready(fake)


def test_persistent_base_control_configures_and_activates_bridge():
    change_client = _RecordingLifecycleChangeClient()
    fake = SimpleNamespace(
        _dry_run=False,
        _startup_timeout_s=0.2,
        _typed_action_bridge_state_client=_LifecycleStateSequenceClient(
            (
                State.PRIMARY_STATE_UNCONFIGURED,
                State.PRIMARY_STATE_INACTIVE,
                State.PRIMARY_STATE_ACTIVE,
            )
        ),
        _typed_action_bridge_change_client=change_client,
        _manager=SimpleNamespace(unexpected_exit=lambda: (None, None)),
    )

    SessionOrchestratorNode._wait_persistent_base_control_ready(fake)

    assert change_client.transitions == [
        Transition.TRANSITION_CONFIGURE,
        Transition.TRANSITION_ACTIVATE,
    ]


def test_persistent_base_control_recovers_when_transition_response_is_lost():
    change_client = _RecordingLifecycleChangeClient(
        responses=(None, SimpleNamespace(success=True))
    )
    fake = SimpleNamespace(
        _dry_run=False,
        _startup_timeout_s=0.2,
        _typed_action_bridge_state_client=_LifecycleStateSequenceClient(
            (
                State.PRIMARY_STATE_UNCONFIGURED,
                State.PRIMARY_STATE_INACTIVE,
                State.PRIMARY_STATE_ACTIVE,
            )
        ),
        _typed_action_bridge_change_client=change_client,
        _manager=SimpleNamespace(unexpected_exit=lambda: (None, None)),
    )

    SessionOrchestratorNode._wait_persistent_base_control_ready(fake)

    assert change_client.transitions == [
        Transition.TRANSITION_CONFIGURE,
        Transition.TRANSITION_ACTIVATE,
    ]


def test_persistent_base_control_times_out_when_lost_transition_never_applies():
    class _UnconfiguredLifecycleClient(_ActiveLifecycleClient):
        def call(self, _request, *, timeout_sec):
            del timeout_sec
            return SimpleNamespace(
                current_state=SimpleNamespace(
                    id=State.PRIMARY_STATE_UNCONFIGURED
                )
            )

    fake = SimpleNamespace(
        _dry_run=False,
        _startup_timeout_s=0.02,
        _typed_action_bridge_state_client=_UnconfiguredLifecycleClient(),
        _typed_action_bridge_change_client=_RecordingLifecycleChangeClient(
            responses=iter(lambda: None, object())
        ),
        _manager=SimpleNamespace(unexpected_exit=lambda: (None, None)),
    )

    with pytest.raises(TimeoutError, match="did not become active"):
        SessionOrchestratorNode._wait_persistent_base_control_ready(fake)


def test_persistent_base_control_rechecks_base_liveness_before_commit():
    exits = iter(((None, None), ("base", 17)))
    fake = SimpleNamespace(
        _dry_run=False,
        _startup_timeout_s=0.2,
        _typed_action_bridge_state_client=_ActiveLifecycleClient(),
        _typed_action_bridge_change_client=_RecordingLifecycleChangeClient(),
        _manager=SimpleNamespace(unexpected_exit=lambda: next(exits)),
    )

    with pytest.raises(
        RuntimeError,
        match=(
            "persistent base process exited code=17 before typed Action "
            "bridge became active"
        ),
    ):
        SessionOrchestratorNode._wait_persistent_base_control_ready(fake)


def test_persistent_velocity_pipeline_requires_fresh_final_zero():
    condition = threading.Condition()
    fake = SimpleNamespace(
        _dry_run=False,
        _startup_timeout_s=0.5,
        _collision_monitor_state_client=_ActiveLifecycleClient(),
        _cmd_vel_condition=condition,
        _cmd_vel_generation=4,
        _last_cmd_vel=(0.0, 0.0),
        count_publishers=lambda _topic: 1,
    )

    def publish_fresh_zero():
        time.sleep(0.02)
        with condition:
            fake._cmd_vel_generation += 1
            fake._last_cmd_vel = (0.0, 0.0)
            condition.notify_all()

    worker = threading.Thread(target=publish_fresh_zero)
    worker.start()
    SessionOrchestratorNode._wait_persistent_velocity_pipeline_ready(fake)
    worker.join(timeout=1.0)

    assert not worker.is_alive()


def test_persistent_velocity_pipeline_rejects_multiple_final_publishers():
    fake = SimpleNamespace(
        _dry_run=False,
        _startup_timeout_s=0.02,
        _collision_monitor_state_client=_ActiveLifecycleClient(),
        _cmd_vel_condition=threading.Condition(),
        _cmd_vel_generation=1,
        _last_cmd_vel=(0.0, 0.0),
        count_publishers=lambda _topic: 2,
    )

    with pytest.raises(TimeoutError, match="publishers=2"):
        SessionOrchestratorNode._wait_persistent_velocity_pipeline_ready(fake)


def test_startup_failure_closes_runtime_and_releases_queued_action():
    queued = CommandRequest(SessionCommand.STOP_SESSION, "action")
    requests = queue.Queue(maxsize=2)
    requests.put_nowait(queued)
    calls = []
    fake = SimpleNamespace(
        _persistent_runtime_enabled=True,
        _stopping=threading.Event(),
        _state_lock=threading.RLock(),
        _mission_outcome=SlamSessionState.MISSION_IDLE,
        _mission_message="",
        _requests=requests,
        _manager=SimpleNamespace(
            shutdown=lambda: calls.append("shutdown"),
            stop=lambda: calls.append("stop"),
        ),
        _start_mapping=lambda: (_ for _ in ()).throw(
            RuntimeError("base launch failed")
        ),
        transition=lambda phase, **kwargs: calls.append(
            ("transition", phase, kwargs["detail"])
        ),
        _request_process_shutdown=lambda: calls.append("process_shutdown"),
        get_logger=lambda: SimpleNamespace(error=lambda message: calls.append(
            ("error", message)
        )),
    )

    SessionOrchestratorNode._worker_loop(fake)

    assert fake._stopping.is_set()
    assert fake._mission_outcome == SlamSessionState.MISSION_FAILED
    assert "base launch failed" in fake._mission_message
    assert queued.completed.is_set()
    assert queued.success is False
    assert "mapping startup failed" in queued.message
    assert calls[0] == (
        "transition",
        SessionPhase.FAILED,
        "mapping startup failed: base launch failed",
    )
    assert "shutdown" in calls
    assert "stop" not in calls
    assert calls[-1] == "process_shutdown"


def test_stopping_session_rejects_new_action_before_queueing():
    fake = SimpleNamespace(_stopping=threading.Event())
    fake._stopping.set()
    request = CommandRequest(SessionCommand.STOP_SESSION, "action")

    accepted = SessionOrchestratorNode._enqueue(fake, request)

    assert accepted is False
    assert request.completed.is_set()
    assert request.message == "session is stopping"


def test_persistent_readiness_requires_exact_armed_stage_profile():
    fake = SimpleNamespace(
        _ready_condition=threading.Condition(),
        _ready_generation=4,
        _latest_ready=True,
        _expected_readiness_profile="persistent_mapping_stage",
    )

    generation = SessionOrchestratorNode._arm_readiness_profile(
        fake, "persistent_navigation_stage"
    )
    assert generation == 4
    assert not fake._latest_ready

    # 旧 mapping aggregator 的 transient-local/heartbeat 不能为新 Nav2
    # stage 解锁；只有本次明确武装的 profile 才能推进 generation。
    SessionOrchestratorNode._on_readiness(
        fake,
        SimpleNamespace(profile="persistent_mapping_stage", ready=True),
    )
    assert fake._ready_generation == 4
    assert not fake._latest_ready

    SessionOrchestratorNode._on_readiness(
        fake,
        SimpleNamespace(profile="persistent_navigation_stage", ready=True),
    )
    assert fake._ready_generation == 5
    assert fake._latest_ready


def test_compatible_readiness_keeps_accepting_unscoped_profiles():
    fake = SimpleNamespace(
        _ready_condition=threading.Condition(),
        _ready_generation=2,
        _latest_ready=False,
        _expected_readiness_profile="",
    )

    SessionOrchestratorNode._on_readiness(
        fake, SimpleNamespace(profile="voice_nav2", ready=True)
    )

    assert fake._ready_generation == 3
    assert fake._latest_ready


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
        _wait_persistent_navigation_ready=lambda _request: events.append(
            ("nav_dependencies_ready",)
        ),
        _wait_persistent_stage_executor_ready=lambda: events.append(
            ("stage_executor_ready",)
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


def test_persistent_navigation_replaces_only_stage_and_keeps_base():
    events = []
    map_yaml = "/tmp/session-map.yaml"
    fake = SimpleNamespace(
        _persistent_runtime_enabled=True,
        _authority_gate_enabled=False,
        _fsm=SimpleNamespace(
            snapshot=SimpleNamespace(
                phase=SessionPhase.MAP_SAVED,
                map_yaml_path=map_yaml,
            )
        ),
        _map_pose_condition=threading.Condition(),
        _navigation_input_generation=2,
        _latest_occupancy=object(),
        _latest_map_pose_xy=(1.0, 2.0),
        _latest_occupancy_generation=2,
        _latest_map_pose_generation=2,
        _navigation_inputs_enabled=True,
        transition=lambda phase, **_kwargs: events.append(
            ("transition", phase)
        ),
        feedback=lambda _request, _progress: None,
        _readiness_generation=lambda: 4,
        _arm_readiness_profile=lambda profile: (
            events.append(("arm", profile)) or 4
        ),
        _wait_for_new_ready=lambda generation: events.append(
            ("ready", generation)
        ),
        _wait_persistent_navigation_ready=lambda _request: events.append(
            ("nav_dependencies_ready",)
        ),
        _wait_persistent_stage_executor_ready=lambda: events.append(
            ("stage_executor_ready",)
        ),
    )

    class _Manager:
        def stop_stage(self):
            events.append(
                (
                    "stop_stage",
                    fake._navigation_input_generation,
                    fake._navigation_inputs_enabled,
                )
            )

        def prepare_navigation(self, map_path):
            events.append(
                (
                    "prepare",
                    str(map_path),
                    fake._navigation_input_generation,
                    fake._latest_occupancy,
                    fake._latest_map_pose_xy,
                    fake._navigation_inputs_enabled,
                )
            )

        def start(self, stage):
            events.append(
                (
                    "start",
                    stage,
                    fake._navigation_inputs_enabled,
                )
            )

    fake._manager = _Manager()

    SessionOrchestratorNode.start_navigation(
        fake, SimpleNamespace(message="")
    )

    assert ("stop_stage", 2, True) in events
    assert ("prepare", map_yaml, 3, None, None, False) in events
    assert ("start", "navigation", True) in events
    assert events.index(("stop_stage", 2, True)) < events.index(
        ("prepare", map_yaml, 3, None, None, False)
    )
    assert events.index(
        ("prepare", map_yaml, 3, None, None, False)
    ) < events.index(("arm", "persistent_navigation_stage"))
    assert events.index(("arm", "persistent_navigation_stage")) < events.index(
        ("start", "navigation", True)
    )
    assert events.index(("start", "navigation", True)) < events.index(
        ("stage_executor_ready",)
    )
    assert events.index(("stage_executor_ready",)) < events.index(
        ("ready", 4)
    )
    assert ("ready", 4) in events
    assert events.index(("ready", 4)) < events.index(
        ("nav_dependencies_ready",)
    )
    assert fake._latest_occupancy is None
    assert fake._latest_map_pose_xy is None
    assert fake._navigation_inputs_enabled


def test_persistent_navigation_wraps_stage_replacement_in_authority_transaction():
    events = []
    identity = QuiescenceIdentity(33, 8)
    map_yaml = "/tmp/session-map.yaml"
    request = CommandRequest(SessionCommand.START_NAVIGATION, "test")
    fake = SimpleNamespace(
        _persistent_runtime_enabled=True,
        _authority_gate_enabled=True,
        _navigation_startup_failure_pending=False,
        _fsm=SimpleNamespace(
            snapshot=SimpleNamespace(
                phase=SessionPhase.MAP_SAVED,
                map_yaml_path=map_yaml,
            )
        ),
        _map_pose_condition=threading.Condition(),
        _navigation_input_generation=0,
        _latest_occupancy=None,
        _latest_map_pose_xy=None,
        _latest_occupancy_generation=-1,
        _latest_map_pose_generation=-1,
        _navigation_inputs_enabled=True,
        transition=lambda phase, **_kwargs: events.append(
            ("transition", phase)
        ),
        feedback=lambda _request, _progress: None,
        _readiness_generation=lambda: 1,
        _arm_readiness_profile=lambda profile: (
            events.append(("arm", profile)) or 1
        ),
        _wait_for_new_ready=lambda _generation: events.append(("ready",)),
        _wait_persistent_navigation_ready=lambda _request: events.append(
            ("nav_dependencies_ready",)
        ),
        _wait_persistent_stage_executor_ready=lambda: events.append(
            ("stage_executor_ready",)
        ),
        _enter_internal_stage_switch_hold=lambda observed_request: (
            events.append(("hold_and_ack", observed_request)) or identity
        ),
        _resume_internal_stage_switch=lambda observed_identity: events.append(
            ("resume", observed_identity)
        ),
        _clear_internal_stage_switch=lambda: events.append(("clear",)),
    )
    fake._manager = SimpleNamespace(
        stop_stage=lambda: events.append(("stop_stage",)),
        prepare_navigation=lambda path: events.append(
            ("prepare", str(path))
        ),
        start=lambda stage: events.append(("start", stage)),
    )

    SessionOrchestratorNode.start_navigation(fake, request)

    assert events.index(("hold_and_ack", request)) < events.index(
        ("stop_stage",)
    )
    assert events.index(("stop_stage",)) < events.index(
        ("prepare", map_yaml)
    )
    assert events.index(("prepare", map_yaml)) < events.index(
        ("arm", "persistent_navigation_stage")
    )
    assert events.index(("arm", "persistent_navigation_stage")) < events.index(
        ("start", "navigation")
    )
    assert events.index(("start", "navigation")) < events.index(
        ("stage_executor_ready",)
    )
    assert events.index(("stage_executor_ready",)) < events.index(("ready",))
    assert events.index(("ready",)) < events.index(
        ("nav_dependencies_ready",)
    )
    assert events.index(("nav_dependencies_ready",)) < events.index(
        ("resume", identity)
    )
    assert events.index(("resume", identity)) < events.index(("clear",))
    assert events.index(("clear",)) < events.index(
        ("transition", SessionPhase.NAVIGATING)
    )


def test_external_takeover_during_stage_switch_never_auto_resumes():
    events = []
    identity = QuiescenceIdentity(33, 8)
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )

    def readiness_then_keyboard_takeover(_generation):
        events.append(("ready",))
        # 这代表真实 callback 已把用户 KEYBOARD/ESTOP 记录到外层 request。
        request.canceled = True

    fake = SimpleNamespace(
        _persistent_runtime_enabled=True,
        _authority_gate_enabled=True,
        _navigation_startup_failure_pending=False,
        _fsm=SimpleNamespace(
            snapshot=SimpleNamespace(
                phase=SessionPhase.MAP_SAVED,
                map_yaml_path="/tmp/session-map.yaml",
            )
        ),
        _map_pose_condition=threading.Condition(),
        _navigation_input_generation=0,
        _latest_occupancy=None,
        _latest_map_pose_xy=None,
        _latest_occupancy_generation=-1,
        _latest_map_pose_generation=-1,
        _navigation_inputs_enabled=True,
        transition=lambda phase, **_kwargs: events.append(
            ("transition", phase)
        ),
        feedback=lambda _request, _progress: None,
        _readiness_generation=lambda: 1,
        _arm_readiness_profile=lambda profile: (
            events.append(("arm", profile)) or 1
        ),
        _wait_for_new_ready=readiness_then_keyboard_takeover,
        _wait_persistent_navigation_ready=lambda _request: events.append(
            ("unexpected_nav_ready_wait",)
        ),
        _wait_persistent_stage_executor_ready=lambda: events.append(
            ("stage_executor_ready",)
        ),
        _enter_internal_stage_switch_hold=lambda _request: identity,
        _resume_internal_stage_switch=lambda _identity: events.append(
            ("unexpected_resume",)
        ),
        _clear_internal_stage_switch=lambda: events.append(("clear",)),
    )
    fake._manager = SimpleNamespace(
        stop_stage=lambda: events.append(("stop_stage",)),
        prepare_navigation=lambda _path: events.append(("prepare",)),
        start=lambda _stage: events.append(("start",)),
    )

    with pytest.raises(
        RuntimeError, match="mapping stage was replaced"
    ):
        SessionOrchestratorNode.start_navigation(fake, request)

    assert ("unexpected_resume",) not in events
    assert ("unexpected_nav_ready_wait",) not in events
    assert events.count(("stop_stage",)) == 2
    assert events.index(("ready",)) < len(events) - 2
    assert events[-2] == ("stop_stage",)
    assert events[-1] == ("clear",)
    assert fake._navigation_startup_failure_pending


def test_internal_quiescence_failure_marks_navigation_startup_failed():
    barrier = AutonomyQuiescenceBarrier()
    barrier.fail("fresh zero missing")
    fake = SimpleNamespace(
        _persistent_runtime_enabled=True,
        _authority_gate_enabled=True,
        _navigation_startup_failure_pending=False,
        _autonomy_quiescence=barrier,
        _map_pose_condition=threading.Condition(),
        _navigation_inputs_enabled=False,
        _fsm=SimpleNamespace(
            snapshot=SimpleNamespace(
                phase=SessionPhase.MAP_SAVED,
                map_yaml_path="/tmp/session-map.yaml",
            )
        ),
        transition=lambda _phase, **_kwargs: None,
        feedback=lambda _request, _progress: None,
        _enter_internal_stage_switch_hold=lambda _request: (
            (_ for _ in ()).throw(QuiescenceError("fresh zero missing"))
        ),
        _clear_internal_stage_switch=lambda: None,
        _manager=SimpleNamespace(
            stop_stage=lambda: pytest.fail(
                "mapping stage was not replaced"
            )
        ),
    )

    with pytest.raises(QuiescenceError, match="fresh zero missing"):
        SessionOrchestratorNode.start_navigation(
            fake,
            CommandRequest(SessionCommand.START_NAVIGATION, "test"),
        )

    assert fake._navigation_startup_failure_pending


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


def test_base_runtime_failure_cannot_recover_to_mapping():
    manager = SimpleNamespace(stage="mapping")

    def base_crashes(node, request):
        node._child_runtime_failure = "base process exited code=23"
        node.transition(
            SessionPhase.FAILED,
            detail=node._child_runtime_failure,
        )
        request.canceled = True
        raise AutomaticMissionCancelled("automatic mission canceled")

    fake, transitions = _automatic_navigation_failure_fixture(
        manager, base_crashes
    )
    request = CommandRequest(
        command=SessionCommand.RUN_AUTOMATIC_MISSION,
        source="test",
    )

    SessionOrchestratorNode._execute_request(fake, request)

    assert fake._fsm.snapshot.phase == SessionPhase.FAILED
    assert transitions[-1] == SessionPhase.FAILED
    assert SessionPhase.MAPPING not in transitions
    assert request.message == "base process exited code=23"
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


def test_persistent_navigation_readiness_requires_stage_lifecycle_and_fresh_inputs(
    monkeypatch,
):
    lifecycle_checks = []

    class _LifecycleClient:
        def __init__(self, name):
            self.name = name

        def wait_for_service(self, timeout_sec):
            lifecycle_checks.append((self.name, timeout_sec))
            return True

    monkeypatch.setattr(
        showcase_session_node_module,
        "_get_lifecycle_state",
        lambda client, _timeout: (
            lifecycle_checks.append((client.name, "active"))
            or State.PRIMARY_STATE_ACTIVE
        ),
    )
    fake = SimpleNamespace(
        _persistent_runtime_enabled=True,
        _dry_run=False,
        _startup_timeout_s=0.2,
        _navigate_to_pose_client=SimpleNamespace(
            server_is_ready=lambda: True
        ),
        _follow_waypoints_client=SimpleNamespace(
            server_is_ready=lambda: True
        ),
        _bt_navigator_state_client=_LifecycleClient("bt_navigator"),
        _waypoint_follower_state_client=_LifecycleClient(
            "waypoint_follower"
        ),
        _map_server_state_client=_LifecycleClient("map_server"),
        _amcl_state_client=_LifecycleClient("amcl"),
        _map_pose_condition=threading.Condition(),
        _navigation_input_generation=6,
        _latest_occupancy=object(),
        _latest_map_pose_xy=(0.0, 0.0),
        _latest_occupancy_generation=6,
        _latest_map_pose_generation=6,
        _cancel_automatic_motion=lambda: None,
        _seed_persistent_amcl_pose=lambda _request, _deadline: (
            lifecycle_checks.append(("initial_pose", "seeded"))
        ),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )

    SessionOrchestratorNode._wait_navigation_ready_impl(
        fake, CommandRequest(SessionCommand.START_NAVIGATION, "test")
    )

    active_names = {
        name
        for name, state in lifecycle_checks
        if state == "active"
    }
    assert active_names == {
        "bt_navigator",
        "waypoint_follower",
        "map_server",
        "amcl",
    }
    assert lifecycle_checks.index(("amcl", "active")) < lifecycle_checks.index(
        ("initial_pose", "seeded")
    )


def test_persistent_navigation_readiness_rejects_stale_map_and_pose(
    monkeypatch,
):
    monkeypatch.setattr(
        showcase_session_node_module,
        "_get_lifecycle_state",
        lambda _client, _timeout: State.PRIMARY_STATE_ACTIVE,
    )
    action_client = SimpleNamespace(server_is_ready=lambda: True)
    lifecycle_client = SimpleNamespace(
        wait_for_service=lambda timeout_sec: True
    )
    fake = SimpleNamespace(
        _persistent_runtime_enabled=True,
        _dry_run=False,
        _startup_timeout_s=0.01,
        _navigate_to_pose_client=action_client,
        _follow_waypoints_client=action_client,
        _bt_navigator_state_client=lifecycle_client,
        _waypoint_follower_state_client=lifecycle_client,
        _map_server_state_client=lifecycle_client,
        _amcl_state_client=lifecycle_client,
        _map_pose_condition=threading.Condition(),
        _navigation_input_generation=8,
        _latest_occupancy=object(),
        _latest_map_pose_xy=(0.0, 0.0),
        _latest_occupancy_generation=7,
        _latest_map_pose_generation=7,
        _cancel_automatic_motion=lambda: None,
        _seed_persistent_amcl_pose=lambda _request, _deadline: None,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )

    with pytest.raises(
        TimeoutError, match="fresh /map and /amcl_pose"
    ):
        SessionOrchestratorNode._wait_navigation_ready_impl(
            fake,
            CommandRequest(SessionCommand.START_NAVIGATION, "test"),
        )


def _return_pose_evidence(x=0.18, y=-0.07, yaw=0.12):
    pose = PlanarPose(
        x=x,
        y=y,
        yaw=yaw,
        frame_id="map",
        observed_at_ns=11,
    )
    return SimpleNamespace(final_pose=pose, map_saved_at_ns=12)


def test_persistent_initial_pose_never_blind_publishes_without_subscriber():
    published = []
    fake = SimpleNamespace(
        _initial_pose_pub=SimpleNamespace(
            get_subscription_count=lambda: 0,
            publish=published.append,
        ),
        _manager=SimpleNamespace(unexpected_exit=lambda: (None, None)),
        _map_pose_condition=threading.Condition(),
        _navigation_input_generation=3,
        _latest_map_pose_xy=None,
        _latest_map_pose_generation=-1,
        _return_to_start_evidence=_return_pose_evidence(),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time())
        ),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
        _cancel_automatic_motion=lambda: None,
    )

    with pytest.raises(
        TimeoutError, match="/initialpose subscriber did not match"
    ):
        SessionOrchestratorNode._seed_persistent_amcl_pose(
            fake,
            CommandRequest(SessionCommand.START_NAVIGATION, "test"),
            time.monotonic() + 0.02,
        )

    assert published == []


def test_persistent_initial_pose_retries_until_current_generation_pose():
    published = []
    condition = threading.Condition()
    fake = SimpleNamespace(
        _manager=SimpleNamespace(unexpected_exit=lambda: (None, None)),
        _map_pose_condition=condition,
        _navigation_input_generation=8,
        _latest_map_pose_xy=None,
        _latest_map_pose_generation=-1,
        _return_to_start_evidence=_return_pose_evidence(),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=4))
        ),
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
        _cancel_automatic_motion=lambda: None,
    )

    def publish(message):
        published.append(message)
        with condition:
            fake._latest_map_pose_xy = (
                message.pose.pose.position.x,
                message.pose.pose.position.y,
            )
            fake._latest_map_pose_generation = 8
            condition.notify_all()

    fake._initial_pose_pub = SimpleNamespace(
        get_subscription_count=lambda: 1,
        publish=publish,
    )

    SessionOrchestratorNode._seed_persistent_amcl_pose(
        fake,
        CommandRequest(SessionCommand.START_NAVIGATION, "test"),
        time.monotonic() + 0.2,
    )

    assert len(published) == 1
    message = published[0]
    assert message.header.frame_id == "map"
    assert message.header.stamp.sec == 4
    assert message.pose.pose.position.x == pytest.approx(0.18)
    assert message.pose.pose.position.y == pytest.approx(-0.07)
    assert message.pose.pose.orientation.z == pytest.approx(
        math.sin(0.12 / 2.0)
    )
    assert message.pose.pose.orientation.w == pytest.approx(
        math.cos(0.12 / 2.0)
    )


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


@pytest.mark.parametrize(
    ("persistent_runtime_enabled", "expected_call"),
    [(False, "stop"), (True, "stop_stage")],
)
def test_navigation_start_failure_preserves_base_only_in_persistent_mode(
    persistent_runtime_enabled,
    expected_call,
):
    calls = []
    manager = SimpleNamespace(
        stop=lambda: calls.append("stop"),
        stop_stage=lambda: calls.append("stop_stage"),
    )
    fake = SimpleNamespace(
        _persistent_runtime_enabled=persistent_runtime_enabled,
        _navigation_startup_failure_pending=False,
        _manager=manager,
        get_logger=lambda: SimpleNamespace(error=lambda _message: None),
    )

    SessionOrchestratorNode._cleanup_failed_navigation_startup(
        fake, RuntimeError("readiness failed")
    )

    assert calls == [expected_call]
    assert fake._navigation_startup_failure_pending


@pytest.mark.parametrize(
    ("persistent_runtime_enabled", "expected_call"),
    [(False, "stop"), (True, "shutdown")],
)
def test_stop_session_releases_the_runtime_owned_by_selected_mode(
    persistent_runtime_enabled,
    expected_call,
):
    calls = []
    fake = SimpleNamespace(
        _persistent_runtime_enabled=persistent_runtime_enabled,
        _state_lock=threading.RLock(),
        _fsm=SimpleNamespace(validate=lambda _command: (True, "")),
        _operation_active=threading.Event(),
        _active_request=None,
        _manager=SimpleNamespace(
            stop=lambda: calls.append("stop"),
            shutdown=lambda: calls.append("shutdown"),
        ),
        transition=lambda phase, **_kwargs: calls.append(
            ("transition", phase)
        ),
        get_logger=lambda: SimpleNamespace(
            warning=lambda _message: None,
            error=lambda _message: None,
        ),
    )
    request = CommandRequest(SessionCommand.STOP_SESSION, "test")

    SessionOrchestratorNode._execute_request(fake, request)

    assert expected_call in calls
    assert ("shutdown" if expected_call == "stop" else "stop") not in calls
    assert request.success
    assert request.completed.is_set()


@pytest.mark.parametrize(
    ("persistent_runtime_enabled", "expected_call"),
    [(False, "stop"), (True, "shutdown")],
)
def test_close_releases_the_runtime_owned_by_selected_mode(
    persistent_runtime_enabled,
    expected_call,
):
    calls = []
    active_request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )

    def shutdown():
        calls.append("shutdown")
        assert active_request.canceled

    fake = SimpleNamespace(
        _persistent_runtime_enabled=persistent_runtime_enabled,
        _stopping=threading.Event(),
        _state_lock=threading.RLock(),
        _active_request=active_request,
        _requests=queue.Queue(maxsize=1),
        _manager=SimpleNamespace(
            stop=lambda: calls.append("stop"),
            shutdown=shutdown,
        ),
        _worker=SimpleNamespace(join=lambda timeout: calls.append(("join", timeout))),
        _action_server=SimpleNamespace(
            destroy=lambda: calls.append(("destroy",))
        ),
    )

    SessionOrchestratorNode.close(fake)

    assert calls[0] == expected_call
    assert ("shutdown" if expected_call == "stop" else "stop") not in calls
    assert ("join", 3.0) in calls
    assert ("destroy",) in calls
    assert active_request.canceled


@pytest.mark.parametrize("role", ["base", "stage", "explorer"])
def test_persistent_child_watchdog_reports_the_failed_owner(role):
    transitions = []
    manager = SimpleNamespace(
        unexpected_exit=lambda: (role, 17),
        exited_unexpectedly=lambda: (_ for _ in ()).throw(
            AssertionError("persistent watchdog must inspect every owner")
        ),
        stage="navigation",
    )
    fake = SimpleNamespace(
        _persistent_runtime_enabled=True,
        _operation_active=threading.Event(),
        _stopping=threading.Event(),
        _state_lock=threading.RLock(),
        _active_request=None,
        _manager=manager,
        _fsm=SimpleNamespace(
            snapshot=SimpleNamespace(phase=SessionPhase.NAVIGATING)
        ),
        transition=lambda phase, **kwargs: transitions.append(
            (phase, kwargs["detail"])
        ),
    )

    SessionOrchestratorNode._check_child_process(fake)

    assert transitions == [
        (SessionPhase.FAILED, f"{role} process exited code=17")
    ]


def test_persistent_watchdog_reports_base_crash_during_active_operation():
    transitions = []
    request = CommandRequest(
        SessionCommand.RUN_AUTOMATIC_MISSION, "test"
    )
    operation_active = threading.Event()
    operation_active.set()
    fake = SimpleNamespace(
        _persistent_runtime_enabled=True,
        _operation_active=operation_active,
        _stopping=threading.Event(),
        _state_lock=threading.RLock(),
        _active_request=request,
        _manager=SimpleNamespace(
            unexpected_exit=lambda: ("base", 23),
            stage="mapping",
        ),
        _fsm=SimpleNamespace(
            snapshot=SimpleNamespace(phase=SessionPhase.MAPPING)
        ),
        transition=lambda phase, **kwargs: transitions.append(
            (phase, kwargs["detail"])
        ),
    )

    SessionOrchestratorNode._check_child_process(fake)

    assert request.canceled
    assert transitions == [
        (SessionPhase.FAILED, "base process exited code=23")
    ]


def test_persistent_watchdog_ignores_expected_stage_exit_during_operation():
    operation_active = threading.Event()
    operation_active.set()
    fake = SimpleNamespace(
        _persistent_runtime_enabled=True,
        _operation_active=operation_active,
        _stopping=threading.Event(),
        _manager=SimpleNamespace(
            unexpected_exit=lambda: ("stage", 0),
            stage="",
        ),
        _fsm=SimpleNamespace(
            snapshot=SimpleNamespace(phase=SessionPhase.SWITCHING_TO_NAVIGATION)
        ),
        transition=lambda *_args, **_kwargs: pytest.fail(
            "intentional stage replacement is not a runtime failure"
        ),
    )

    SessionOrchestratorNode._check_child_process(fake)


def test_compatible_child_watchdog_keeps_the_legacy_stage_contract():
    transitions = []
    manager = SimpleNamespace(
        exited_unexpectedly=lambda: (True, 9),
        unexpected_exit=lambda: (_ for _ in ()).throw(
            AssertionError("legacy mode must not require persistent owners")
        ),
        stage="mapping",
    )
    fake = SimpleNamespace(
        _persistent_runtime_enabled=False,
        _operation_active=threading.Event(),
        _stopping=threading.Event(),
        _manager=manager,
        _fsm=SimpleNamespace(
            snapshot=SimpleNamespace(phase=SessionPhase.MAPPING)
        ),
        transition=lambda phase, **kwargs: transitions.append(
            (phase, kwargs["detail"])
        ),
    )

    SessionOrchestratorNode._check_child_process(fake)

    assert transitions == [
        (SessionPhase.FAILED, "mapping process exited code=9")
    ]


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


def test_recovery_backup_delegates_transaction_and_measures_fresh_odom():
    """Node 只负责恢复动作前后的里程计取证，Action 生命周期归事务模块。"""

    evidence = MappingEvidenceTracker(1)
    evidence.record_odom(1.0, 2.0)
    transaction_calls = []

    class _Motion:
        @staticmethod
        def execute(intent, *, is_cancelled, deadline_monotonic):
            transaction_calls.append(
                (intent, is_cancelled(), deadline_monotonic)
            )
            # 模拟事务成功后到达的新一代 odom；不污染 mapping path 统计。
            evidence.record_odom(0.76, 2.0)

    class _Runtime:
        _dry_run = False
        _mapping_evidence = evidence
        _nav2_motion = _Motion()

        def _wait_for_recovery_odom(self, *args, **kwargs):
            return SessionOrchestratorNode._wait_for_recovery_odom(
                self, *args, **kwargs
            )

        @staticmethod
        def get_logger():
            return SimpleNamespace(info=lambda _message: None)

    request = SimpleNamespace(canceled=False)
    displacement = SessionOrchestratorNode.run_recovery_backup(
        _Runtime(),
        request,
        distance_m=0.30,
        speed_mps=0.08,
        timeout_s=10.0,
    )

    assert displacement == pytest.approx(0.24)
    assert evidence.mapping_path_m == 0.0
    assert len(transaction_calls) == 1
    intent, canceled, deadline = transaction_calls[0]
    assert intent == RecoveryBackup(
        distance_m=0.30,
        speed_mps=0.08,
        time_allowance_s=10.0,
    )
    assert canceled is False
    assert deadline > time.monotonic()


def test_navigation_goal_preflights_records_then_delegates_transaction(
    monkeypatch,
):
    """Node 保留安全准入 seam，accepted 到 terminal 不再维护第二套状态机。"""

    monkeypatch.setattr(
        showcase_session_node_module.time,
        "monotonic",
        lambda: 100.0,
    )
    events = []
    preflight_path = object()
    request = SimpleNamespace(canceled=False)

    class _Motion:
        @staticmethod
        def execute(intent, *, is_cancelled, deadline_monotonic):
            events.append(
                ("execute", intent, is_cancelled(), deadline_monotonic)
            )

    fake = SimpleNamespace(
        _dry_run=False,
        _nav2_motion=_Motion(),
        _set_navigation_goal_status=lambda sequence, status, **kwargs: (
            events.append(("status", sequence, status, kwargs))
        ),
        _request_preflight_path=lambda actual_request, *, goal_xy, deadline: (
            events.append(
                ("preflight", actual_request, goal_xy, deadline)
            )
            or preflight_path
        ),
        _record_navigation_plan=lambda sequence, path: (
            events.append(("record_plan", sequence, path)) or True
        ),
    )

    SessionOrchestratorNode.run_navigation_goal(
        fake,
        request,
        sequence=3,
        goal_xy=(1.25, -0.75),
        timeout_s=5.0,
    )

    assert events == [
        (
            "status",
            3,
            NavigationGoalStatus.REQUESTED,
            {"detail": "preflight path requested"},
        ),
        ("preflight", request, (1.25, -0.75), 105.0),
        ("record_plan", 3, preflight_path),
        (
            "execute",
            SampledNavigate(sequence=3, goal_xy=(1.25, -0.75)),
            False,
            105.0,
        ),
    ]


def test_mapping_return_preflights_then_delegates_and_verifies_pose(
    monkeypatch,
):
    """返航的 ROS 取证留在 Node，NavigateToPose 生命周期委托给事务模块。"""

    monkeypatch.setattr(
        showcase_session_node_module.time,
        "monotonic",
        lambda: 100.0,
    )
    monkeypatch.setattr(
        showcase_session_node_module.time,
        "sleep",
        lambda _seconds: None,
    )
    start_pose = PlanarPose(1.0, -0.5, 0.25, "map", 10)
    final_pose = PlanarPose(1.01, -0.49, 0.24, "map", 20)
    request = SimpleNamespace(canceled=False)
    events = []

    class _Motion:
        @staticmethod
        def execute(intent, *, is_cancelled, deadline_monotonic):
            events.append(
                ("execute", intent, is_cancelled(), deadline_monotonic)
            )

    fake = SimpleNamespace(
        _dry_run=False,
        _mission_sequence=4,
        _nav2_motion=_Motion(),
        _request_preflight_path=(
            lambda actual_request, *, goal_xy, deadline: events.append(
                ("preflight", actual_request, goal_xy, deadline)
            )
        ),
        _lookup_mapping_pose=(
            lambda actual_request, *, timeout_s: final_pose
        ),
        stop_motion_and_wait=(
            lambda actual_request, *, timeout_s: events.append(
                ("stop", actual_request.source, timeout_s)
            )
        ),
        _cmd_vel_condition=threading.Condition(),
        _last_cmd_vel=(0.0, 0.0),
        _last_cmd_vel_observed_at_ns=30,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=25)
        ),
    )

    evidence = SessionOrchestratorNode.run_mapping_return_goal(
        fake,
        request,
        start_pose=start_pose,
        spec=ReturnToStartSpec(),
        timeout_s=5.0,
    )

    assert events[0] == ("preflight", request, (1.0, -0.5), 105.0)
    assert events[1] == (
        "execute",
        MappingReturn(goal_pose=start_pose),
        False,
        105.0,
    )
    assert events[-1][0:2] == ("stop", "navigation_safety_stop")
    assert evidence.start_pose == start_pose
    assert evidence.final_pose == final_pose
    assert evidence.action_status is ReturnActionStatus.SUCCEEDED
    assert evidence.cmd_vel_linear_x == 0.0
    assert evidence.cmd_vel_angular_z == 0.0


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
