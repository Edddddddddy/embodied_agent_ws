"""Nav2MotionTransaction 只通过 execute Interface 验证安全事务。"""

from __future__ import annotations

from collections import defaultdict
import threading

import pytest

from embodied_slam_tools.mapping_evidence import (
    NavigationGoalLedger,
    NavigationGoalStatus,
)
from embodied_slam_tools.mapping_return import PlanarPose
from embodied_slam_tools.mission_executor import AutomaticMissionCancelled
from embodied_slam_tools.nav2_motion_transaction import (
    ActionStatus,
    ActionTerminal,
    BackupGoal,
    MappingReturn,
    Nav2ActionKind,
    Nav2MotionFailed,
    Nav2MotionTransaction,
    Nav2GoalRejected,
    Nav2SafetyFailure,
    Nav2TransactionBusy,
    NavigateGoal,
    RecoveryBackup,
    SampledNavigate,
)


class _DoneFuture:
    def __init__(self, value):
        self._value = value

    @staticmethod
    def done() -> bool:
        return True

    def result(self):
        return self._value


class _DelayedFuture:
    def __init__(self, value, *, pending_polls: int):
        self._value = value
        self._pending_polls = pending_polls
        self.poll_count = 0

    def done(self) -> bool:
        self.poll_count += 1
        return self.poll_count > self._pending_polls

    def result(self):
        return self._value


class _DelayedRaisingResponse(_DelayedFuture):
    def result(self):
        raise RuntimeError("late response payload unavailable")


class _NeverFuture:
    @staticmethod
    def done() -> bool:
        return False

    @staticmethod
    def result():
        raise AssertionError("a never-completing future has no result")


class _RaisingResultFuture:
    def __init__(self, message: str):
        self._message = message

    @staticmethod
    def done() -> bool:
        return True

    def result(self):
        raise RuntimeError(self._message)


class _RaisingDoneFuture:
    @staticmethod
    def done() -> bool:
        raise RuntimeError("response readiness unavailable")

    @staticmethod
    def result():
        raise AssertionError("unreadable response must not be consumed")


class _EventResultFuture:
    def __init__(self, release: threading.Event):
        self._release = release

    def done(self) -> bool:
        return self._release.is_set()

    def result(self):
        assert self._release.is_set()
        return ActionTerminal(
            status=ActionStatus.SUCCEEDED,
            error_code=0,
        )


class _CancelCompletesResult:
    def __init__(self, *, polls_after_cancel: int = 2):
        self.cancel_requested = False
        self.terminal = False
        self._polls_after_cancel = polls_after_cancel
        self._poll_count = 0

    def done(self) -> bool:
        if self.cancel_requested and not self.terminal:
            self._poll_count += 1
            if self._poll_count >= self._polls_after_cancel:
                self.terminal = True
        return self.terminal

    def result(self):
        if not self.terminal:
            raise AssertionError("result was read before terminal")
        return ActionTerminal(
            status=ActionStatus.CANCELED,
            error_code=0,
        )


class _Handle:
    def __init__(self, runtime, result_future, *, accepted: bool = True):
        self.accepted = accepted
        self._runtime = runtime
        self._result_future = result_future
        self.cancel_count = 0

    def get_result_async(self):
        return self._result_future

    def cancel_goal_async(self):
        self.cancel_count += 1
        self._runtime.events.append("cancel")
        if hasattr(self._result_future, "cancel_requested"):
            self._result_future.cancel_requested = True
        return _DoneFuture(object())


class _HandleWithoutResultFuture(_Handle):
    def get_result_async(self):
        raise RuntimeError("result channel unavailable")


class _HandleWithUnreadableAcceptance:
    @property
    def accepted(self):
        raise RuntimeError("acceptance channel unavailable")


class _RuntimeAdapter:
    """脚本化的 in-memory Adapter；不让测试接触事务 private 方法。"""

    def __init__(self, ledger: NavigationGoalLedger):
        self.ledger = ledger
        self.now = 0.0
        self.evidence_ns = 100
        self.events: list[str] = []
        self.sent_goals = []
        self.stop_timeouts: list[float] = []
        self.status_snapshots: list[NavigationGoalStatus] = []
        self.responses = defaultdict(list)
        self.send_error: BaseException | None = None
        self.priority_stop_error: BaseException | None = None
        self.stage_stop_error: BaseException | None = None
        self.evidence_error: BaseException | None = None
        self.evidence_error_statuses: set[NavigationGoalStatus] | None = None
        self.sleep_hook = None
        self.available = {
            Nav2ActionKind.NAVIGATE_TO_POSE: True,
            Nav2ActionKind.BACK_UP: True,
        }

    def enqueue(self, kind: Nav2ActionKind, response) -> None:
        self.responses[kind].append(response)

    def monotonic(self) -> float:
        return self.now

    def evidence_timestamp_ns(self) -> int:
        self.evidence_ns += 1
        return self.evidence_ns

    def sleep(self, seconds: float) -> None:
        self.now += seconds
        if self.sleep_hook is not None:
            self.sleep_hook()

    def wait_for_server(
        self,
        kind: Nav2ActionKind,
        *,
        timeout_s: float,
    ) -> bool:
        assert timeout_s >= 0.0
        self.events.append(f"wait:{kind.value}")
        return self.available[kind]

    def send_goal(self, goal):
        if self.send_error is not None:
            raise self.send_error
        kind = (
            Nav2ActionKind.BACK_UP
            if isinstance(goal, BackupGoal)
            else Nav2ActionKind.NAVIGATE_TO_POSE
        )
        self.sent_goals.append(goal)
        self.events.append(f"send:{kind.value}")
        return self.responses[kind].pop(0)

    def force_priority_stop(self, *, timeout_s: float) -> None:
        assert timeout_s >= 0.0
        self.stop_timeouts.append(timeout_s)
        self.events.append("priority_stop")
        if self.priority_stop_error is not None:
            raise self.priority_stop_error

    def stop_navigation_stage(self) -> None:
        self.events.append("stage_stop")
        if self.stage_stop_error is not None:
            raise self.stage_stop_error

    def nav2_goal_started(self, token: str) -> None:
        self.events.append(f"quiescence_started:{token}")

    def nav2_goal_terminal(self, token: str) -> None:
        self.events.append(f"quiescence_terminal:{token}")

    def navigation_evidence_changed(self) -> None:
        snapshot = self.ledger.snapshot()
        assert snapshot
        status = snapshot[0].status
        self.status_snapshots.append(status)
        if (
            self.evidence_error is not None
            and (
                self.evidence_error_statuses is None
                or status in self.evidence_error_statuses
            )
        ):
            raise self.evidence_error


class _OneShotFailingReadLedger(NavigationGoalLedger):
    """第一次 precondition 读取成功，accepted 后的运行期读取失败一次。"""

    def __init__(self):
        super().__init__()
        self._get_calls = 0

    def get(self, sequence: int):
        self._get_calls += 1
        if self._get_calls == 2:
            raise RuntimeError("ledger snapshot unavailable")
        return super().get(sequence)


def _requested_sampled_ledger(
    *,
    goal_xy=(1.0, 2.0),
    plan_count: int = 1,
    unsafe: bool = False,
) -> NavigationGoalLedger:
    ledger = NavigationGoalLedger()
    ledger.reset(1)
    ledger.plan((goal_xy,))
    ledger.transition(
        1,
        NavigationGoalStatus.REQUESTED,
        timestamp_ns=1,
    )
    for _ in range(plan_count):
        ledger.record_plan(
            1,
            unknown_cell_count=1 if unsafe else 0,
            occupied_cell_count=0,
            outside_map_cell_count=0,
        )
    return ledger


def _transaction(runtime, ledger, *, safety_timeout_s=0.2):
    return Nav2MotionTransaction(
        runtime,
        ledger,
        safety_timeout_s=safety_timeout_s,
        poll_interval_s=0.05,
        late_plan_drain_s=0.3,
    )


def _cancel_after_pre_dispatch_checks():
    calls = 0

    def callback() -> bool:
        nonlocal calls
        calls += 1
        return calls >= 3

    return callback


def _raise_after_pre_dispatch_checks():
    calls = 0

    def callback() -> bool:
        nonlocal calls
        calls += 1
        if calls >= 3:
            raise RuntimeError("request state unavailable")
        return False

    return callback


def test_sampled_navigation_hides_goal_lifecycle_and_typed_evidence():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    handle = _Handle(
        runtime,
        _DoneFuture(
            ActionTerminal(
                status=ActionStatus.SUCCEEDED,
                error_code=0,
            )
        ),
    )
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    _transaction(runtime, ledger).execute(
        SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
        is_cancelled=lambda: False,
        deadline_monotonic=10.0,
    )

    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.SUCCEEDED
    assert evidence.nav2_status == ActionStatus.SUCCEEDED
    assert runtime.status_snapshots == [
        NavigationGoalStatus.ACCEPTED,
        NavigationGoalStatus.EXECUTING,
        NavigationGoalStatus.SUCCEEDED,
    ]
    assert runtime.sent_goals == [
        NavigateGoal(frame_id="map", x=1.0, y=2.0, yaw=0.0)
    ]
    assert "quiescence_started:sampled-1:1" in runtime.events
    assert "quiescence_terminal:sampled-1:1" in runtime.events
    assert "priority_stop" not in runtime.events


def test_aborted_terminal_primary_survives_terminal_observer_failure():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.evidence_error = RuntimeError("aborted evidence publish failed")
    runtime.evidence_error_statuses = {NavigationGoalStatus.ABORTED}
    handle = _Handle(
        runtime,
        _DoneFuture(
            ActionTerminal(
                status=ActionStatus.ABORTED,
                error_code=208,
                error_message="controller failed",
            )
        ),
    )
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(
        Nav2MotionFailed,
        match="navigation goal failed",
    ) as raised:
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert "status=6 error=208" in str(raised.value)
    assert any(
        "aborted evidence publish failed" in note
        for note in getattr(raised.value, "__notes__", ())
    )
    assert handle.cancel_count == 0
    assert "priority_stop" not in runtime.events
    assert "quiescence_terminal:sampled-1:1" in runtime.events
    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.ABORTED
    assert evidence.nav2_status == ActionStatus.ABORTED
    assert evidence.nav2_error_code == 208


def test_pending_cancel_waits_for_late_accept_then_stops_and_proves_terminal():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    result_future = _CancelCompletesResult(polls_after_cancel=2)
    handle = _Handle(runtime, result_future)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DelayedFuture(handle, pending_polls=2),
    )

    with pytest.raises(AutomaticMissionCancelled):
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=_cancel_after_pre_dispatch_checks(),
            deadline_monotonic=10.0,
        )

    assert handle.cancel_count == 1
    assert result_future.terminal is True
    assert ledger.get(1).status is NavigationGoalStatus.CANCELED
    cancel_index = runtime.events.index("cancel")
    stop_index = runtime.events.index("priority_stop")
    terminal_index = runtime.events.index(
        "quiescence_terminal:sampled-1:1"
    )
    assert cancel_index < stop_index < terminal_index
    assert "stage_stop" not in runtime.events


def test_pending_goal_without_response_fails_closed_by_stopping_stage():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _NeverFuture(),
    )

    with pytest.raises(
        Nav2SafetyFailure,
        match="goal response did not arrive within safety budget",
    ):
        _transaction(
            runtime,
            ledger,
            safety_timeout_s=0.11,
        ).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=_cancel_after_pre_dispatch_checks(),
            deadline_monotonic=10.0,
        )

    assert runtime.events[-1] == "stage_stop"
    assert runtime.events.index("priority_stop") < runtime.events.index(
        "stage_stop"
    )
    # late response 已经耗尽唯一 cleanup deadline，仍必须用 timeout=0 发布
    # priority STOP；“没有等待预算”不能等价成“不停车”。
    assert runtime.stop_timeouts == [0.0]
    assert not any(
        event.startswith("quiescence_terminal")
        for event in runtime.events
    )
    assert ledger.get(1).status is NavigationGoalStatus.ABORTED


def test_late_response_result_exception_reuses_remaining_cleanup_deadline():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DelayedRaisingResponse(None, pending_polls=2),
    )

    with pytest.raises(
        Nav2SafetyFailure,
        match="late response payload unavailable",
    ):
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=_cancel_after_pre_dispatch_checks(),
            deadline_monotonic=10.0,
        )

    assert runtime.events.index("priority_stop") < runtime.events.index(
        "stage_stop"
    )
    # 等待迟到 response 已消费 0.05s，STOP 只能使用原 0.2s cleanup
    # deadline 的余额，不能重新获得一份完整预算。
    assert runtime.stop_timeouts == [pytest.approx(0.15)]
    assert ledger.get(1).status is NavigationGoalStatus.ABORTED


def test_late_unreadable_acceptance_reuses_remaining_cleanup_deadline():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DelayedFuture(
            _HandleWithUnreadableAcceptance(),
            pending_polls=2,
        ),
    )

    with pytest.raises(
        Nav2SafetyFailure,
        match="acceptance channel unavailable",
    ):
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=_cancel_after_pre_dispatch_checks(),
            deadline_monotonic=10.0,
        )

    assert runtime.events.index("priority_stop") < runtime.events.index(
        "stage_stop"
    )
    assert runtime.stop_timeouts == [pytest.approx(0.15)]
    assert ledger.get(1).status is NavigationGoalStatus.ABORTED


def test_unsafe_sampled_plan_cancels_stops_and_waits_for_terminal():
    ledger = _requested_sampled_ledger(unsafe=True)
    runtime = _RuntimeAdapter(ledger)
    result_future = _CancelCompletesResult()
    handle = _Handle(runtime, result_future)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(Nav2MotionFailed, match="unsafe runtime"):
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert handle.cancel_count == 1
    assert result_future.terminal is True
    assert runtime.events.index("cancel") < runtime.events.index(
        "priority_stop"
    )
    assert "stage_stop" not in runtime.events
    assert ledger.get(1).status is NavigationGoalStatus.ABORTED


def test_invalid_terminal_cancels_stops_then_stops_stage_when_unprovable():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    handle = _Handle(
        runtime,
        _DoneFuture(
            ActionTerminal(
                status=ActionStatus.EXECUTING,
                error_code=0,
            )
        ),
    )
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(Nav2SafetyFailure, match="non-terminal status"):
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert runtime.events.index("cancel") < runtime.events.index(
        "priority_stop"
    )
    assert runtime.events[-1] == "stage_stop"
    assert not any(
        event.startswith("quiescence_terminal")
        for event in runtime.events
    )
    assert ledger.get(1).status is NavigationGoalStatus.ABORTED


def test_mapping_return_builds_pose_goal_and_shares_terminal_protocol():
    ledger = NavigationGoalLedger()
    runtime = _RuntimeAdapter(ledger)
    handle = _Handle(
        runtime,
        _DoneFuture(
            ActionTerminal(
                status=ActionStatus.SUCCEEDED,
                error_code=0,
            )
        ),
    )
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )
    pose = PlanarPose(
        x=-1.25,
        y=0.5,
        yaw=0.75,
        frame_id="map",
        observed_at_ns=123,
    )

    _transaction(runtime, ledger).execute(
        MappingReturn(goal_pose=pose),
        is_cancelled=lambda: False,
        deadline_monotonic=10.0,
    )

    assert runtime.sent_goals == [
        NavigateGoal(
            frame_id="map",
            x=-1.25,
            y=0.5,
            yaw=0.75,
        )
    ]
    assert "quiescence_terminal:mapping-return:1" in runtime.events
    assert runtime.status_snapshots == []


def test_backup_business_failure_finishes_quiescence_then_forces_stop():
    ledger = NavigationGoalLedger()
    runtime = _RuntimeAdapter(ledger)
    handle = _Handle(
        runtime,
        _DoneFuture(
            ActionTerminal(
                status=ActionStatus.ABORTED,
                error_code=407,
                error_message="collision ahead",
            )
        ),
    )
    runtime.enqueue(
        Nav2ActionKind.BACK_UP,
        _DoneFuture(handle),
    )

    with pytest.raises(Nav2MotionFailed, match="collision ahead"):
        _transaction(runtime, ledger).execute(
            RecoveryBackup(
                distance_m=0.3,
                speed_mps=0.08,
                time_allowance_s=10.0,
            ),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert runtime.sent_goals == [
        BackupGoal(
            distance_m=0.3,
            speed_mps=0.08,
            time_allowance_s=10.0,
        )
    ]
    assert runtime.events.index(
        "quiescence_terminal:backup:1"
    ) < runtime.events.index("priority_stop")
    assert "cancel" not in runtime.events
    assert "stage_stop" not in runtime.events


def test_rejected_handle_closes_quiescence_and_marks_ledger_rejected():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(
            _Handle(
                runtime,
                _NeverFuture(),
                accepted=False,
            )
        ),
    )

    with pytest.raises(Nav2GoalRejected, match="navigation goal rejected"):
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.REJECTED
    assert evidence.nav2_status == ActionStatus.UNKNOWN
    assert "quiescence_terminal:sampled-1:1" in runtime.events
    assert "priority_stop" not in runtime.events
    assert "stage_stop" not in runtime.events


def test_goal_dispatch_exception_is_uncertain_and_fails_closed():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.send_error = RuntimeError("DDS write failed")

    with pytest.raises(
        Nav2SafetyFailure,
        match="DDS write failed",
    ) as raised:
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert "cannot read navigation goal response" in str(raised.value)
    assert runtime.events[-1] == "stage_stop"
    assert runtime.events.index("priority_stop") < runtime.events.index(
        "stage_stop"
    )
    assert runtime.stop_timeouts == [pytest.approx(0.2)]
    assert not any(
        event.startswith("quiescence_terminal")
        for event in runtime.events
    )
    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.ABORTED
    assert evidence.nav2_status == ActionStatus.UNKNOWN


def test_completed_response_exception_stops_stage_and_preserves_reason():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _RaisingResultFuture("response payload corrupt"),
    )

    with pytest.raises(
        Nav2SafetyFailure,
        match="response payload corrupt",
    ) as raised:
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert "cannot read navigation goal response" in str(raised.value)
    assert runtime.events[-1] == "stage_stop"
    assert runtime.events.index("priority_stop") < runtime.events.index(
        "stage_stop"
    )
    assert runtime.stop_timeouts == [pytest.approx(0.2)]
    assert ledger.get(1).status is NavigationGoalStatus.ABORTED


def test_response_readiness_exception_stops_unknown_motion_before_stage():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _RaisingDoneFuture(),
    )

    with pytest.raises(
        Nav2SafetyFailure,
        match="response readiness unavailable",
    ):
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert runtime.events.index("priority_stop") < runtime.events.index(
        "stage_stop"
    )
    assert runtime.stop_timeouts == [pytest.approx(0.2)]
    assert ledger.get(1).status is NavigationGoalStatus.ABORTED


def test_unreadable_acceptance_stops_unknown_motion_before_stage_shutdown():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(_HandleWithUnreadableAcceptance()),
    )

    with pytest.raises(
        Nav2SafetyFailure,
        match="acceptance channel unavailable",
    ):
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert runtime.events.index("priority_stop") < runtime.events.index(
        "stage_stop"
    )
    assert runtime.stop_timeouts == [pytest.approx(0.2)]
    assert not any(
        event.startswith("quiescence_terminal")
        for event in runtime.events
    )
    assert ledger.get(1).status is NavigationGoalStatus.ABORTED


def test_result_future_exception_cancels_stops_and_fails_closed():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    handle = _Handle(
        runtime,
        _RaisingResultFuture("result wrapper corrupt"),
    )
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(
        Nav2SafetyFailure,
        match="result wrapper corrupt",
    ) as raised:
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert "invalid NavigateToPose terminal result" in str(raised.value)
    assert handle.cancel_count == 1
    assert runtime.events.index("cancel") < runtime.events.index(
        "priority_stop"
    )
    assert runtime.events[-1] == "stage_stop"
    assert ledger.get(1).status is NavigationGoalStatus.ABORTED


def test_missing_result_future_preserves_primary_error_through_cleanup():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    handle = _HandleWithoutResultFuture(runtime, _NeverFuture())
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(
        Nav2SafetyFailure,
        match="result channel unavailable",
    ) as raised:
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    message = str(raised.value)
    assert "cannot get Nav2 result future" in message
    assert "result future unavailable" in message
    assert handle.cancel_count == 1
    assert runtime.events.index("cancel") < runtime.events.index(
        "priority_stop"
    )
    assert runtime.events[-1] == "stage_stop"


def test_cleanup_failure_keeps_cancel_reason_and_real_nav2_terminal():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.priority_stop_error = RuntimeError("STOP acknowledgement timeout")
    result_future = _CancelCompletesResult()
    handle = _Handle(runtime, result_future)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(
        Nav2SafetyFailure,
        match="STOP acknowledgement timeout",
    ) as raised:
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=_cancel_after_pre_dispatch_checks(),
            deadline_monotonic=10.0,
        )

    assert "automatic mission canceled during navigation" in str(raised.value)
    assert runtime.events[-1] == "stage_stop"
    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.ABORTED
    assert evidence.nav2_status == ActionStatus.CANCELED
    assert evidence.nav2_error_code == 0


def test_cancelled_sampled_goal_records_actual_nav2_terminal_status():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    result_future = _CancelCompletesResult()

    def aborted_after_cancel():
        if not result_future.terminal:
            raise AssertionError("terminal was read before cancel completed")
        return ActionTerminal(
            status=ActionStatus.ABORTED,
            error_code=208,
            error_message="controller aborted while canceling",
        )

    result_future.result = aborted_after_cancel
    handle = _Handle(runtime, result_future)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(AutomaticMissionCancelled):
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=_cancel_after_pre_dispatch_checks(),
            deadline_monotonic=10.0,
        )

    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.CANCELED
    assert evidence.nav2_status == ActionStatus.ABORTED
    assert evidence.nav2_error_code == 208


def test_cancel_primary_survives_terminal_observer_failure():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.evidence_error = RuntimeError("cancel evidence publish failed")
    runtime.evidence_error_statuses = {NavigationGoalStatus.CANCELED}
    result_future = _CancelCompletesResult()
    handle = _Handle(runtime, result_future)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(
        AutomaticMissionCancelled,
        match="automatic mission canceled during navigation",
    ) as raised:
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=_cancel_after_pre_dispatch_checks(),
            deadline_monotonic=10.0,
        )

    assert handle.cancel_count == 1
    assert any(
        "cancel evidence publish failed" in note
        for note in getattr(raised.value, "__notes__", ())
    )
    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.CANCELED
    assert evidence.nav2_status == ActionStatus.CANCELED


def test_timeout_primary_survives_terminal_observer_failure():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.evidence_error = RuntimeError("timeout evidence publish failed")
    runtime.evidence_error_statuses = {NavigationGoalStatus.TIMED_OUT}
    result_future = _CancelCompletesResult()
    handle = _Handle(runtime, result_future)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(
        TimeoutError,
        match="navigation goal result timeout",
    ) as raised:
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=0.1,
        )

    assert handle.cancel_count == 1
    assert any(
        "timeout evidence publish failed" in note
        for note in getattr(raised.value, "__notes__", ())
    )
    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.TIMED_OUT
    assert evidence.nav2_status == ActionStatus.CANCELED


def test_concurrent_execute_is_rejected_without_touching_second_goal():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    release = threading.Event()
    first_poll = threading.Event()
    runtime.sleep_hook = lambda: (
        first_poll.set(),
        release.wait(timeout=2.0),
    )
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(_Handle(runtime, _EventResultFuture(release))),
    )
    transaction = _transaction(runtime, ledger)
    worker_errors: list[BaseException] = []

    def execute_first() -> None:
        try:
            transaction.execute(
                SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
                is_cancelled=lambda: False,
                deadline_monotonic=10.0,
            )
        except BaseException as exc:  # pragma: no cover - asserted below
            worker_errors.append(exc)

    worker = threading.Thread(target=execute_first, daemon=True)
    worker.start()
    assert first_poll.wait(timeout=1.0)

    with pytest.raises(Nav2TransactionBusy):
        transaction.execute(
            MappingReturn(
                goal_pose=PlanarPose(
                    x=0.0,
                    y=0.0,
                    yaw=0.0,
                    frame_id="map",
                    observed_at_ns=1,
                )
            ),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    release.set()
    worker.join(timeout=2.0)
    assert not worker.is_alive()
    assert worker_errors == []
    assert len(runtime.sent_goals) == 1


def test_cancel_callback_failure_after_acceptance_still_closes_motion():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    result_future = _CancelCompletesResult()
    handle = _Handle(runtime, result_future)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(
        Nav2MotionFailed,
        match="request state unavailable",
    ):
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=_raise_after_pre_dispatch_checks(),
            deadline_monotonic=10.0,
        )

    assert handle.cancel_count == 1
    assert runtime.events.index("cancel") < runtime.events.index(
        "priority_stop"
    )
    assert "stage_stop" not in runtime.events
    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.ABORTED
    assert evidence.nav2_status == ActionStatus.CANCELED


def test_evidence_observer_failure_after_acceptance_still_closes_motion():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.evidence_error = RuntimeError("state publisher unavailable")
    result_future = _CancelCompletesResult()
    handle = _Handle(runtime, result_future)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(
        Nav2MotionFailed,
        match="state publisher unavailable",
    ) as raised:
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert str(raised.value).startswith(
        "cannot record sampled navigation acceptance"
    )
    assert handle.cancel_count == 1
    assert runtime.events.index("cancel") < runtime.events.index(
        "priority_stop"
    ) < runtime.events.index("quiescence_terminal:sampled-1:1")
    assert "stage_stop" not in runtime.events
    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.ABORTED
    assert evidence.nav2_status == ActionStatus.CANCELED


def test_ledger_failure_after_acceptance_still_closes_motion():
    ledger = _OneShotFailingReadLedger()
    ledger.reset(1)
    ledger.plan(((1.0, 2.0),))
    ledger.transition(
        1,
        NavigationGoalStatus.REQUESTED,
        timestamp_ns=1,
    )
    ledger.record_plan(
        1,
        unknown_cell_count=0,
        occupied_cell_count=0,
        outside_map_cell_count=0,
    )
    runtime = _RuntimeAdapter(ledger)
    result_future = _CancelCompletesResult()
    handle = _Handle(runtime, result_future)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(
        Nav2MotionFailed,
        match="ledger snapshot unavailable",
    ) as raised:
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert str(raised.value).startswith(
        "cannot audit sampled navigation ledger after acceptance"
    )
    assert handle.cancel_count == 1
    assert runtime.events.index("cancel") < runtime.events.index(
        "priority_stop"
    ) < runtime.events.index("quiescence_terminal:sampled-1:1")
    assert "stage_stop" not in runtime.events
    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.ABORTED
    assert evidence.nav2_status == ActionStatus.CANCELED


def test_terminal_plan_ledger_failure_stops_and_preserves_real_terminal():
    ledger = _OneShotFailingReadLedger()
    ledger.reset(1)
    ledger.plan(((1.0, 2.0),))
    ledger.transition(
        1,
        NavigationGoalStatus.REQUESTED,
        timestamp_ns=1,
    )
    ledger.record_plan(
        1,
        unknown_cell_count=0,
        occupied_cell_count=0,
        outside_map_cell_count=0,
    )
    runtime = _RuntimeAdapter(ledger)
    handle = _Handle(
        runtime,
        _DoneFuture(
            ActionTerminal(
                status=ActionStatus.SUCCEEDED,
                error_code=0,
            )
        ),
    )
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(handle),
    )

    with pytest.raises(
        Nav2MotionFailed,
        match="ledger snapshot unavailable",
    ) as raised:
        _transaction(runtime, ledger).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=lambda: False,
            deadline_monotonic=10.0,
        )

    assert str(raised.value).startswith(
        "cannot audit sampled navigation ledger after terminal"
    )
    assert handle.cancel_count == 1
    assert runtime.events.index("cancel") < runtime.events.index(
        "priority_stop"
    ) < runtime.events.index("quiescence_terminal:sampled-1:1")
    assert "stage_stop" not in runtime.events
    evidence = ledger.get(1)
    assert evidence.status is NavigationGoalStatus.ABORTED
    assert evidence.nav2_status == ActionStatus.SUCCEEDED
    assert evidence.nav2_error_code == 0


def test_late_accept_stop_and_terminal_share_one_cleanup_deadline():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    handle = _Handle(runtime, _NeverFuture())
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DelayedFuture(handle, pending_polls=4),
    )

    with pytest.raises(
        Nav2SafetyFailure,
        match="did not reach terminal",
    ):
        _transaction(
            runtime,
            ledger,
            safety_timeout_s=0.2,
        ).execute(
            SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
            is_cancelled=_cancel_after_pre_dispatch_checks(),
            deadline_monotonic=10.0,
        )

    # late response 已消费约 0.15s，STOP 与 terminal 只能共享剩余预算；
    # 若每一步重置 0.2s，这里会膨胀到约 0.4s。
    assert runtime.stop_timeouts == [pytest.approx(0.05)]
    assert runtime.now == pytest.approx(0.2)
    assert runtime.events[-1] == "stage_stop"


def test_late_plan_drain_is_capped_by_business_deadline():
    ledger = _requested_sampled_ledger()
    runtime = _RuntimeAdapter(ledger)
    runtime.enqueue(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        _DoneFuture(
            _Handle(
                runtime,
                _DoneFuture(
                    ActionTerminal(
                        status=ActionStatus.SUCCEEDED,
                        error_code=0,
                    )
                ),
            )
        ),
    )

    _transaction(runtime, ledger).execute(
        SampledNavigate(sequence=1, goal_xy=(1.0, 2.0)),
        is_cancelled=lambda: False,
        deadline_monotonic=0.1,
    )

    # Action 立即 terminal 时，排空窗口最多只能消费剩余的 0.1s，
    # 不能把公开的业务 deadline 静默扩展为 0.3s。
    assert runtime.now == pytest.approx(0.1)
    assert ledger.get(1).status is NavigationGoalStatus.SUCCEEDED
