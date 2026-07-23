"""动态障碍场景事务的失败清理契约。"""

from __future__ import annotations

import pytest

from tools.acceptance.dynamic_scenario_transaction import (
    DynamicScenarioCleanupError,
    DynamicScenarioTransaction,
    cancel_pending_navigation,
)


def _transaction(events: list[str], warnings: list[str]):
    return DynamicScenarioTransaction(
        park_obstacle=lambda: events.append("park"),
        clear_detection=lambda: events.append("clear"),
        verify_scene_cleared=lambda: events.append("scene_cleared"),
        verify_stopped=lambda: events.append("stopped"),
        warning=warnings.append,
        clear_attempts=3,
        clear_settle_s=0.0,
        clear_interval_s=0.0,
        sleep=lambda _seconds: None,
    )


def test_cleanup_order_cancels_goal_then_resets_scene_and_verifies_stop():
    events: list[str] = []
    warnings: list[str] = []
    transaction = _transaction(events, warnings)

    with transaction:
        transaction.set_cancel_navigation(lambda: events.append("cancel"))

    assert events == [
        "cancel",
        "park",
        "clear",
        "clear",
        "clear",
        "scene_cleared",
        "stopped",
    ]
    assert warnings == []


def test_marking_obstacle_parked_avoids_a_duplicate_gazebo_pose_update():
    events: list[str] = []
    transaction = _transaction(events, [])

    with transaction:
        transaction.mark_obstacle_parked()

    assert events == ["clear", "clear", "clear", "scene_cleared", "stopped"]


def test_primary_scenario_error_is_never_masked_by_cleanup_failure():
    events: list[str] = []
    warnings: list[str] = []

    def fail_parking() -> None:
        events.append("park")
        raise RuntimeError("gz unavailable")

    transaction = DynamicScenarioTransaction(
        park_obstacle=fail_parking,
        clear_detection=lambda: events.append("clear"),
        verify_scene_cleared=lambda: events.append("scene_cleared"),
        verify_stopped=lambda: events.append("stopped"),
        warning=warnings.append,
        clear_attempts=1,
        clear_settle_s=0.0,
        clear_interval_s=0.0,
        sleep=lambda _seconds: None,
    )

    with pytest.raises(ValueError, match="planner failed") as raised:
        with transaction:
            raise ValueError("planner failed")

    assert raised.value.__notes__
    assert "gz unavailable" in raised.value.__notes__[0]
    assert events == ["park", "clear", "scene_cleared", "stopped"]
    assert len(warnings) == 1
    assert "park_obstacle" in warnings[0]
    assert "gz unavailable" in warnings[0]


def test_cleanup_failure_becomes_a_gate_failure_after_successful_scenario():
    transaction = DynamicScenarioTransaction(
        park_obstacle=lambda: None,
        clear_detection=lambda: (_ for _ in ()).throw(
            RuntimeError("tracker unavailable")
        ),
        verify_scene_cleared=lambda: None,
        verify_stopped=lambda: None,
        warning=lambda _message: None,
        clear_attempts=1,
        clear_settle_s=0.0,
        clear_interval_s=0.0,
        sleep=lambda _seconds: None,
    )

    with pytest.raises(DynamicScenarioCleanupError, match="clear_detection"):
        with transaction:
            pass


def test_close_is_idempotent_when_outer_cleanup_runs_twice():
    events: list[str] = []
    transaction = _transaction(events, [])

    transaction.close()
    transaction.close()

    assert events == [
        "park",
        "clear",
        "clear",
        "clear",
        "scene_cleared",
        "stopped",
    ]


def test_inter_clear_wait_failure_does_not_skip_remaining_safety_steps():
    events: list[str] = []
    warnings: list[str] = []
    wait_calls = 0

    def flaky_sleep(_seconds: float) -> None:
        nonlocal wait_calls
        wait_calls += 1
        events.append("wait")
        if wait_calls == 1:
            raise RuntimeError("timer interrupted")

    transaction = DynamicScenarioTransaction(
        park_obstacle=lambda: events.append("park"),
        clear_detection=lambda: events.append("clear"),
        verify_scene_cleared=lambda: events.append("scene_cleared"),
        verify_stopped=lambda: events.append("stopped"),
        warning=warnings.append,
        clear_attempts=3,
        clear_settle_s=0.0,
        clear_interval_s=0.1,
        sleep=flaky_sleep,
    )

    with pytest.raises(DynamicScenarioCleanupError, match="timer interrupted"):
        with transaction:
            pass

    assert events == [
        "park",
        "clear",
        "wait",
        "clear",
        "wait",
        "clear",
        "scene_cleared",
        "stopped",
    ]
    assert "wait_between_clears" in warnings[0]


class _Future:
    def __init__(self, *, done: bool = False) -> None:
        self.complete = done

    def done(self) -> bool:
        return self.complete


class _GoalHandle:
    def __init__(self, cancel_future: _Future) -> None:
        self.cancel_future = cancel_future
        self.cancel_calls = 0

    def cancel_goal_async(self) -> _Future:
        self.cancel_calls += 1
        return self.cancel_future


def test_completed_navigation_is_not_canceled():
    handle = _GoalHandle(_Future(done=True))

    cancel_pending_navigation(
        goal_handle=handle,
        result_future=_Future(done=True),
        wait_until=lambda *_args: None,
        timeout_s=2.0,
    )

    assert handle.cancel_calls == 0


def test_pending_navigation_waits_for_cancel_response_and_terminal_result():
    result_future = _Future()
    cancel_future = _Future()
    handle = _GoalHandle(cancel_future)
    waited: list[str] = []

    def wait_until(predicate, _timeout: float, description: str) -> None:
        waited.append(description)
        if "cancel response" in description:
            cancel_future.complete = True
        else:
            result_future.complete = True
        assert predicate()

    cancel_pending_navigation(
        goal_handle=handle,
        result_future=result_future,
        wait_until=wait_until,
        timeout_s=2.0,
    )

    assert handle.cancel_calls == 1
    assert waited == [
        "dynamic NavigateToPose cancel response timeout",
        "dynamic NavigateToPose did not reach a terminal result after cancel",
    ]
