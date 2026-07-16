"""Lightweight tests for the typed automatic-mission CLI."""

import argparse
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

from embodied_agent_interfaces.action import ManageSlamSession
from embodied_agent_interfaces.msg import SlamSessionState


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "trigger_automatic_slam_mission.py"
SPEC = importlib.util.spec_from_file_location(
    "trigger_automatic_slam_mission", SCRIPT
)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


@pytest.mark.parametrize("value", ["0", "-1", "nan", "inf", "-inf", "bad"])
def test_timeout_arguments_must_be_positive_and_finite(value):
    with pytest.raises(argparse.ArgumentTypeError):
        MODULE.positive_finite_seconds(value)


def test_mapping_readiness_uses_typed_phase_constants_and_fails_fast():
    assert not MODULE.mapping_is_ready(None)

    state = SlamSessionState()
    state.phase = SlamSessionState.STARTING_MAPPING
    assert not MODULE.mapping_is_ready(state)

    state.phase = SlamSessionState.STOPPED
    state.detail = ""
    assert not MODULE.mapping_is_ready(state)

    state.detail = "session stopped"
    with pytest.raises(RuntimeError, match="session stopped"):
        MODULE.mapping_is_ready(state)

    state.phase = SlamSessionState.MAPPING
    state.detail = "mapping ready"
    assert MODULE.mapping_is_ready(state)

    state.phase = SlamSessionState.AUTOMATIC_MAPPING
    with pytest.raises(RuntimeError, match="already started"):
        MODULE.mapping_is_ready(state)

    state.phase = SlamSessionState.FAILED
    state.detail = "mapper exited"
    with pytest.raises(RuntimeError, match="mapper exited"):
        MODULE.mapping_is_ready(state)

    state.phase = SlamSessionState.NAVIGATING
    with pytest.raises(RuntimeError, match="cannot start"):
        MODULE.mapping_is_ready(state)


class StubFuture:
    def __init__(self, *, done: bool, result=None):
        self._done = done
        self._result = result

    def done(self):
        return self._done

    def result(self):
        return self._result


class StubGoalHandle:
    accepted = True

    def __init__(self):
        self.result_future = StubFuture(done=False)
        self.cancel_future = StubFuture(
            done=True, result=SimpleNamespace(goals_canceling=[object()])
        )
        self.cancel_calls = 0

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        return self.cancel_future


class StubClient:
    def __init__(self, send_future):
        self.send_future = send_future
        self.goal = None

    def wait_for_server(self, *, timeout_sec):
        assert timeout_sec == 10.0
        return True

    def send_goal_async(self, goal, *, feedback_callback):
        assert feedback_callback is not None
        self.goal = goal
        return self.send_future


class StubRclpy:
    def __init__(self, result_future=None, goal_handle=None):
        self.result_future = result_future
        self.goal_handle = goal_handle
        self.spun_futures = []

    @staticmethod
    def ok():
        return True

    def spin_until_future_complete(self, _node, future, *, timeout_sec):
        self.spun_futures.append((future, timeout_sec))
        if future is not self.result_future or self.goal_handle is None:
            return
        if self.goal_handle.cancel_calls:
            future._done = True

    @staticmethod
    def spin_once(_node, *, timeout_sec):
        raise AssertionError(f"unexpected spin_once timeout={timeout_sec}")


def bare_trigger(client):
    node = object.__new__(MODULE.AutomaticMissionTrigger)
    node.client = client
    return node


def test_goal_response_timeout_is_reported(monkeypatch):
    send_future = StubFuture(done=False)
    client = StubClient(send_future)
    fake_rclpy = StubRclpy()
    monkeypatch.setattr(MODULE, "rclpy", fake_rclpy)

    with pytest.raises(TimeoutError, match="goal response timed out"):
        bare_trigger(client).send(1.0)


def test_result_timeout_cancels_goal_and_waits_for_termination(monkeypatch):
    goal_handle = StubGoalHandle()
    send_future = StubFuture(done=True, result=goal_handle)
    client = StubClient(send_future)
    fake_rclpy = StubRclpy(goal_handle.result_future, goal_handle)
    clock = iter((100.0, 101.0))
    monkeypatch.setattr(MODULE, "rclpy", fake_rclpy)
    monkeypatch.setattr(MODULE.time, "monotonic", lambda: next(clock))

    with pytest.raises(TimeoutError, match="goal was canceled"):
        bare_trigger(client).send(0.5)

    assert client.goal.command == ManageSlamSession.Goal.RUN_AUTOMATIC_MISSION
    assert goal_handle.cancel_calls == 1
    assert fake_rclpy.spun_futures == [
        (send_future, 10.0),
        (goal_handle.cancel_future, 5.0),
        (goal_handle.result_future, 5.0),
    ]
