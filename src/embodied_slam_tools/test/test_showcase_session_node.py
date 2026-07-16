"""Focused tests for ROS-facing showcase orchestrator helpers."""

import threading
from types import SimpleNamespace

from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
import pytest

from embodied_slam_tools.showcase_session_node import (
    AutomaticMissionCancelled,
    _get_lifecycle_state,
    _wait_for_required_event,
)


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

    assert _wait_for_required_event(event, 0.1, lambda: False)


def test_wait_for_required_event_reports_timeout():
    assert not _wait_for_required_event(
        threading.Event(), 0.001, lambda: False, poll_s=0.001
    )


def test_wait_for_required_event_honors_cancel_before_readiness():
    with pytest.raises(AutomaticMissionCancelled):
        _wait_for_required_event(threading.Event(), 1.0, lambda: True)
