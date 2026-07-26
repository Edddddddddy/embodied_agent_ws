from types import SimpleNamespace

import pytest

from embodied_slam_tools.nav2_motion_ros import RosNav2MotionAdapter
from embodied_slam_tools.nav2_motion_transaction import (
    BackupGoal,
    NavigateGoal,
    Nav2ActionKind,
)


class _DoneFuture:
    def __init__(self, value):
        self._value = value

    @staticmethod
    def done():
        return True

    def result(self):
        return self._value


class _Handle:
    accepted = True

    def __init__(self, terminal_wrapper):
        self._terminal_wrapper = terminal_wrapper
        self.cancel_count = 0

    def get_result_async(self):
        return _DoneFuture(self._terminal_wrapper)

    def cancel_goal_async(self):
        self.cancel_count += 1
        return _DoneFuture(SimpleNamespace())


class _Client:
    def __init__(self, handle):
        self.handle = handle
        self.goals = []
        self.waits = []

    def wait_for_server(self, *, timeout_sec):
        self.waits.append(timeout_sec)
        return True

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return _DoneFuture(self.handle)


def _adapter(*, navigate_wrapper=None, backup_wrapper=None):
    navigate_client = _Client(_Handle(navigate_wrapper))
    backup_client = _Client(_Handle(backup_wrapper))
    events = []
    adapter = RosNav2MotionAdapter(
        navigate_to_pose_client=navigate_client,
        backup_client=backup_client,
        force_priority_stop=lambda timeout: events.append(("stop", timeout)),
        stop_navigation_stage=lambda: events.append(("stage_stop",)),
        nav2_goal_started=lambda token: events.append(("started", token)),
        nav2_goal_terminal=lambda token: events.append(("terminal", token)),
        evidence_timestamp_ns=lambda: 123,
        navigation_evidence_changed=lambda: events.append(("evidence",)),
    )
    return adapter, navigate_client, backup_client, events


def test_navigate_goal_and_terminal_are_converted_at_ros_seam():
    wrapper = SimpleNamespace(
        status=4,
        result=SimpleNamespace(error_code=0, error_msg=""),
    )
    adapter, navigate_client, _backup_client, _events = _adapter(
        navigate_wrapper=wrapper
    )

    assert adapter.wait_for_server(
        Nav2ActionKind.NAVIGATE_TO_POSE,
        timeout_s=1.25,
    )
    response = adapter.send_goal(
        NavigateGoal(frame_id="map", x=1.5, y=-0.25, yaw=1.0)
    )
    handle = response.result()
    assert handle is not None and handle.accepted
    terminal = handle.get_result_async().result()

    assert navigate_client.waits == [1.25]
    goal = navigate_client.goals[0]
    assert goal.pose.header.frame_id == "map"
    assert goal.pose.pose.position.x == pytest.approx(1.5)
    assert goal.pose.pose.position.y == pytest.approx(-0.25)
    assert goal.pose.pose.orientation.z == pytest.approx(0.4794255386)
    assert goal.pose.pose.orientation.w == pytest.approx(0.8775825619)
    assert terminal is not None
    assert terminal.status == 4
    assert terminal.error_code == 0


def test_backup_goal_keeps_positive_magnitudes_and_error_message():
    wrapper = SimpleNamespace(
        status=6,
        result=SimpleNamespace(
            error_code=7,
            error_msg="collision ahead",
        ),
    )
    adapter, _navigate_client, backup_client, _events = _adapter(
        backup_wrapper=wrapper
    )

    response = adapter.send_goal(
        BackupGoal(
            distance_m=0.30,
            speed_mps=0.08,
            time_allowance_s=10.25,
        )
    )
    terminal = response.result().get_result_async().result()

    goal = backup_client.goals[0]
    assert goal.target.x == pytest.approx(0.30)
    assert goal.target.y == 0.0
    assert goal.target.z == 0.0
    assert goal.speed == pytest.approx(0.08)
    assert goal.time_allowance.sec == 10
    assert goal.time_allowance.nanosec == 250_000_000
    assert terminal is not None
    assert terminal.status == 6
    assert terminal.error_code == 7
    assert terminal.error_message == "collision ahead"


def test_missing_ros_result_payload_is_not_treated_as_terminal():
    adapter, navigate_client, _backup_client, _events = _adapter(
        navigate_wrapper=SimpleNamespace(status=4, result=None)
    )

    result_future = (
        adapter.send_goal(NavigateGoal("map", 0.0, 0.0, 0.0))
        .result()
        .get_result_async()
    )

    with pytest.raises(RuntimeError, match="payload is missing"):
        result_future.result()
    assert len(navigate_client.goals) == 1


def test_runtime_callbacks_preserve_zero_budget_stop_and_evidence_events():
    adapter, _navigate_client, _backup_client, events = _adapter()

    adapter.force_priority_stop(timeout_s=0.0)
    adapter.stop_navigation_stage()
    adapter.nav2_goal_started("sampled-1:1")
    adapter.nav2_goal_terminal("sampled-1:1")
    adapter.navigation_evidence_changed()

    assert adapter.evidence_timestamp_ns() == 123
    assert events == [
        ("stop", 0.0),
        ("stage_stop",),
        ("started", "sampled-1:1"),
        ("terminal", "sampled-1:1"),
        ("evidence",),
    ]


def test_server_wait_rejects_negative_or_non_finite_timeout():
    adapter, _navigate_client, _backup_client, _events = _adapter()

    with pytest.raises(ValueError, match="finite and non-negative"):
        adapter.wait_for_server(
            Nav2ActionKind.BACK_UP,
            timeout_s=-0.1,
        )
    with pytest.raises(ValueError, match="finite and non-negative"):
        adapter.wait_for_server(
            Nav2ActionKind.BACK_UP,
            timeout_s=float("nan"),
        )
