"""Agent text -> typed result correlation tests without ROS runtime."""

import pytest

from embodied_slam_tools.agent_action_gateway import (
    AgentActionGateway,
    AgentActionOutcome,
)
from embodied_slam_tools.mission_executor import (
    AutomaticMissionCancelled,
    CommandRequest,
)
from embodied_slam_tools.showcase_session import SessionCommand


def _request():
    return CommandRequest(
        command=SessionCommand.RUN_AUTOMATIC_MISSION, source="test"
    )


def _success():
    return AgentActionOutcome(True, 1, "succeeded")


def test_matches_candidate_type_and_correlated_result():
    holder = {}

    def publish(_text):
        gateway = holder["gateway"]
        gateway.record_candidate(3, "wrong-type")
        gateway.record_candidate(2, "move-1")
        gateway.record_result("move-1", _success())

    gateway = AgentActionGateway(
        publish_text=publish,
        subscriber_count=lambda: 1,
        cancel_motion=lambda: None,
    )
    holder["gateway"] = gateway

    outcome = gateway.run(
        _request(),
        text="向前走",
        expected_action_type=2,
        expected_action_name="move",
        timeout_s=0.1,
    )

    assert outcome == _success()


def test_failed_action_preserves_backend_status_and_message():
    holder = {}

    def publish(_text):
        gateway = holder["gateway"]
        gateway.record_candidate(3, "turn-1")
        gateway.record_result(
            "turn-1", AgentActionOutcome(False, 5, "laser blocked")
        )

    gateway = AgentActionGateway(
        publish_text=publish,
        subscriber_count=lambda: 1,
        cancel_motion=lambda: None,
    )
    holder["gateway"] = gateway

    with pytest.raises(RuntimeError, match="status=5: laser blocked"):
        gateway.run(
            _request(),
            text="左转",
            expected_action_type=3,
            expected_action_name="turn",
            timeout_s=0.1,
        )


def test_typed_command_waits_for_its_correlated_result_without_text_round_trip():
    holder = {}

    def publish():
        holder["gateway"].record_result("slam-stop-1", _success())

    gateway = AgentActionGateway(
        publish_text=lambda _text: pytest.fail("typed action must not publish text"),
        subscriber_count=lambda: 0,
        cancel_motion=lambda: None,
    )
    holder["gateway"] = gateway

    outcome = gateway.run_typed(
        _request(),
        command_id="slam-stop-1",
        publish_command=publish,
        expected_action_name="stop",
        timeout_s=0.1,
    )

    assert outcome == _success()


def test_stale_reused_command_id_does_not_complete_new_request():
    holder = {}

    def publish(_text):
        holder["gateway"].record_candidate(2, "agent-action-1")

    gateway = AgentActionGateway(
        publish_text=publish,
        subscriber_count=lambda: 1,
        cancel_motion=lambda: None,
    )
    holder["gateway"] = gateway
    gateway.record_result("agent-action-1", _success())

    with pytest.raises(TimeoutError, match="action result timeout"):
        gateway.run(
            _request(),
            text="向前走",
            expected_action_type=2,
            expected_action_name="move",
            timeout_s=0.01,
        )


def test_cancel_stops_motion_before_propagating():
    stops = []
    request = _request()
    request.canceled = True
    gateway = AgentActionGateway(
        publish_text=lambda _text: None,
        subscriber_count=lambda: 0,
        cancel_motion=lambda: stops.append(True),
    )

    with pytest.raises(AutomaticMissionCancelled):
        gateway.run(
            request,
            text="向前走",
            expected_action_type=2,
            expected_action_name="move",
            timeout_s=0.1,
        )

    assert stops == [True]


def test_subscriber_timeout_is_explicit():
    gateway = AgentActionGateway(
        publish_text=lambda _text: None,
        subscriber_count=lambda: 0,
        cancel_motion=lambda: None,
    )

    with pytest.raises(TimeoutError, match="subscriber unavailable"):
        gateway.run(
            _request(),
            text="向前走",
            expected_action_type=2,
            expected_action_name="move",
            timeout_s=0.0,
        )


def test_dry_run_has_no_runtime_dependency():
    messages = []
    gateway = AgentActionGateway(
        publish_text=lambda _text: pytest.fail("must not publish"),
        subscriber_count=lambda: 0,
        cancel_motion=lambda: None,
        dry_run=True,
        log_dry_run=messages.append,
    )

    assert gateway.run(
        _request(),
        text="向前走",
        expected_action_type=2,
        expected_action_name="move",
        timeout_s=0.0,
    ) is None
    assert messages == ["DRY RUN agent action: 向前走 -> move"]
