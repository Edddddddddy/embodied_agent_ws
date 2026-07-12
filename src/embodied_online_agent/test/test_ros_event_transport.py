import pytest

from embodied_agent_interfaces.msg import CommandExecutionEvent as ExecutionMessage
from embodied_agent_interfaces.msg import CommandQueueEvent as QueueMessage
from embodied_online_agent.continuous_voice import (
    CommandExecutionEvent,
    CommandQueueEvent,
)
from embodied_online_agent.ros_event_transport import (
    execution_event_message_to_dict,
    execution_event_to_message,
    queue_event_message_to_dict,
    queue_event_to_message,
)
from embodied_online_agent.ros_qos import command_event_qos, latched_state_qos
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy


def test_queue_event_round_trip_preserves_batch_context():
    event = CommandQueueEvent(
        "enqueue",
        "offline",
        "向前走一秒",
        2,
        metadata={
            "batch_id": "batch-1",
            "batch_index": 1,
            "batch_size": 2,
            "source_text": "左转然后向前走一秒",
            "nlu_intent": "move_forward",
            "nlu_confidence": 0.875,
        },
    )

    message = queue_event_to_message(event)

    assert message.event == QueueMessage.EVENT_ENQUEUED
    assert queue_event_message_to_dict(message) == {
        "event": "enqueue",
        "source": "offline",
        "text": "向前走一秒",
        "size": 2,
        "dropped": 0,
        "reason": "",
        "priority_stop": False,
        "batch_id": "batch-1",
        "batch_index": 1,
        "batch_size": 2,
        "source_text": "左转然后向前走一秒",
        "nlu_intent": "move_forward",
        "nlu_confidence": pytest.approx(0.875),
    }


def test_execution_event_distinguishes_unknown_and_false_success():
    started = execution_event_to_message(
        CommandExecutionEvent("started", "online", "绕圈")
    )
    failed = execution_event_to_message(
        CommandExecutionEvent("finished", "online", "绕圈", False, "timeout")
    )

    assert started.event == ExecutionMessage.EVENT_STARTED
    assert execution_event_message_to_dict(started)["success"] is None
    assert failed.event == ExecutionMessage.EVENT_FINISHED
    assert execution_event_message_to_dict(failed)["success"] is False
    assert execution_event_message_to_dict(failed)["reason"] == "timeout"


def test_unknown_domain_event_is_rejected_before_reaching_ros_graph():
    with pytest.raises(ValueError, match="unsupported queue event"):
        queue_event_to_message(CommandQueueEvent("typo", "online", "x", 0))


def test_named_qos_profiles_encode_delivery_semantics():
    command_qos = command_event_qos()
    state_qos = latched_state_qos()

    assert command_qos.reliability == ReliabilityPolicy.RELIABLE
    assert command_qos.depth == 50
    assert state_qos.reliability == ReliabilityPolicy.RELIABLE
    assert state_qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert state_qos.depth == 1
