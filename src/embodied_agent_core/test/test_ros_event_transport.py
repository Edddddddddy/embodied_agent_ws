import pytest

from embodied_agent_interfaces.msg import CommandExecutionEvent as ExecutionMessage
from embodied_agent_interfaces.msg import CommandQueueEvent as QueueMessage
from embodied_agent_interfaces.msg import WakeEvent as WakeMessage
from embodied_agent_interfaces.msg import RecognitionFeedback as RecognitionMessage
from embodied_agent_core.continuous_voice import (
    CommandExecutionEvent,
    CommandQueueEvent,
)
from embodied_agent_core.ros_event_transport import (
    execution_event_message_to_dict,
    execution_event_to_message,
    nlu_parse_message_to_dict,
    nlu_parse_to_message,
    queue_event_message_to_dict,
    queue_event_to_message,
    recognition_feedback_message_to_dict,
    recognition_feedback_to_message,
    wake_event_message_to_dict,
    wake_event_to_message,
)
from embodied_agent_core.ros_qos import (
    audio_stream_qos,
    command_event_qos,
    latched_state_qos,
)
from rclpy.qos import DurabilityPolicy, ReliabilityPolicy
from embodied_agent_core.wake_provider import WakeEvent, WakeEventKind
from embodied_agent_core.command_nlu import CommandNLU


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
    audio_qos = audio_stream_qos()

    assert command_qos.reliability == ReliabilityPolicy.RELIABLE
    assert command_qos.depth == 50
    assert state_qos.reliability == ReliabilityPolicy.RELIABLE
    assert state_qos.durability == DurabilityPolicy.TRANSIENT_LOCAL
    assert state_qos.depth == 1
    assert audio_qos.reliability == ReliabilityPolicy.BEST_EFFORT
    assert audio_qos.depth == 20


def test_wake_event_round_trip_preserves_optional_command_semantics():
    continued = wake_event_to_message(
        WakeEvent(WakeEventKind.CONTINUE, "text", "向前走一秒", "向前走一秒")
    )
    rejected = wake_event_to_message(
        WakeEvent(WakeEventKind.REJECTED, "text", "嗯。", None)
    )

    assert continued.kind == WakeMessage.KIND_CONTINUE
    assert wake_event_message_to_dict(continued)["command"] == "向前走一秒"
    assert rejected.kind == WakeMessage.KIND_REJECTED
    assert wake_event_message_to_dict(rejected)["command"] is None


def test_recognition_feedback_round_trip_preserves_rewrite_and_retry_fields():
    normalized = recognition_feedback_to_message(
        {
            "status": "normalized",
            "reason": "command_normalized",
            "original": "小志",
            "normalized": "小智",
            "confidence": 0.92,
            "matches": [
                {"source": "小志", "target": "小智", "score": 0.92, "provider": "alias"}
            ],
        }
    )
    retry = recognition_feedback_to_message(
        {
            "status": "retry",
            "reason": "wake_word_not_detected",
            "transcript": "嗯",
            "attempt": 2,
            "max_attempts": 3,
        }
    )

    assert normalized.status == RecognitionMessage.STATUS_NORMALIZED
    normalized_payload = recognition_feedback_message_to_dict(normalized)
    assert normalized_payload["normalized"] == "小智"
    assert normalized_payload["matches"][0]["provider"] == "alias"
    assert recognition_feedback_message_to_dict(retry)["attempt"] == 2


def test_nlu_parse_event_preserves_typed_slots_and_actions():
    result = CommandNLU().parse("向右转九十度，然后向前走一秒")

    message = nlu_parse_to_message(
        "向右转九十度，然后向前走一秒",
        result,
        "batch-7",
        source="online",
    )
    payload = nlu_parse_message_to_dict(message)

    assert payload["status"] == "nlu_parsed"
    assert payload["batch_id"] == "batch-7"
    assert [item["intent"] for item in payload["commands"]] == [
        "turn_right",
        "move_forward",
    ]
    assert payload["commands"][0]["slots"]["angle_deg"] == pytest.approx(90.0)
    assert payload["commands"][0]["actions"][0]["name"] == "turn"


def test_nlu_parse_event_preserves_waypoint_list_slot():
    result = CommandNLU().parse("依次去门口、书桌、起点")

    message = nlu_parse_to_message(
        "依次去门口、书桌、起点",
        result,
        "batch-nav",
        source="offline",
    )
    payload = nlu_parse_message_to_dict(message)

    assert payload["commands"][0]["slots"]["waypoints"] == [
        "door",
        "desk",
        "home",
    ]


def test_unknown_recognition_status_is_rejected():
    with pytest.raises(ValueError, match="unsupported recognition feedback status"):
        recognition_feedback_to_message({"status": "typo"})
