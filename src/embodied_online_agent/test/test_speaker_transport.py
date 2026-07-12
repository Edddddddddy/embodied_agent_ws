from embodied_agent_interfaces.msg import SpeakerEnrollStatus
from embodied_online_agent.memory_command_service import SpeakerEnrollRequest
from embodied_online_agent.speaker_transport import (
    enroll_request_to_message,
    enroll_status_to_dict,
    enroll_status_to_message,
    identity_message_to_domain,
    identity_payload_to_message,
)


def test_identity_transport_applies_agent_confidence_policy():
    message = identity_payload_to_message(
        {
            "speaker_id": "alice",
            "confidence": 0.52,
            "enrolled": True,
            "model": "sherpa-onnx",
            "reason": "matched",
        }
    )

    identity = identity_message_to_domain(message, min_confidence=0.55)

    assert identity.speaker_id == "unknown"
    assert identity.usable is False
    assert identity.confidence == message.confidence


def test_enrollment_messages_have_stable_schema_and_enum():
    request = enroll_request_to_message(SpeakerEnrollRequest("alice", "Alice", 3))
    status = enroll_status_to_message(
        {
            "status": "collecting",
            "reason": "sample_saved",
            "speaker_id": "alice",
            "display_name": "Alice",
            "collected": 2,
            "required": 3,
            "sample_path": "/tmp/sample.wav",
        }
    )

    assert request.samples_required == 3
    assert status.status == SpeakerEnrollStatus.STATUS_COLLECTING
    assert enroll_status_to_dict(status)["collected"] == 2
