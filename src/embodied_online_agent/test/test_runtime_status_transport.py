from dataclasses import dataclass
from enum import Enum

from embodied_agent_interfaces.msg import (
    AudioFrontendStatus,
    BehaviorTreeStatus,
    RobotActionAck,
    SimulationState,
    VadEvent,
)
from embodied_online_agent.runtime_status_transport import (
    action_ack_to_dict,
    audio_frontend_status_to_dict,
    audio_frontend_status_to_message,
    behavior_tree_status_to_dict,
    simulation_state_to_dict,
    vad_event_to_dict,
    vad_event_to_message,
)


class EventName(Enum):
    SPEECH_ENDED = "speech_ended"


@dataclass
class EndpointEvent:
    name: EventName
    reason: str
    probability: float


def test_audio_frontend_status_round_trip_preserves_operational_fields():
    message = audio_frontend_status_to_message(
        {
            "rms": 0.031,
            "peak": 2048,
            "speech": True,
            "vad_provider": "energy",
            "audio_enhancer_active": "nlms",
            "aec_active": True,
            "dropped_input_frames": 2,
        }
    )

    assert isinstance(message, AudioFrontendStatus)
    payload = audio_frontend_status_to_dict(message)
    assert payload["rms"] == message.rms
    assert payload["peak"] == 2048
    assert payload["speech"] is True
    assert payload["audio_enhancer_active"] == "nlms"
    assert payload["dropped_input_frames"] == 2


def test_vad_event_uses_stable_enum_instead_of_free_form_json():
    message = vad_event_to_message(
        EndpointEvent(EventName.SPEECH_ENDED, "silence_timeout", 0.12),
        provider="silero",
    )

    assert message.event == VadEvent.EVENT_SPEECH_ENDED
    assert vad_event_to_dict(message) == {
        "name": "speech_ended",
        "reason": "silence_timeout",
        "probability": message.probability,
        "provider": "silero",
    }


def test_runtime_status_converters_keep_reports_readable():
    ack = RobotActionAck()
    ack.action = "move"
    ack.backend = "simulation"
    ack.sequence = 7
    ack.status = RobotActionAck.STATUS_SUCCEEDED
    assert action_ack_to_dict(ack)["status"] == "succeeded"

    state = SimulationState()
    state.mode = "manual"
    state.front_distance_valid = False
    state.right_distance_valid = True
    state.right_distance = 0.8
    assert simulation_state_to_dict(state)["front_distance"] is None
    assert simulation_state_to_dict(state)["right_distance"] == state.right_distance

    bt = BehaviorTreeStatus()
    bt.command_id = "cmd-7"
    bt.stage = "confirm"
    bt.outcome = BehaviorTreeStatus.OUTCOME_SUCCEEDED
    assert behavior_tree_status_to_dict(bt)["outcome"] == "succeeded"
