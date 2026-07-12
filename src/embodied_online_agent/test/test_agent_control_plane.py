from dataclasses import replace

from embodied_online_agent.agent_control_plane import (
    AgentControlPlane,
    AgentControlPlaneConfig,
)


def _config(source: str = "online") -> AgentControlPlaneConfig:
    return AgentControlPlaneConfig(
        source=source,
        continuous_enabled=True,
        queue_size=2,
        command_max_age_s=30.0,
        wake_words=["小智"],
        wake_word_aliases=["小志"],
        wake_word_enabled=True,
        wake_timeout_s=60.0,
        duplicate_window_s=1.5,
        recognition_max_retries=3,
        command_normalization_enabled=True,
        normalization_feedback_enabled=True,
        normalization_fuzzy_threshold=0.78,
        normalization_rules_path="",
        command_completion_enabled=True,
        command_nlu_enabled=True,
        command_nlu_min_confidence=0.18,
        partial_merge_enabled=True,
        partial_max_age_s=2.0,
    )


def test_control_plane_owns_provider_independent_voice_state():
    control = AgentControlPlane(_config())

    wake = control.voice_session.accept("小志")
    command = control.voice_session.accept("向前走一秒")

    assert wake.event.kind == "wake"
    assert command.accepted is True
    assert command.command == "向前走一秒"
    assert control.command_nlu.parse(command.command).accepted is True


def test_control_plane_generates_source_scoped_batch_and_queue_events():
    control = AgentControlPlane(_config("offline"))

    assert control.next_nlu_batch_id() == "offline-nlu-1"
    assert control.next_nlu_batch_id() == "offline-nlu-2"

    snapshot = control.command_queue.put(
        "左转九十度", metadata={"batch_id": "offline-nlu-2"}
    )
    event = control.queue_event("enqueue", "左转九十度", snapshot)

    assert event.source == "offline"
    assert event.event == "enqueue"
    assert event.metadata["batch_id"] == "offline-nlu-2"


def test_transcript_decision_completes_short_command_after_wake():
    control = AgentControlPlane(_config())
    control.accept_transcript("小智", wake_word_required=True)

    decision = control.accept_transcript("左转", wake_word_required=True)

    assert decision.directive == "command"
    assert decision.command == "左转九十度"
    assert [item["status"] for item in decision.recognition_feedback] == [
        "completed"
    ]


def test_transcript_decision_retries_without_wake_word():
    control = AgentControlPlane(_config())

    decision = control.accept_transcript("向前走一秒", wake_word_required=True)

    assert decision.directive == "retry"
    assert decision.state == "retry_listening"
    assert decision.recognition_feedback[-1]["reason"] == "wake_word_not_detected"


def test_priority_stop_clears_pending_queue_and_bypasses_fifo():
    control = AgentControlPlane(_config())
    control.accept_transcript("小智", wake_word_required=True)
    control.command_queue.put("向前走一秒")

    decision = control.accept_transcript("急停", wake_word_required=True)

    assert decision.directive == "priority"
    assert decision.priority_action == "stop"
    assert decision.cancel_reason == "priority_stop"
    assert decision.dropped == 1
    assert decision.queue_event.event == "clear"
    assert control.command_queue.size() == 0


def test_normalization_can_be_disabled_without_changing_transcript():
    config = _config()
    config = replace(
        config,
        command_normalization_enabled=False,
    )
    control = AgentControlPlane(config)

    decision = control.accept_transcript("小智向前走1秒", wake_word_required=True)

    assert decision.transcript == "小智向前走1秒"
    assert not any(
        item.get("status") == "normalized"
        for item in decision.recognition_feedback
    )
