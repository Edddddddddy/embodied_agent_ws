from embodied_online_agent.continuous_voice import (
    CommandExecutionTracker,
    ContinuousCommandQueue,
    ContinuousVoiceSession,
    QueuedCommand,
    QueueSnapshot,
    SessionEventKind,
)
from embodied_online_agent.wakeword import WakeWordGate


def test_one_wake_word_opens_a_continuous_control_session():
    now = [100.0]
    gate = WakeWordGate(
        ["小智"],
        aliases=["小志"],
        enabled=True,
        active_timeout_s=60.0,
        clock=lambda: now[0],
    )
    session = ContinuousVoiceSession(gate, enabled=True)

    wake_only = session.accept("小志")
    first_command = session.accept("向前走一秒")
    now[0] += 30.0
    second_command = session.accept("左转九十度")

    assert wake_only.reason == "session_awake"
    assert wake_only.event.kind == SessionEventKind.WAKE
    assert wake_only.event.session_state == "awake"
    assert first_command.accepted
    assert first_command.command == "向前走一秒"
    assert first_command.event.kind == SessionEventKind.COMMAND
    assert second_command.accepted
    assert second_command.command == "左转九十度"


def test_sleep_phrase_closes_the_continuous_control_session():
    gate = WakeWordGate(["小智"], enabled=True, active_timeout_s=60.0)
    session = ContinuousVoiceSession(gate, enabled=True)

    assert session.accept("小智向前走").accepted
    sleeping = session.accept("退出控制")
    rejected = session.accept("向前走一秒")

    assert sleeping.reason == "session_sleep"
    assert not sleeping.session_active
    assert sleeping.event.kind == SessionEventKind.SLEEP
    assert sleeping.event.session_state == "sleeping"
    assert rejected.reason == "wake_word_not_detected"
    assert rejected.event.kind == SessionEventKind.REJECTED


def test_stop_intent_is_marked_as_priority_command():
    gate = WakeWordGate(["小智"], enabled=True, active_timeout_s=60.0)
    session = ContinuousVoiceSession(gate, enabled=True)

    decision = session.accept("小智急停")

    assert decision.accepted
    assert decision.priority_stop
    assert decision.command == "急停"
    assert decision.event.kind == SessionEventKind.COMMAND


def test_priority_stop_clears_waiting_commands_before_enqueueing_stop():
    commands = ContinuousCommandQueue(max_size=4, clock=lambda: 1.0)
    assert commands.put("向前走一秒").accepted
    assert commands.put("左转九十度").accepted

    snapshot = commands.put("停下", priority_stop=True)
    item = commands.get(timeout=0.01)

    assert snapshot.accepted
    assert snapshot.dropped == 2
    assert snapshot.size == 1
    assert item.text == "停下"
    assert item.priority_stop


def test_bounded_command_queue_rejects_when_full():
    commands = ContinuousCommandQueue(max_size=1)

    assert commands.put("向前走一秒").accepted
    rejected = commands.put("左转九十度")

    assert not rejected.accepted
    assert rejected.reason == "queue_full"


def test_command_execution_tracker_reports_queue_events():
    tracker = CommandExecutionTracker(source="online")
    accepted = QueueSnapshot(True, size=2)
    rejected = QueueSnapshot(False, size=2, reason="queue_full")

    enqueue = tracker.queue_event("enqueue", "向前走一秒", accepted)
    queue_full = tracker.queue_event("enqueue", "左转九十度", rejected)
    cleared = tracker.queue_event("clear", "", QueueSnapshot(True, size=0, dropped=2))

    assert enqueue.as_dict() == {
        "event": "enqueue",
        "source": "online",
        "text": "向前走一秒",
        "size": 2,
        "dropped": 0,
        "reason": "",
        "priority_stop": False,
    }
    assert queue_full.as_dict()["event"] == "rejected"
    assert queue_full.as_dict()["reason"] == "queue_full"
    assert cleared.as_dict()["dropped"] == 2


def test_command_execution_tracker_reports_execution_events():
    tracker = CommandExecutionTracker(source="offline")
    item = QueuedCommand("绕圈", created_at=10.0)

    started = tracker.execution_started(item)
    finished = tracker.execution_finished(item, success=True, reason="completed")

    assert started.as_dict()["event"] == "started"
    assert started.as_dict()["text"] == "绕圈"
    assert finished.as_dict()["event"] == "finished"
    assert finished.as_dict()["success"] is True
    assert finished.as_dict()["reason"] == "completed"
