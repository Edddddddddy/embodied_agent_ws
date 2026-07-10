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


def test_continuous_session_ignores_short_asr_fillers():
    gate = WakeWordGate(["小智"], enabled=True, active_timeout_s=60.0)
    session = ContinuousVoiceSession(gate, enabled=True)

    filler = session.accept("嗯。")

    assert not filler.accepted
    assert filler.reason == "filler"
    assert filler.event.kind == SessionEventKind.REJECTED


def test_continuous_session_deduplicates_repeated_asr_finals_briefly():
    now = [100.0]
    gate = WakeWordGate(
        ["小智"],
        enabled=True,
        active_timeout_s=60.0,
        clock=lambda: now[0],
    )
    session = ContinuousVoiceSession(
        gate,
        enabled=True,
        duplicate_window_s=1.2,
        clock=lambda: now[0],
    )
    session.accept("小智")

    first = session.accept("向前走一秒")
    now[0] += 0.5
    duplicate = session.accept("向前走一秒。")
    now[0] += 1.3
    repeated_later = session.accept("向前走一秒")

    assert first.accepted
    assert not duplicate.accepted
    assert duplicate.reason == "duplicate_command"
    assert repeated_later.accepted


def test_continuous_session_allows_same_command_after_sleep_and_rewake():
    now = [100.0]
    gate = WakeWordGate(
        ["小智"],
        enabled=True,
        active_timeout_s=60.0,
        clock=lambda: now[0],
    )
    session = ContinuousVoiceSession(
        gate,
        enabled=True,
        duplicate_window_s=5.0,
        clock=lambda: now[0],
    )

    assert session.accept("小智").event.kind == SessionEventKind.WAKE
    first = session.accept("向前走一秒")
    sleeping = session.accept("退出控制")
    now[0] += 0.2
    rewake = session.accept("小智")
    repeated_in_new_session = session.accept("向前走一秒")

    assert first.accepted
    assert sleeping.event.kind == SessionEventKind.SLEEP
    assert rewake.event.kind == SessionEventKind.WAKE
    assert repeated_in_new_session.accepted
    assert repeated_in_new_session.command == "向前走一秒"


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


def test_exact_short_exit_closes_session_without_matching_longer_command():
    """ASR 可能只提交“退出”；但“退出自动模式”仍应作为机器人命令处理。"""
    gate = WakeWordGate(["小智"], enabled=True, active_timeout_s=60.0)
    session = ContinuousVoiceSession(gate, enabled=True)

    assert session.accept("小智").event.kind == SessionEventKind.WAKE
    sleeping = session.accept("退出。")
    assert sleeping.event.kind == SessionEventKind.SLEEP
    assert sleeping.event.session_state == "sleeping"

    assert session.accept("小智").event.kind == SessionEventKind.WAKE
    mode_command = session.accept("退出自动模式")
    assert mode_command.accepted
    assert mode_command.command == "退出自动模式"


def test_continuous_session_reports_timeout_after_previous_wake():
    now = [100.0]
    gate = WakeWordGate(
        ["小智"],
        enabled=True,
        active_timeout_s=1.0,
        clock=lambda: now[0],
    )
    session = ContinuousVoiceSession(gate, enabled=True)

    session.accept("小智")
    now[0] += 1.2
    timed_out = session.accept("向前走一秒")
    second_rejection = session.accept("左转九十度")

    assert not timed_out.accepted
    assert timed_out.reason == "session_timeout"
    assert timed_out.event.kind == SessionEventKind.REJECTED
    assert second_rejection.reason == "wake_word_not_detected"


def test_external_wake_event_opens_same_continuous_session():
    gate = WakeWordGate(["小智"], enabled=True, active_timeout_s=60.0)
    session = ContinuousVoiceSession(gate, enabled=True)

    wake_event = session.external_wake("sherpa_kws", transcript="小智")
    command = session.accept("向前走一秒")
    sleep_event = session.external_sleep("sherpa_kws")
    rejected = session.accept("左转九十度")

    assert wake_event.kind == SessionEventKind.WAKE
    assert wake_event.wake_event.provider == "sherpa_kws"
    assert command.accepted
    assert command.command == "向前走一秒"
    assert sleep_event.kind == SessionEventKind.SLEEP
    assert rejected.reason == "wake_word_not_detected"


def test_external_sleep_and_rewake_allow_same_command_again():
    now = [100.0]
    gate = WakeWordGate(
        ["小智"],
        enabled=True,
        active_timeout_s=60.0,
        clock=lambda: now[0],
    )
    session = ContinuousVoiceSession(
        gate,
        enabled=True,
        duplicate_window_s=5.0,
        clock=lambda: now[0],
    )

    session.external_wake("sherpa_kws", transcript="小智")
    first = session.accept("向前走一秒")
    sleep_event = session.external_sleep("sherpa_kws")
    now[0] += 0.2
    session.external_wake("sherpa_kws", transcript="小智")
    repeated_in_new_session = session.accept("向前走一秒")

    assert first.accepted
    assert sleep_event.kind == SessionEventKind.SLEEP
    assert repeated_in_new_session.accepted
    assert repeated_in_new_session.command == "向前走一秒"


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


def test_command_queue_skips_stale_normal_commands():
    now = [10.0]
    stale = []
    commands = ContinuousCommandQueue(max_size=4, max_age_s=2.0, clock=lambda: now[0])
    assert commands.put("向前走一秒").accepted
    now[0] += 3.0
    assert commands.put("左转九十度").accepted

    item = commands.get(timeout=0.01, on_stale=stale.append)

    assert item.text == "左转九十度"
    assert item.created_at == 13.0
    assert [item.text for item in stale] == ["向前走一秒"]


def test_command_queue_never_skips_stale_priority_stop():
    now = [10.0]
    commands = ContinuousCommandQueue(max_size=4, max_age_s=2.0, clock=lambda: now[0])
    assert commands.put("停下", priority_stop=True).accepted
    now[0] += 30.0

    item = commands.get(timeout=0.01)

    assert item.text == "停下"
    assert item.priority_stop


def test_command_execution_tracker_reports_queue_events():
    tracker = CommandExecutionTracker(source="online")
    accepted = QueueSnapshot(True, size=2)
    rejected = QueueSnapshot(False, size=2, reason="queue_full")

    enqueue = tracker.queue_event("enqueue", "向前走一秒", accepted)
    queue_full = tracker.queue_event("enqueue", "左转九十度", rejected)
    cleared = tracker.queue_event("clear", "", QueueSnapshot(True, size=0, dropped=2))
    expired = tracker.queue_expired(QueuedCommand("向前走一秒", created_at=10.0), size=1)

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
    assert expired.as_dict()["event"] == "expired"
    assert expired.as_dict()["text"] == "向前走一秒"
    assert expired.as_dict()["reason"] == "stale_command"


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
