from embodied_online_agent.wake_provider import TextWakeProvider, WakeEventKind


def test_text_wake_provider_reports_wake_event_and_remainder():
    now = [10.0]
    provider = TextWakeProvider(
        ["小智"],
        aliases=["小志"],
        enabled=True,
        active_timeout_s=60.0,
        clock=lambda: now[0],
    )

    decision = provider.accept("小志，向前走一秒")

    assert decision.command == "向前走一秒"
    assert decision.event.kind == WakeEventKind.WAKE
    assert decision.event.provider == "text"
    assert decision.active


def test_text_wake_provider_reports_continue_inside_active_session():
    now = [10.0]
    provider = TextWakeProvider(["小智"], enabled=True, active_timeout_s=60.0, clock=lambda: now[0])

    assert provider.accept("小智").event.kind == WakeEventKind.WAKE
    decision = provider.accept("左转九十度")

    assert decision.command == "左转九十度"
    assert decision.event.kind == WakeEventKind.CONTINUE
    assert decision.active


def test_text_wake_provider_reports_rejected_when_asleep():
    provider = TextWakeProvider(["小智"], enabled=True, active_timeout_s=60.0)

    decision = provider.accept("向前走一秒")

    assert decision.command is None
    assert decision.event.kind == WakeEventKind.REJECTED
    assert not decision.active


def test_text_wake_provider_can_sleep_current_session():
    provider = TextWakeProvider(["小智"], enabled=True, active_timeout_s=60.0)
    provider.accept("小智")

    event = provider.sleep()
    decision = provider.accept("向前走一秒")

    assert event.kind == WakeEventKind.SLEEP
    assert not provider.active
    assert decision.event.kind == WakeEventKind.REJECTED


def test_text_wake_provider_accepts_external_wake_event():
    now = [10.0]
    provider = TextWakeProvider(["小智"], enabled=True, active_timeout_s=60.0, clock=lambda: now[0])

    event = provider.external_wake("sherpa_kws", transcript="小智")
    decision = provider.accept("向前走一秒")

    assert event.kind == WakeEventKind.WAKE
    assert event.provider == "sherpa_kws"
    assert decision.command == "向前走一秒"
    assert decision.event.kind == WakeEventKind.CONTINUE
    assert provider.active


def test_disabled_text_wake_provider_is_always_active():
    provider = TextWakeProvider(["小智"], enabled=False, active_timeout_s=60.0)

    decision = provider.accept("向前走一秒")

    assert decision.command == "向前走一秒"
    assert decision.event.kind == WakeEventKind.CONTINUE
    assert decision.active
    assert provider.active
