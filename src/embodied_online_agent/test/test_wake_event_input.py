from embodied_online_agent.wake_event_input import parse_external_wake_event


def test_parses_json_wake_event_for_kws_sidecar():
    event = parse_external_wake_event(
        '{"kind": "wake", "provider": "sherpa_kws", "transcript": "小智"}'
    )

    assert event is not None
    assert event.kind == "wake"
    assert event.provider == "sherpa_kws"
    assert event.transcript == "小智"


def test_parses_plain_text_sleep_for_manual_debugging():
    event = parse_external_wake_event("sleep")

    assert event is not None
    assert event.kind == "sleep"
    assert event.provider == "external"


def test_rejects_unknown_external_wake_payloads():
    assert parse_external_wake_event("") is None
    assert parse_external_wake_event("hello") is None
    assert parse_external_wake_event('{"kind": "noise"}') is None
