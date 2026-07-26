import pytest

from embodied_agent_core.streaming_turn import StreamingTurnRuntime


def _runtime(events):
    return StreamingTurnRuntime(
        max_chunk_chars=32,
        on_first_token=lambda: events.append(("first", "")),
        on_speech_delta=lambda text: events.append(("delta", text)),
        on_speakable=lambda text: events.append(("speak", text)),
        on_protocol_error=lambda error: events.append(("error", error)),
    )


def test_streams_speech_and_selects_deterministic_action_over_model_action():
    events = []
    runtime = _runtime(events)
    runtime.feed('<speech>好的，正在前进。</speech><action>{"name":"stop"}</action>')

    result = runtime.finish("向前走一秒")

    assert [action.name for action in result.actions] == ["move"]
    assert [action.name for action in result.model_actions] == ["stop"]
    assert result.action_source == "deterministic"
    assert result.assistant_text == "好的，正在前进。"
    assert ("speak", "好的，") in events
    assert events[0] == ("first", "")


def test_semantic_safety_blocks_model_action_and_memory_result_contains_no_action():
    events = []
    runtime = _runtime(events)
    runtime.feed('<speech>不能执行。</speech><action>{"name":"move"}</action>')

    result = runtime.finish("你能不能向前走")

    assert result.actions == ()
    assert [action.name for action in result.model_actions] == ["move"]
    assert result.action_source == "blocked"


def test_invalid_protocol_emits_fallback_speech_and_sanitized_model_output():
    events = []
    runtime = _runtime(events)
    runtime.feed('<action>{"name":"move"')

    result = runtime.finish("随便聊聊")

    assert result.actions == ()
    assert result.protocol_errors == ("incomplete tagged response",)
    assert result.assistant_text == StreamingTurnRuntime.FORMAT_ERROR_SPEECH
    assert result.model_output == (
        f"<speech>{StreamingTurnRuntime.FORMAT_ERROR_SPEECH}</speech>"
    )
    assert any(kind == "error" for kind, _value in events)


def test_model_action_is_selected_when_no_deterministic_or_safety_policy_matches():
    runtime = _runtime([])
    runtime.feed(
        '<speech>好的。</speech>'
        '<action>{"name":"wave","arguments":{"count":2}}</action>'
    )

    result = runtime.finish("请做刚才约定的动作")

    assert result.action_source == "model"
    assert result.actions[0].name == "wave"
    assert result.actions[0].arguments == {"count": 2}


def test_untrusted_rag_turn_blocks_even_well_formed_model_actions():
    runtime = _runtime([])
    runtime.feed(
        '<speech>资料中这样写。</speech>'
        '<action>{"name":"move","arguments":{"linear_x":0.2,"duration_s":1}}</action>'
    )

    result = runtime.finish("如何部署机器人？", allow_actions=False)

    assert result.actions == ()
    assert [action.name for action in result.model_actions] == ["move"]
    assert result.action_source == "context_blocked"


def test_finished_runtime_rejects_additional_input():
    runtime = _runtime([])
    runtime.feed("<speech>完成。</speech>")
    runtime.finish("完成")

    with pytest.raises(RuntimeError, match="already finished"):
        runtime.feed("more")
