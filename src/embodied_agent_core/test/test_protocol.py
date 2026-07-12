from embodied_agent_core.protocol import SentenceChunker, TaggedStreamParser


def test_parser_streams_speech_and_parses_split_action():
    parser = TaggedStreamParser()
    speech = []
    actions = []
    response = (
        '<speech>好的，马上前进。</speech>'
        '<action>{"name":"move","arguments":{"linear_x":0.2}}</action>'
    )
    for chunk in [response[:5], response[5:17], response[17:41], response[41:]]:
        events = parser.feed(chunk)
        speech.extend(events.speech)
        actions.extend(events.actions)

    assert "".join(speech) == "好的，马上前进。"
    assert len(actions) == 1
    assert actions[0].name == "move"
    assert actions[0].arguments["linear_x"] == 0.2
    assert parser.finish().errors == []


def test_parser_rejects_bad_action_without_crashing():
    parser = TaggedStreamParser()
    events = parser.feed("<action>not-json</action>")
    assert events.actions == []
    assert events.errors


def test_parser_accepts_action_array_for_ordered_demo():
    parser = TaggedStreamParser()
    events = parser.feed(
        '<action>[{"name":"move","arguments":{"linear_x":0.18,"duration_s":1.2}},'
        '{"name":"turn","arguments":{"angular_z":0.6,"duration_s":2.6}}]</action>'
    )
    assert [action.name for action in events.actions] == ["move", "turn"]
    assert events.errors == []


def test_sentence_chunker_flushes_on_punctuation_and_length():
    chunker = SentenceChunker(max_chars=5)
    assert chunker.feed("你好，继续前进") == ["你好，"]
    assert chunker.finish() == ["继续前进"]
