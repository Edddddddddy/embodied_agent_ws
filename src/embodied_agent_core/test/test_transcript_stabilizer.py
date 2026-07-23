from embodied_agent_core.transcript_stabilizer import TranscriptStabilizer


def test_recovers_safe_command_slot_tail_from_latest_partial():
    now = [10.0]
    stabilizer = TranscriptStabilizer(clock=lambda: now[0])

    stabilizer.observe_partial("左转九十度")
    result = stabilizer.finalize("左转")

    assert result.text == "左转九十度"
    assert result.recovered
    assert result.reason == "partial_slot_tail_recovered"


def test_recovers_time_color_and_navigation_slot_tails():
    cases = [
        ("前进一秒", "前进"),
        ("把灯设成蓝色", "把灯"),
        ("取消导航", "取消"),
        ("退出控制", "退出"),
        ("挥手两次", "挥手"),
    ]

    for partial, final in cases:
        stabilizer = TranscriptStabilizer()
        stabilizer.observe_partial(partial)
        assert stabilizer.finalize(final).text == partial


def test_recovers_automatic_mapping_tail_only_when_partial_observed_it():
    stabilizer = TranscriptStabilizer()
    stabilizer.observe_partial("开始自动巡检建图")

    result = stabilizer.finalize("开始自动")

    assert result.text == "开始自动巡检建图"
    assert result.reason == "partial_slot_tail_recovered"


def test_does_not_restore_arbitrary_chat_tail():
    stabilizer = TranscriptStabilizer()
    stabilizer.observe_partial("今天天气很热")

    result = stabilizer.finalize("今天天气")

    assert result.text == "今天天气"
    assert not result.recovered


def test_contradictory_partial_replaces_old_candidate():
    stabilizer = TranscriptStabilizer()
    stabilizer.observe_partial("左转九十度")
    stabilizer.observe_partial("右转")

    result = stabilizer.finalize("右转")

    assert result.text == "右转"
    assert not result.recovered


def test_stale_partial_and_previous_utterance_are_never_reused():
    now = [10.0]
    stabilizer = TranscriptStabilizer(max_age_s=1.0, clock=lambda: now[0])
    stabilizer.observe_partial("前进一秒")
    now[0] += 1.1

    assert stabilizer.finalize("前进").text == "前进"

    stabilizer.observe_partial("左转九十度")
    assert stabilizer.finalize("左转九十度").text == "左转九十度"
    # finalize 后必须清空 utterance 状态，下一句话不能继承上句 partial。
    assert stabilizer.finalize("左转").text == "左转"


def test_disabled_stabilizer_only_clears_state():
    stabilizer = TranscriptStabilizer(enabled=False)
    stabilizer.observe_partial("把灯设成蓝色")

    result = stabilizer.finalize("把灯")

    assert result.text == "把灯"
    assert not result.recovered
