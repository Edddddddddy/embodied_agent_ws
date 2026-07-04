import json

from embodied_online_agent.command_completion import CommandCompleter


def test_completes_missing_motion_slots_for_continuous_demo_commands():
    completer = CommandCompleter(enabled=True)

    assert completer.complete("前进").text == "前进一秒"
    assert completer.complete("向前").text == "前进一秒"
    assert completer.complete("后退").text == "后退一秒"
    assert completer.complete("向后").text == "后退一秒"
    assert completer.complete("左转").text == "左转九十度"
    assert completer.complete("右转").text == "右转九十度"


def test_does_not_complete_full_or_safety_commands():
    completer = CommandCompleter(enabled=True)

    for text in ["前进一秒", "左转九十度", "左转90度", "停下", "急停", "绕圈", "走正方形", "退出控制"]:
        result = completer.complete(text)
        assert result.text == text
        assert not result.changed


def test_completion_feedback_is_monitor_friendly_json():
    result = CommandCompleter(enabled=True).complete("左转")

    payload = json.loads(result.to_feedback_json())

    assert payload == {
        "status": "completed",
        "reason": "completed_missing_slot",
        "original": "左转",
        "completed": "左转九十度",
        "confidence": 1.0,
    }


def test_disabled_completer_leaves_text_unchanged():
    result = CommandCompleter(enabled=False).complete("左转")

    assert result.text == "左转"
    assert not result.changed
