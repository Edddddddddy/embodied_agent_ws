from embodied_online_agent.command_fallback import parse_fallback_action, should_block_model_actions


def test_parses_explicit_motion_and_duration():
    action = parse_fallback_action("向前走一秒")
    assert action.as_dict() == {
        "name": "move", "arguments": {"linear_x": 0.2, "duration_s": 1.0}
    }
    action = parse_fallback_action("后退半秒")
    assert action.arguments["duration_s"] == 0.5
    assert action.arguments["linear_x"] == -0.2


def test_parses_stop_wave_and_led():
    assert parse_fallback_action("马上停下").name == "stop"
    assert parse_fallback_action("挥手三次").arguments["count"] == 3
    assert parse_fallback_action("把灯设为蓝色").arguments["color"] == "blue"
    turn = parse_fallback_action("左转九十度")
    assert turn.arguments == {"angular_z": 0.6, "duration_s": 2.6}


def test_parses_simulation_mode_commands():
    assert parse_fallback_action("开启自动避障").as_dict() == {
        "name": "set_mode",
        "arguments": {"mode": "obstacle_avoidance"},
    }
    assert parse_fallback_action("开始沿墙行走").arguments["mode"] == "wall_following"
    assert parse_fallback_action("退出自动模式").arguments["mode"] == "manual"


def test_rejects_negated_or_ambiguous_text():
    assert parse_fallback_action("不要向前走") is None
    assert parse_fallback_action("你觉得向前意味着什么") is None
    assert parse_fallback_action("一边前进一边高速旋转") is None
    assert parse_fallback_action("不要停下") is None
    assert should_block_model_actions("一边前进一边高速旋转")
    assert should_block_model_actions("今天天气怎么样")
