from embodied_agent_core.command_fallback import (
    parse_fallback_action,
    parse_fallback_actions,
    should_block_model_actions,
)


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
    spin = parse_fallback_action("原地转一圈")
    assert spin.name == "turn"
    assert spin.arguments == {"angular_z": 0.8, "duration_s": 7.85}
    arc = parse_fallback_action("绕圈演示")
    assert arc.name == "arc"
    assert arc.arguments == {"linear_x": 0.12, "angular_z": 0.45, "duration_s": 6.0}


def test_bare_direction_commands_use_safe_demo_defaults():
    assert parse_fallback_action("前进").as_dict() == {
        "name": "move",
        "arguments": {"linear_x": 0.2, "duration_s": 1.0},
    }
    assert parse_fallback_action("后退").as_dict() == {
        "name": "move",
        "arguments": {"linear_x": -0.2, "duration_s": 1.0},
    }
    assert parse_fallback_action("左转").as_dict() == {
        "name": "turn",
        "arguments": {"angular_z": 0.6, "duration_s": 2.6},
    }
    assert parse_fallback_action("右转").as_dict() == {
        "name": "turn",
        "arguments": {"angular_z": -0.6, "duration_s": 2.6},
    }


def test_parses_simulation_mode_commands():
    assert parse_fallback_action("开启自动避障").as_dict() == {
        "name": "set_mode",
        "arguments": {"mode": "obstacle_avoidance"},
    }
    assert parse_fallback_action("开始沿墙行走").arguments["mode"] == "wall_following"
    assert parse_fallback_action("退出自动模式").arguments["mode"] == "manual"


def test_parses_composite_demo_sequences():
    square = parse_fallback_actions("走正方形")
    assert [action.name for action in square] == [
        "move", "turn", "move", "turn", "move", "turn", "move", "turn"
    ]
    assert square[0].arguments == {"linear_x": 0.18, "duration_s": 1.2}
    assert square[1].arguments == {"angular_z": 0.6, "duration_s": 2.6}

    demo = parse_fallback_actions("演示一下")
    assert [action.name for action in demo] == [
        "set_led", "wave", "move", "turn", "arc", "stop"
    ]


def test_parses_navigation_and_patrol_commands():
    assert parse_fallback_action("去门口").as_dict() == {
        "name": "navigate_to",
        "arguments": {"target": "door"},
    }
    assert parse_fallback_action("回到起点").arguments["target"] == "home"

    patrol = parse_fallback_action("开始巡航")
    assert patrol.name == "follow_waypoints"
    assert patrol.arguments == {
        "waypoints": ["door", "desk", "home"],
        "number_of_loops": 1,
    }

    sequence = parse_fallback_action("依次去门口、书桌、起点")
    assert sequence.name == "follow_waypoints"
    assert sequence.arguments["waypoints"] == ["door", "desk", "home"]
    natural_sequence = parse_fallback_action("先去门口再去书桌最后回起点")
    assert natural_sequence.name == "follow_waypoints"
    assert natural_sequence.arguments["waypoints"] == ["door", "desk", "home"]
    patrol_places = parse_fallback_action("巡逻门口、书桌、起点")
    assert patrol_places.name == "follow_waypoints"
    assert patrol_places.arguments["waypoints"] == ["door", "desk", "home"]
    assert parse_fallback_action("取消导航").name == "cancel_navigation"


def test_rejects_negated_or_ambiguous_text():
    assert parse_fallback_action("不要向前走") is None
    assert parse_fallback_action("你觉得向前意味着什么") is None
    assert parse_fallback_action("一边前进一边高速旋转") is None
    assert parse_fallback_action("不要停下") is None
    assert should_block_model_actions("一边前进一边高速旋转")
    assert should_block_model_actions("今天天气怎么样")
