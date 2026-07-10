from embodied_online_agent.command_nlu import CommandNLU


def action_names(text):
    result = CommandNLU().parse(text)
    return [action.name for command in result.commands for action in command.actions]


def test_nlu_extracts_multiple_commands_with_punctuation():
    result = CommandNLU().parse("向右转，向前走一秒")

    assert result.accepted
    assert [command.intent for command in result.commands] == [
        "turn_right",
        "move_forward",
    ]
    assert [action.name for command in result.commands for action in command.actions] == [
        "turn",
        "move",
    ]
    assert result.commands[0].actions[0].arguments == {
        "angular_z": -0.6,
        "duration_s": 2.6,
    }
    assert result.commands[1].actions[0].arguments == {
        "linear_x": 0.2,
        "duration_s": 1.0,
    }


def test_nlu_extracts_multiple_commands_with_connectors():
    assert action_names("向右转然后向前走一秒") == ["turn", "move"]
    assert action_names("向前走一秒再左转九十度然后后退一秒") == [
        "move",
        "turn",
        "move",
    ]


def test_nlu_extracts_commands_without_punctuation():
    assert action_names("向右转向前走一秒") == ["turn", "move"]


def test_nlu_keeps_composite_commands_atomic():
    square = CommandNLU().parse("走正方形")
    assert len(square.commands) == 1
    assert [action.name for action in square.commands[0].actions] == [
        "move",
        "turn",
        "move",
        "turn",
        "move",
        "turn",
        "move",
        "turn",
    ]


def test_nlu_prioritizes_stop_and_blocks_unsafe_language():
    stop = CommandNLU().parse("停下然后向前走")
    assert [action.name for command in stop.commands for action in command.actions] == [
        "stop"
    ]

    assert not CommandNLU().parse("不要向前走").accepted
    assert not CommandNLU().parse("你觉得向前是什么意思").accepted
    assert not CommandNLU().parse("一边前进一边高速旋转").accepted
    assert not CommandNLU().parse("高速前进").accepted
    assert not CommandNLU().parse("全速冲过去").accepted


def test_nlu_extracts_navigation_and_waypoint_patrol():
    nav = CommandNLU().parse("去门口")
    assert nav.accepted
    assert nav.commands[0].intent == "navigate_to"
    assert nav.commands[0].actions[0].as_dict() == {
        "name": "navigate_to",
        "arguments": {"target": "door"},
    }

    patrol = CommandNLU().parse("依次去门口、书桌、起点")
    assert patrol.accepted
    assert patrol.commands[0].intent == "follow_waypoints"
    assert patrol.commands[0].actions[0].as_dict() == {
        "name": "follow_waypoints",
        "arguments": {"waypoints": ["door", "desk", "home"], "number_of_loops": 1},
    }

    cancel = CommandNLU().parse("取消导航然后去门口")
    assert [action.name for command in cancel.commands for action in command.actions] == [
        "cancel_navigation"
    ]

    unreachable = CommandNLU().parse("去封闭区")
    assert unreachable.commands[0].actions[0].arguments == {
        "target": "unreachable_zone"
    }
    assert unreachable.commands[0].slots == {"place": "unreachable_zone"}


def test_nlu_treats_natural_multi_target_navigation_as_waypoint_patrol():
    for text in (
        "先去门口再去书桌最后回起点",
        "去门口然后前往书桌最后返回起点",
        "巡逻门口、书桌、起点",
    ):
        result = CommandNLU().parse(text)

        assert result.accepted
        assert result.commands[0].intent == "follow_waypoints"
        assert result.commands[0].actions[0].as_dict() == {
            "name": "follow_waypoints",
            "arguments": {"waypoints": ["door", "desk", "home"], "number_of_loops": 1},
        }


def test_nlu_keeps_two_target_navigation_as_separate_queue_items():
    result = CommandNLU().parse("去门口，然后前往书桌")

    assert result.accepted
    assert [command.intent for command in result.commands] == [
        "navigate_to",
        "navigate_to",
    ]
    assert [command.actions[0].arguments["target"] for command in result.commands] == [
        "door",
        "desk",
    ]


def test_nlu_extracts_distance_and_speed_slots_into_safe_motion_time():
    result = CommandNLU().parse("以每秒零点二米向前走一米")

    assert result.accepted
    command = result.commands[0]
    assert command.intent == "move_forward"
    assert command.slots == {
        "direction": "forward",
        "distance_m": 1.0,
        "speed_mps": 0.2,
        "duration_s": 5.0,
    }
    assert command.actions[0].arguments == {
        "linear_x": 0.2,
        "duration_s": 5.0,
    }


def test_nlu_extracts_arbitrary_turn_angle_slot():
    result = CommandNLU().parse("向右转四十五度")

    assert result.accepted
    command = result.commands[0]
    assert command.slots == {
        "direction": "right",
        "angle_deg": 45.0,
        "angular_speed_rps": 0.6,
        "duration_s": 1.309,
    }
    assert command.actions[0].arguments == {
        "angular_z": -0.6,
        "duration_s": 1.309,
    }


def test_nlu_exposes_duration_and_place_slots():
    motion = CommandNLU().parse("后退两秒").commands[0]
    place = CommandNLU().parse("去门口").commands[0]

    assert motion.slots == {
        "direction": "backward",
        "speed_mps": 0.2,
        "duration_s": 2.0,
    }
    assert place.slots == {"place": "door"}


def test_nlu_requests_repetition_when_real_asr_drops_a_required_slot():
    missing_color = CommandNLU().parse("把灯")
    missing_direction = CommandNLU().parse("我九十")

    assert not missing_color.accepted
    assert missing_color.reason == "missing_led_color"
    assert "颜色" in missing_color.retry_prompt
    assert not missing_direction.accepted
    assert missing_direction.reason == "missing_turn_direction"
    assert "左转或右转" in missing_direction.retry_prompt


def test_nlu_does_not_turn_normal_chat_into_a_slot_retry():
    result = CommandNLU().parse("今天天气怎么样")

    assert not result.accepted
    assert result.reason == "blocked_semantic"
    assert result.retry_prompt == ""


def test_nlu_distance_conversion_respects_motion_limits_without_silent_truncation():
    long_default = CommandNLU().parse("向前走五米")
    impossible_explicit = CommandNLU().parse("以每秒零点一米向前走两米")

    assert long_default.accepted
    assert long_default.commands[0].actions[0].arguments == {
        "linear_x": 0.5,
        "duration_s": 10.0,
    }
    assert impossible_explicit.accepted
    command = impossible_explicit.commands[0]
    assert command.slots == {
        "direction": "forward",
        "distance_m": 2.0,
        "speed_mps": 0.1,
        "duration_s": 20.0,
        "segment_count": 2,
    }
    assert [action.arguments for action in command.actions] == [
        {"linear_x": 0.1, "duration_s": 10.0},
        {"linear_x": 0.1, "duration_s": 10.0},
    ]
