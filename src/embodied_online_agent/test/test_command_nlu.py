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
