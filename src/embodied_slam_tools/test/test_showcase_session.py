from embodied_slam_tools.showcase_session import (
    SessionCommand,
    SessionPhase,
    ShowcaseSessionStateMachine,
    parse_session_command,
)


def test_parser_recognizes_only_explicit_slam_session_intents():
    assert parse_session_command("保存地图") == SessionCommand.SAVE_MAP
    assert (
        parse_session_command("保存地图，然后开始导航")
        == SessionCommand.SAVE_AND_START_NAVIGATION
    )
    assert parse_session_command("进入导航模式") == SessionCommand.START_NAVIGATION
    assert parse_session_command("结束建图演示") == SessionCommand.STOP_SESSION
    assert parse_session_command("向前走一秒") is None
    assert parse_session_command("你能介绍一下地图吗") is None


def test_state_machine_requires_mapping_then_save_before_navigation():
    machine = ShowcaseSessionStateMachine()
    accepted, reason = machine.validate(SessionCommand.START_NAVIGATION)
    assert not accepted
    assert "save the map" in reason

    machine.transition(SessionPhase.STARTING_MAPPING, detail="starting")
    machine.transition(SessionPhase.MAPPING, detail="ready")
    assert machine.validate(SessionCommand.SAVE_MAP)[0]

    machine.transition(SessionPhase.SAVING_MAP, detail="saving")
    machine.transition(
        SessionPhase.MAP_SAVED,
        detail="saved",
        map_saved=True,
        map_yaml_path="/tmp/map.yaml",
    )
    assert machine.validate(SessionCommand.START_NAVIGATION)[0]
    machine.transition(SessionPhase.NAVIGATING, detail="ready")
    assert machine.validate(SessionCommand.START_NAVIGATION) == (
        True,
        "navigation already active",
    )


def test_busy_transition_rejects_a_second_command_without_losing_map_state():
    machine = ShowcaseSessionStateMachine()
    machine.transition(SessionPhase.MAPPING, detail="ready")
    machine.transition(SessionPhase.SAVING_MAP, detail="saving")
    accepted, reason = machine.validate(SessionCommand.SAVE_MAP)
    assert not accepted
    assert "busy" in reason

    snapshot = machine.transition(
        SessionPhase.FAILED,
        detail="map saver failed",
        map_saved=False,
    )
    assert snapshot.phase == SessionPhase.FAILED
    assert not snapshot.map_saved
