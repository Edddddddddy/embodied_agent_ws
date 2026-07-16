from embodied_slam_tools.showcase_session import (
    SessionCommand,
    SessionPhase,
    ShowcaseSessionStateMachine,
    exploration_completion_reason,
    is_automatic_mission_cancel_text,
    parse_mapping_bootstrap_route,
    parse_session_command,
)


def test_parser_recognizes_only_explicit_slam_session_intents():
    assert (
        parse_session_command("开始自动巡检建图")
        == SessionCommand.RUN_AUTOMATIC_MISSION
    )
    assert (
        parse_session_command("自动建图并导航")
        == SessionCommand.RUN_AUTOMATIC_MISSION
    )
    assert parse_session_command("保存地图") == SessionCommand.SAVE_MAP
    assert (
        parse_session_command("保存地图，然后开始导航")
        == SessionCommand.SAVE_AND_START_NAVIGATION
    )
    assert parse_session_command("进入导航模式") == SessionCommand.START_NAVIGATION
    assert parse_session_command("结束建图演示") == SessionCommand.STOP_SESSION
    assert parse_session_command("向前走一秒") is None
    assert parse_session_command("你能介绍一下地图吗") is None


def test_automatic_mission_only_starts_from_ready_mapping_phase():
    machine = ShowcaseSessionStateMachine()
    accepted, reason = machine.validate(SessionCommand.RUN_AUTOMATIC_MISSION)
    assert not accepted
    assert "mapping" in reason

    machine.transition(SessionPhase.STARTING_MAPPING, detail="starting")
    machine.transition(SessionPhase.MAPPING, detail="ready")
    assert machine.validate(SessionCommand.RUN_AUTOMATIC_MISSION)[0]

    machine.transition(SessionPhase.AUTOMATIC_MAPPING, detail="exploring")
    accepted, reason = machine.validate(SessionCommand.RUN_AUTOMATIC_MISSION)
    assert not accepted
    assert "busy" in reason

    machine.transition(SessionPhase.MISSION_COMPLETED, detail="done", map_saved=True)
    assert machine.validate(SessionCommand.START_NAVIGATION) == (
        True,
        "navigation already active",
    )


def test_automatic_mission_cancel_phrases_bypass_normal_intent_parsing():
    assert is_automatic_mission_cancel_text("急停！")
    assert is_automatic_mission_cancel_text("请停止自动任务")
    assert is_automatic_mission_cancel_text("取消自动任务")
    assert not is_automatic_mission_cancel_text("保存地图")


def test_exploration_can_finish_by_native_status_or_coverage_plateau():
    values = {
        "completion_status": "exploration_complete",
        "elapsed_s": 60.0,
        "min_runtime_s": 45.0,
        "known_cells": 7000,
        "occupied_cells": 200,
        "min_known_cells": 6000,
        "min_occupied_cells": 150,
        "stable_map_s": 20.0,
    }
    assert exploration_completion_reason(
        status="exploration_complete",
        seconds_since_map_growth=1.0,
        **values,
    ) == "no_frontiers"
    assert exploration_completion_reason(
        status="exploration_in_progress",
        seconds_since_map_growth=21.0,
        **values,
    ) == "coverage_plateau"
    assert exploration_completion_reason(
        status="exploration_in_progress",
        seconds_since_map_growth=2.0,
        **values,
    ) is None


def test_exploration_time_budget_accepts_only_threshold_complete_map():
    values = {
        "status": "exploration_in_progress",
        "completion_status": "exploration_complete",
        "elapsed_s": 300.0,
        "min_runtime_s": 45.0,
        "occupied_cells": 200,
        "min_known_cells": 6000,
        "min_occupied_cells": 150,
        "seconds_since_map_growth": 0.0,
        "stable_map_s": 20.0,
        "time_budget_reached": True,
    }
    assert exploration_completion_reason(
        known_cells=7000,
        **values,
    ) == "time_budget_coverage"
    assert exploration_completion_reason(
        known_cells=5999,
        **values,
    ) is None


def test_mapping_bootstrap_route_accepts_only_safe_motion_primitives():
    route = parse_mapping_bootstrap_route(
        {
            "bootstrap_route": [
                {"label": "离开充电位", "text": "前进三秒", "action": "move"},
                {"label": "转向走廊", "text": "左转九十度", "action": "turn"},
            ]
        }
    )
    assert route == [
        ("离开充电位", "前进三秒", "move"),
        ("转向走廊", "左转九十度", "turn"),
    ]

    try:
        parse_mapping_bootstrap_route(
            {"bootstrap_route": [{"text": "去办公室", "action": "navigate_to"}]}
        )
    except ValueError as exc:
        assert "move/turn" in str(exc)
    else:
        raise AssertionError("unsafe bootstrap action must be rejected")


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
