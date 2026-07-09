import pytest

from embodied_online_agent.types import ActionCommand
from embodied_online_agent.user_preferences import apply_user_preferences


def _single(action, preferences):
    return apply_user_preferences([action], preferences)[0]


def test_slow_speed_preference_lowers_move_speed_without_mutating_input():
    original = ActionCommand("move", {"linear_x": 0.2, "duration_s": 1.0})

    adjusted = _single(original, {"movement_speed": "slow"})

    assert adjusted.arguments == {"linear_x": 0.15, "duration_s": 1.0}
    assert original.arguments == {"linear_x": 0.2, "duration_s": 1.0}


def test_fast_speed_preference_is_modest_and_preserves_direction():
    forward = _single(
        ActionCommand("move", {"linear_x": 0.2, "duration_s": 1.0}),
        {"movement_speed": "fast"},
    )
    backward = _single(
        ActionCommand("move", {"linear_x": -0.2, "duration_s": 1.0}),
        {"movement_speed": "fast"},
    )

    assert forward.arguments["linear_x"] == pytest.approx(0.23)
    assert backward.arguments["linear_x"] == pytest.approx(-0.23)


def test_default_move_duration_only_rewrites_default_duration():
    default_move = _single(
        ActionCommand("move", {"linear_x": 0.2, "duration_s": 1.0}),
        {"default_move_duration_s": 2.0},
    )
    explicit_move = _single(
        ActionCommand("move", {"linear_x": 0.2, "duration_s": 3.0}),
        {"default_move_duration_s": 2.0},
    )

    assert default_move.arguments["duration_s"] == 2.0
    assert explicit_move.arguments["duration_s"] == 3.0


def test_default_turn_degrees_rewrites_default_ninety_degree_turn():
    adjusted = _single(
        ActionCommand("turn", {"angular_z": 0.6, "duration_s": 2.6}),
        {"default_turn_degrees": 45},
    )

    assert adjusted.arguments["angular_z"] == 0.6
    assert adjusted.arguments["duration_s"] == pytest.approx(1.3)


def test_slow_turn_keeps_default_angle_by_extending_duration():
    adjusted = _single(
        ActionCommand("turn", {"angular_z": -0.6, "duration_s": 2.6}),
        {"movement_speed": "slow"},
    )

    assert adjusted.arguments["angular_z"] == pytest.approx(-0.45)
    assert adjusted.arguments["duration_s"] == pytest.approx(3.467)


def test_slow_turn_with_default_degrees_preserves_preferred_angle():
    adjusted = _single(
        ActionCommand("turn", {"angular_z": 0.6, "duration_s": 2.6}),
        {"movement_speed": "slow", "default_turn_degrees": 45},
    )

    assert adjusted.arguments["angular_z"] == pytest.approx(0.45)
    assert adjusted.arguments["duration_s"] == pytest.approx(1.733)


def test_arc_slow_preference_preserves_shape_with_longer_duration():
    adjusted = _single(
        ActionCommand("arc", {"linear_x": 0.12, "angular_z": 0.45, "duration_s": 6.0}),
        {"movement_speed": "slow"},
    )

    assert adjusted.arguments["linear_x"] == pytest.approx(0.09)
    assert adjusted.arguments["angular_z"] == pytest.approx(0.3375)
    assert adjusted.arguments["duration_s"] == pytest.approx(8.0)


def test_stop_and_cancel_are_not_rewritten_by_preferences():
    stop = ActionCommand("stop", {})
    cancel = ActionCommand("cancel_navigation", {})

    assert _single(stop, {"movement_speed": "fast"}) is stop
    assert _single(cancel, {"default_turn_degrees": 45}) is cancel
