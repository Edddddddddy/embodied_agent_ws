import pytest

from embodied_online_agent.actions import ActionValidator
from embodied_online_agent.types import ActionCommand


def test_action_values_are_clamped():
    result = ActionValidator().validate(
        ActionCommand("move", {"linear_x": 9.0, "duration_s": 20.0})
    )
    assert result.arguments == {"linear_x": 0.5, "duration_s": 10.0}


def test_unknown_action_is_rejected():
    with pytest.raises(ValueError):
        ActionValidator().validate(ActionCommand("launch_missile", {}))

