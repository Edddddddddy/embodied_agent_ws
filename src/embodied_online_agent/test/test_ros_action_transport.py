import pytest
from embodied_agent_interfaces.msg import RobotCommand, RobotCommandResult

from embodied_online_agent.ros_action_transport import (
    action_command_to_message,
    command_message_to_dict,
    result_message_to_dict,
)
from embodied_online_agent.types import ActionCommand


@pytest.mark.parametrize(
    ("action", "expected_type"),
    [
        (ActionCommand("stop", {}, "stop-1"), RobotCommand.STOP),
        (
            ActionCommand(
                "move", {"linear_x": 0.2, "duration_s": 1.0}, "move-1"
            ),
            RobotCommand.MOVE,
        ),
        (
            ActionCommand(
                "arc",
                {"linear_x": 0.1, "angular_z": 0.4, "duration_s": 2.0},
                "arc-1",
            ),
            RobotCommand.ARC,
        ),
        (
            ActionCommand(
                "turn", {"angular_z": -0.6, "duration_s": 2.6}, "turn-1"
            ),
            RobotCommand.TURN,
        ),
        (ActionCommand("wave", {"count": 2}, "wave-1"), RobotCommand.WAVE),
        (
            ActionCommand("set_led", {"color": "blue"}, "led-1"),
            RobotCommand.SET_LED,
        ),
        (
            ActionCommand("set_mode", {"mode": "manual"}, "mode-1"),
            RobotCommand.SET_MODE,
        ),
        (
            ActionCommand("navigate_to", {"target": "door"}, "nav-1"),
            RobotCommand.NAVIGATE_TO,
        ),
        (
            ActionCommand(
                "follow_waypoints",
                {"waypoints": ["door", "desk"], "number_of_loops": 2},
                "patrol-1",
            ),
            RobotCommand.FOLLOW_WAYPOINTS,
        ),
        (
            ActionCommand("cancel_navigation", {}, "cancel-1"),
            RobotCommand.CANCEL_NAVIGATION,
        ),
    ],
)
def test_action_command_maps_to_typed_ros_message(action, expected_type):
    message = action_command_to_message(action, source="offline_agent")

    assert message.action_type == expected_type
    assert message.command_id == action.request_id
    assert message.source == "offline_agent"
    assert message.priority is action.priority
    assert command_message_to_dict(message)["request_id"] == action.request_id


def test_priority_stop_metadata_crosses_typed_ros_seam():
    action = ActionCommand("stop", {}, "urgent-stop", priority=True)

    message = action_command_to_message(action, source="online_agent")

    assert message.action_type == RobotCommand.STOP
    assert message.priority is True
    assert command_message_to_dict(message)["priority"] is True


def test_unsupported_action_is_rejected_before_ros_publish():
    with pytest.raises(ValueError, match="unsupported ROS action candidate"):
        action_command_to_message(ActionCommand("dance", {}, "dance-1"), source="agent")


def test_typed_result_is_only_converted_for_observability():
    message = RobotCommandResult()
    message.command_id = "move-1"
    message.success = True
    message.status = RobotCommandResult.STATUS_SUCCEEDED
    message.message = "succeeded"

    assert result_message_to_dict(message) == {
        "command_id": "move-1",
        "success": True,
        "status": RobotCommandResult.STATUS_SUCCEEDED,
        "message": "succeeded",
    }
