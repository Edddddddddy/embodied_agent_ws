"""Shared typed-action helpers for standalone ROS integration probes."""

from embodied_agent_interfaces.msg import RobotCommand, RobotCommandResult
from embodied_agent_core.ros_action_transport import (
    action_command_to_message,
    command_message_to_dict,
    result_message_to_dict,
)
from embodied_agent_core.types import ActionCommand


def candidate_dict(message: RobotCommand) -> dict:
    return command_message_to_dict(message)


def result_dict(message: RobotCommandResult) -> dict:
    return result_message_to_dict(message)


def candidate_message(
    name: str,
    arguments: dict,
    *,
    request_id: str = "integration-command",
    source: str = "integration_test",
    priority: bool = False,
) -> RobotCommand:
    return action_command_to_message(
        ActionCommand(name, arguments, request_id, priority=priority), source=source
    )
