"""领域动作与 ROS 2 强类型消息之间的唯一转换 seam。"""

from __future__ import annotations

from embodied_agent_interfaces.msg import (
    RobotCommand,
    RobotCommandFeedback,
    RobotCommandResult,
)

from .types import ActionCommand


_ACTION_TYPE_NAMES = {
    RobotCommand.STOP: "stop",
    RobotCommand.MOVE: "move",
    RobotCommand.TURN: "turn",
    RobotCommand.WAVE: "wave",
    RobotCommand.SET_LED: "set_led",
    RobotCommand.SET_MODE: "set_mode",
    RobotCommand.NAVIGATE_TO: "navigate_to",
    RobotCommand.FOLLOW_WAYPOINTS: "follow_waypoints",
    RobotCommand.CANCEL_NAVIGATION: "cancel_navigation",
    RobotCommand.ARC: "arc",
}


def action_command_to_message(action: ActionCommand, *, source: str) -> RobotCommand:
    """把解析层动作映射为候选消息；安全限幅仍由 C++ ActionGuard 完成。"""

    message = RobotCommand()
    message.command_id = action.request_id
    message.source = source
    message.priority = action.priority
    arguments = action.arguments
    if action.name == "move":
        message.action_type = RobotCommand.MOVE
        message.linear_x = float(arguments["linear_x"])
        message.angular_z = float(arguments.get("angular_z", 0.0))
        message.duration_s = float(arguments["duration_s"])
    elif action.name == "arc":
        # 候选层保留 arc 意图，C++ ActionGuard 校验后再规范化成可执行 MOVE。
        message.action_type = RobotCommand.ARC
        message.linear_x = float(arguments["linear_x"])
        message.angular_z = float(arguments["angular_z"])
        message.duration_s = float(arguments["duration_s"])
    elif action.name == "turn":
        message.action_type = RobotCommand.TURN
        message.angular_z = float(arguments["angular_z"])
        message.duration_s = float(arguments["duration_s"])
    elif action.name == "stop":
        message.action_type = RobotCommand.STOP
    elif action.name == "wave":
        message.action_type = RobotCommand.WAVE
        message.count = int(arguments["count"])
    elif action.name == "set_led":
        message.action_type = RobotCommand.SET_LED
        message.color = str(arguments["color"])
    elif action.name == "set_mode":
        message.action_type = RobotCommand.SET_MODE
        message.mode = str(arguments["mode"])
    elif action.name == "navigate_to":
        message.action_type = RobotCommand.NAVIGATE_TO
        message.target = str(arguments["target"])
    elif action.name == "follow_waypoints":
        message.action_type = RobotCommand.FOLLOW_WAYPOINTS
        message.waypoints = [str(item) for item in arguments["waypoints"]]
        message.number_of_loops = int(arguments.get("number_of_loops", 1))
    elif action.name == "cancel_navigation":
        message.action_type = RobotCommand.CANCEL_NAVIGATION
    else:
        raise ValueError(f"unsupported ROS action candidate: {action.name}")
    return message


def command_message_to_dict(message: RobotCommand) -> dict:
    """仅用于日志和报告展示；机器人控制本身不再依赖 JSON 字符串。"""

    name = _ACTION_TYPE_NAMES.get(message.action_type, "unknown")
    arguments: dict = {}
    if name in {"move", "arc"}:
        arguments = {
            "linear_x": message.linear_x,
            "duration_s": message.duration_s,
        }
        if message.angular_z:
            arguments["angular_z"] = message.angular_z
    elif name == "turn":
        arguments = {
            "angular_z": message.angular_z,
            "duration_s": message.duration_s,
        }
    elif name == "wave":
        arguments = {"count": message.count}
    elif name == "set_led":
        arguments = {"color": message.color}
    elif name == "set_mode":
        arguments = {"mode": message.mode}
    elif name == "navigate_to":
        arguments = {"target": message.target}
    elif name == "follow_waypoints":
        arguments = {
            "waypoints": list(message.waypoints),
            "number_of_loops": message.number_of_loops,
        }
    payload = {
        "name": name,
        "arguments": arguments,
        "request_id": message.command_id,
        "source": message.source,
        "priority": message.priority,
    }
    return payload


def feedback_message_to_dict(message: RobotCommandFeedback) -> dict:
    return {
        "command_id": message.command_id,
        "phase": message.phase,
        "progress": message.progress,
        "detail": message.detail,
    }


def result_message_to_dict(message: RobotCommandResult) -> dict:
    return {
        "command_id": message.command_id,
        "success": message.success,
        "status": message.status,
        "message": message.message,
    }
