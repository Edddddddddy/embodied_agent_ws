"""可执行 ROS 验收探针共享的 typed-action 消息 Adapter。

Probe 只描述要验证的场景，不应各自手写 ROS 消息字段或复制传输转换规则；
这些小函数统一复用生产链路的序列化实现，让验收输入与真实 Agent 保持同一语义。
"""

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
    """用生产转换器构造候选动作，避免 probe 形成第二套消息协议。"""

    return action_command_to_message(
        ActionCommand(name, arguments, request_id, priority=priority), source=source
    )
