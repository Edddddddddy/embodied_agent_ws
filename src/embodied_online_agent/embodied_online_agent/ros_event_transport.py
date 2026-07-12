"""连续语音领域事件与 ROS 2 强类型消息之间的唯一转换 seam。"""

from __future__ import annotations

from builtin_interfaces.msg import Time
from embodied_agent_interfaces.msg import (
    CommandContext as CommandContextMessage,
    CommandExecutionEvent as CommandExecutionEventMessage,
    CommandQueueEvent as CommandQueueEventMessage,
)

from .continuous_voice import CommandExecutionEvent, CommandQueueEvent


_QUEUE_EVENT_TO_WIRE = {
    "enqueue": CommandQueueEventMessage.EVENT_ENQUEUED,
    "rejected": CommandQueueEventMessage.EVENT_REJECTED,
    "clear": CommandQueueEventMessage.EVENT_CLEARED,
    "expired": CommandQueueEventMessage.EVENT_EXPIRED,
}
_QUEUE_EVENT_FROM_WIRE = {value: key for key, value in _QUEUE_EVENT_TO_WIRE.items()}
_EXECUTION_EVENT_TO_WIRE = {
    "started": CommandExecutionEventMessage.EVENT_STARTED,
    "finished": CommandExecutionEventMessage.EVENT_FINISHED,
}
_EXECUTION_EVENT_FROM_WIRE = {
    value: key for key, value in _EXECUTION_EVENT_TO_WIRE.items()
}


def _context_to_message(metadata: dict) -> CommandContextMessage:
    message = CommandContextMessage()
    message.batch_id = str(metadata.get("batch_id") or "")
    message.batch_index = max(0, int(metadata.get("batch_index") or 0))
    message.batch_size = max(0, int(metadata.get("batch_size") or 0))
    message.source_text = str(metadata.get("source_text") or "")
    message.nlu_intent = str(metadata.get("nlu_intent") or "")
    message.nlu_confidence = float(metadata.get("nlu_confidence") or 0.0)
    return message


def _context_to_dict(message: CommandContextMessage) -> dict:
    values = {
        "batch_id": message.batch_id,
        "batch_index": int(message.batch_index),
        "batch_size": int(message.batch_size),
        "source_text": message.source_text,
        "nlu_intent": message.nlu_intent,
        "nlu_confidence": float(message.nlu_confidence),
    }
    # 监控输出保持精简：未设置的上下文字段不制造无意义的零值噪声。
    return {key: value for key, value in values.items() if value not in ("", 0, 0.0)}


def queue_event_to_message(
    event: CommandQueueEvent, *, stamp: Time | None = None
) -> CommandQueueEventMessage:
    if event.event not in _QUEUE_EVENT_TO_WIRE:
        raise ValueError(f"unsupported queue event: {event.event}")
    message = CommandQueueEventMessage()
    if stamp is not None:
        message.stamp = stamp
    message.event = _QUEUE_EVENT_TO_WIRE[event.event]
    message.source = event.source
    message.text = event.text
    message.size = max(0, int(event.size))
    message.dropped = max(0, int(event.dropped))
    message.reason = event.reason
    message.priority_stop = bool(event.priority_stop)
    message.context = _context_to_message(event.metadata)
    return message


def execution_event_to_message(
    event: CommandExecutionEvent, *, stamp: Time | None = None
) -> CommandExecutionEventMessage:
    if event.event not in _EXECUTION_EVENT_TO_WIRE:
        raise ValueError(f"unsupported execution event: {event.event}")
    message = CommandExecutionEventMessage()
    if stamp is not None:
        message.stamp = stamp
    message.event = _EXECUTION_EVENT_TO_WIRE[event.event]
    message.source = event.source
    message.text = event.text
    message.success_known = event.success is not None
    message.success = bool(event.success) if event.success is not None else False
    message.reason = event.reason
    message.context = _context_to_message(event.metadata)
    return message


def queue_event_message_to_dict(message: CommandQueueEventMessage) -> dict:
    payload = {
        "event": _QUEUE_EVENT_FROM_WIRE.get(message.event, "unknown"),
        "source": message.source,
        "text": message.text,
        "size": int(message.size),
        "dropped": int(message.dropped),
        "reason": message.reason,
        "priority_stop": bool(message.priority_stop),
    }
    payload.update(_context_to_dict(message.context))
    return payload


def execution_event_message_to_dict(message: CommandExecutionEventMessage) -> dict:
    payload = {
        "event": _EXECUTION_EVENT_FROM_WIRE.get(message.event, "unknown"),
        "source": message.source,
        "text": message.text,
        "success": bool(message.success) if message.success_known else None,
        "reason": message.reason,
    }
    payload.update(_context_to_dict(message.context))
    return payload
