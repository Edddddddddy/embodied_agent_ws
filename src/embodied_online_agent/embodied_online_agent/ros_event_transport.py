"""连续语音领域事件与 ROS 2 强类型消息之间的唯一转换 seam。"""

from __future__ import annotations

from builtin_interfaces.msg import Time
from embodied_agent_interfaces.msg import (
    CommandContext as CommandContextMessage,
    CommandExecutionEvent as CommandExecutionEventMessage,
    CommandQueueEvent as CommandQueueEventMessage,
    CommandSlot as CommandSlotMessage,
    NluCommand as NluCommandMessage,
    NluParseEvent as NluParseEventMessage,
    RecognitionFeedback as RecognitionFeedbackMessage,
    TextRewrite as TextRewriteMessage,
    WakeEvent as WakeEventMessage,
)

from .continuous_voice import CommandExecutionEvent, CommandQueueEvent
from .ros_action_transport import action_command_to_message, command_message_to_dict
from .wake_provider import WakeEvent, WakeEventKind


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
_WAKE_EVENT_TO_WIRE = {
    WakeEventKind.WAKE: WakeEventMessage.KIND_WAKE,
    WakeEventKind.CONTINUE: WakeEventMessage.KIND_CONTINUE,
    WakeEventKind.REJECTED: WakeEventMessage.KIND_REJECTED,
    WakeEventKind.SLEEP: WakeEventMessage.KIND_SLEEP,
}
_WAKE_EVENT_FROM_WIRE = {value: key.value for key, value in _WAKE_EVENT_TO_WIRE.items()}
_RECOGNITION_STATUS_TO_WIRE = {
    "retry": RecognitionFeedbackMessage.STATUS_RETRY,
    "ignored": RecognitionFeedbackMessage.STATUS_IGNORED,
    "queue_rejected": RecognitionFeedbackMessage.STATUS_QUEUE_REJECTED,
    "asr_endpoint": RecognitionFeedbackMessage.STATUS_ASR_ENDPOINT,
    "asr_commit": RecognitionFeedbackMessage.STATUS_ASR_COMMIT,
    "session_timeout": RecognitionFeedbackMessage.STATUS_SESSION_TIMEOUT,
    "normalized": RecognitionFeedbackMessage.STATUS_NORMALIZED,
    "completed": RecognitionFeedbackMessage.STATUS_COMPLETED,
    "asr_final_recovered": RecognitionFeedbackMessage.STATUS_ASR_FINAL_RECOVERED,
}
_RECOGNITION_STATUS_FROM_WIRE = {
    value: key for key, value in _RECOGNITION_STATUS_TO_WIRE.items()
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


def wake_event_to_message(
    event: WakeEvent, *, stamp: Time | None = None
) -> WakeEventMessage:
    if event.kind not in _WAKE_EVENT_TO_WIRE:
        raise ValueError(f"unsupported wake event: {event.kind}")
    message = WakeEventMessage()
    if stamp is not None:
        message.stamp = stamp
    message.kind = _WAKE_EVENT_TO_WIRE[event.kind]
    message.provider = event.provider
    message.transcript = event.transcript
    message.command_known = event.command is not None
    message.command = event.command or ""
    return message


def recognition_feedback_to_message(
    payload: dict, *, stamp: Time | None = None
) -> RecognitionFeedbackMessage:
    status = str(payload.get("status") or "")
    if status not in _RECOGNITION_STATUS_TO_WIRE:
        raise ValueError(f"unsupported recognition feedback status: {status}")
    message = RecognitionFeedbackMessage()
    if stamp is not None:
        message.stamp = stamp
    message.status = _RECOGNITION_STATUS_TO_WIRE[status]
    message.reason = str(payload.get("reason") or "")
    message.transcript = str(payload.get("transcript") or "")
    message.prompt = str(payload.get("prompt") or "")
    message.source = str(payload.get("source") or "")
    message.original = str(
        payload.get("original") or payload.get("original_final") or ""
    )
    message.rewritten = str(
        payload.get("normalized")
        or payload.get("completed")
        or payload.get("recovered")
        or ""
    )
    message.partial = str(payload.get("partial") or "")
    message.confidence = float(payload.get("confidence") or 0.0)
    message.attempt = max(0, int(payload.get("attempt") or 0))
    message.max_attempts = max(0, int(payload.get("max_attempts") or 0))
    message.queue_size = max(0, int(payload.get("queue_size") or 0))
    message.delay_ms = max(0, int(payload.get("delay_ms") or 0))
    for item in payload.get("matches") or []:
        if not isinstance(item, dict):
            continue
        rewrite = TextRewriteMessage()
        rewrite.source = str(item.get("source") or "")
        rewrite.target = str(item.get("target") or "")
        rewrite.score = float(item.get("score") or 0.0)
        rewrite.provider = str(item.get("provider") or "")
        message.rewrites.append(rewrite)
    return message


def _slot_to_message(name: str, value) -> CommandSlotMessage:
    message = CommandSlotMessage()
    message.name = str(name)
    if value is None:
        message.value_type = CommandSlotMessage.TYPE_UNKNOWN
    elif isinstance(value, bool):
        message.value_type = CommandSlotMessage.TYPE_BOOLEAN
        message.boolean_value = value
    elif isinstance(value, (int, float)):
        message.value_type = CommandSlotMessage.TYPE_NUMBER
        message.number_value = float(value)
    elif isinstance(value, (list, tuple)) and all(
        isinstance(item, str) for item in value
    ):
        message.value_type = CommandSlotMessage.TYPE_TEXT_LIST
        message.text_list_value = list(value)
    else:
        message.value_type = CommandSlotMessage.TYPE_TEXT
        message.text_value = str(value)
    return message


def nlu_parse_to_message(
    transcript: str,
    nlu_result,
    batch_id: str,
    *,
    source: str,
    stamp: Time | None = None,
) -> NluParseEventMessage:
    message = NluParseEventMessage()
    if stamp is not None:
        message.stamp = stamp
    message.source = source
    message.transcript = transcript
    message.batch_id = batch_id
    for parsed in nlu_result.commands:
        command = NluCommandMessage()
        command.intent = parsed.intent
        command.span_text = parsed.span_text
        command.confidence = float(parsed.confidence)
        command.slots = [
            _slot_to_message(name, value) for name, value in parsed.slots.items()
        ]
        command.actions = [
            action_command_to_message(action, source=source)
            for action in parsed.actions
        ]
        message.commands.append(command)
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


def wake_event_message_to_dict(message: WakeEventMessage) -> dict:
    return {
        "kind": _WAKE_EVENT_FROM_WIRE.get(message.kind, "unknown"),
        "provider": message.provider,
        "transcript": message.transcript,
        "command": message.command if message.command_known else None,
    }


def recognition_feedback_message_to_dict(message: RecognitionFeedbackMessage) -> dict:
    status = _RECOGNITION_STATUS_FROM_WIRE.get(message.status, "unknown")
    payload = {"status": status, "reason": message.reason}
    for key, value in (
        ("transcript", message.transcript),
        ("prompt", message.prompt),
        ("source", message.source),
    ):
        if value:
            payload[key] = value
    if status == "normalized":
        payload.update(
            {
                "original": message.original,
                "normalized": message.rewritten,
                "confidence": float(message.confidence),
                "matches": [
                    {
                        "source": item.source,
                        "target": item.target,
                        "score": float(item.score),
                        "provider": item.provider,
                    }
                    for item in message.rewrites
                ],
            }
        )
    elif status == "completed":
        payload.update(
            {
                "original": message.original,
                "completed": message.rewritten,
                "confidence": float(message.confidence),
            }
        )
    elif status == "asr_final_recovered":
        payload.update(
            {
                "original_final": message.original,
                "recovered": message.rewritten,
                "partial": message.partial,
            }
        )
    if message.attempt or message.max_attempts:
        payload["attempt"] = int(message.attempt)
        payload["max_attempts"] = int(message.max_attempts)
    if status == "queue_rejected":
        payload["queue_size"] = int(message.queue_size)
    if status == "asr_endpoint":
        payload["delay_ms"] = int(message.delay_ms)
    return payload


def _slot_message_value(message: CommandSlotMessage):
    if message.value_type == CommandSlotMessage.TYPE_BOOLEAN:
        return bool(message.boolean_value)
    if message.value_type == CommandSlotMessage.TYPE_NUMBER:
        return float(message.number_value)
    if message.value_type == CommandSlotMessage.TYPE_TEXT:
        return message.text_value
    if message.value_type == CommandSlotMessage.TYPE_TEXT_LIST:
        return list(message.text_list_value)
    return None


def nlu_parse_message_to_dict(message: NluParseEventMessage) -> dict:
    return {
        "status": "nlu_parsed",
        "reason": "command_nlu",
        "transcript": message.transcript,
        "batch_id": message.batch_id,
        "source": message.source,
        "commands": [
            {
                "intent": command.intent,
                "span_text": command.span_text,
                "confidence": round(float(command.confidence), 3),
                "slots": {
                    slot.name: _slot_message_value(slot) for slot in command.slots
                },
                "actions": [command_message_to_dict(action) for action in command.actions],
            }
            for command in message.commands
        ],
    }
