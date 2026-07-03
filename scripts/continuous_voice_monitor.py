#!/usr/bin/env python3
"""Human-facing monitor for long-running voice control demos."""

import argparse
import json
from typing import Any

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def _json_dict(serialized: str) -> dict[str, Any]:
    try:
        payload = json.loads(serialized)
    except json.JSONDecodeError:
        return {}
    return payload if isinstance(payload, dict) else {}


def format_session_state(state: str) -> str:
    return f"[session] {state}"


def format_wake_event(serialized: str) -> str:
    payload = _json_dict(serialized)
    provider = payload.get("provider", "unknown")
    kind = payload.get("kind", "unknown")
    return f"[wake] {provider}:{kind}"


def format_asr_final(text: str) -> str:
    return f"[asr] {text}"


def format_queue_state(state: str, size: int | None = None) -> str:
    if state == "queued":
        suffix = "" if size is None else f" size={size}"
        return f"[queue] enqueue{suffix}"
    if state == "queue_full":
        return "[queue] full"
    return f"[state] {state}"


def format_queue_event(serialized: str) -> str:
    payload = _json_dict(serialized)
    event = payload.get("event", "unknown")
    text = payload.get("text") or ""
    size = payload.get("size")
    dropped = payload.get("dropped", 0)
    reason = payload.get("reason") or ""
    if event == "enqueue":
        return f"[queue] enqueue {text} size={size}"
    if event == "rejected":
        return f"[queue] rejected {text} reason={reason}"
    if event == "clear":
        return f"[queue] clear dropped={dropped}"
    return f"[queue] {event}"


def format_execution_event(serialized: str) -> str:
    payload = _json_dict(serialized)
    event = payload.get("event", "unknown")
    text = payload.get("text") or ""
    if event == "finished":
        reason = payload.get("reason") or ""
        return f"[exec] finished {text} {reason}".rstrip()
    return f"[exec] {event} {text}".rstrip()


def format_action_candidate(serialized: str) -> str:
    payload = _json_dict(serialized)
    name = payload.get("name", "unknown")
    return f"[action] executing {name}"


def format_action_result(serialized: str) -> str:
    payload = _json_dict(serialized)
    if payload.get("success") is True:
        return f"[result] {payload.get('message', 'succeeded')}"
    if "status" in payload:
        return f"[ack] {payload.get('action', 'unknown')} {payload.get('status')}"
    return f"[result] {payload.get('message', 'unknown')}"


def format_recognition_feedback(serialized: str) -> str:
    payload = _json_dict(serialized)
    if payload.get("status") == "normalized":
        original = payload.get("original", "")
        normalized = payload.get("normalized", "")
        return f"[normalize] {original} -> {normalized}"
    if payload.get("status") == "retry":
        attempt = payload.get("attempt", "?")
        prompt = payload.get("prompt", "请再说一次")
        return f"[retry] attempt={attempt} {prompt}"
    return "[feedback] " + (payload.get("reason") or "unknown")


class ContinuousVoiceMonitor(Node):
    def __init__(self):
        super().__init__("continuous_voice_monitor")
        self._queued_seen = 0
        self._structured_queue_seen = False
        self.create_subscription(String, "/agent/session_state", self._on_session, 10)
        self.create_subscription(String, "/agent/wake_event", self._on_wake, 10)
        self.create_subscription(String, "/agent/asr_final", self._on_asr, 10)
        self.create_subscription(String, "/agent/state", self._on_state, 10)
        self.create_subscription(String, "/agent/command_queue", self._on_queue, 10)
        self.create_subscription(
            String, "/agent/command_execution", self._on_execution, 10
        )
        self.create_subscription(String, "/agent/action_candidate", self._on_action, 10)
        self.create_subscription(
            String, "/agent/recognition_feedback", self._on_feedback, 10
        )
        self.create_subscription(String, "/robot/action_result", self._on_result, 10)
        self.create_subscription(String, "/robot/action_ack", self._on_result, 10)

    def _emit(self, line: str) -> None:
        print(line, flush=True)

    def _on_session(self, message: String) -> None:
        self._emit(format_session_state(message.data))

    def _on_wake(self, message: String) -> None:
        self._emit(format_wake_event(message.data))

    def _on_asr(self, message: String) -> None:
        self._emit(format_asr_final(message.data))

    def _on_state(self, message: String) -> None:
        if message.data == "queued" and not self._structured_queue_seen:
            self._queued_seen += 1
            self._emit(format_queue_state(message.data, self._queued_seen))
        elif message.data == "queue_full":
            self._emit(format_queue_state(message.data))

    def _on_queue(self, message: String) -> None:
        self._structured_queue_seen = True
        self._emit(format_queue_event(message.data))

    def _on_execution(self, message: String) -> None:
        self._emit(format_execution_event(message.data))

    def _on_action(self, message: String) -> None:
        self._emit(format_action_candidate(message.data))

    def _on_feedback(self, message: String) -> None:
        self._emit(format_recognition_feedback(message.data))

    def _on_result(self, message: String) -> None:
        self._emit(format_action_result(message.data))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_args()
    rclpy.init()
    node = ContinuousVoiceMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
