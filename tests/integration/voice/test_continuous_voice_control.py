#!/usr/bin/env python3
"""Verify one wake word opens a multi-command voice-control session."""

import json
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import (
    CommandExecutionEvent,
    CommandQueueEvent,
    RecognitionFeedback,
    RobotCommand,
    RobotCommandFeedback,
    RobotCommandResult,
    WakeEvent,
)
from embodied_agent_core.ros_event_transport import (
    execution_event_message_to_dict,
    queue_event_message_to_dict,
    recognition_feedback_message_to_dict,
    wake_event_message_to_dict,
)
from embodied_agent_core.ros_qos import event_qos, state_qos
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from tests.integration.typed_action_test_utils import candidate_dict, result_dict


class ContinuousVoiceProbe(Node):
    def __init__(self):
        super().__init__("continuous_voice_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.wake_input_pub = self.create_publisher(WakeEvent, "/agent/wake_event_input", 10)
        self.states = []
        self.session_states = []
        self.wake_events = []
        self.queue_events = []
        self.execution_events = []
        self.recognition_feedback = []
        self.candidates = []
        self.results = []
        self.feedback = []
        self.velocities = []
        self.create_subscription(String, "/agent/state", self._on_state, state_qos())
        self.create_subscription(String, "/agent/session_state", self._on_session_state, state_qos())
        self.create_subscription(WakeEvent, "/agent/wake_event", self._on_wake_event, event_qos())
        self.create_subscription(CommandQueueEvent, "/agent/command_queue", self._on_queue_event, event_qos())
        self.create_subscription(
            CommandExecutionEvent, "/agent/command_execution", self._on_execution_event, event_qos()
        )
        self.create_subscription(
            RecognitionFeedback,
            "/agent/recognition_feedback",
            self._on_feedback,
            event_qos(),
        )
        self.create_subscription(
            RobotCommand, "/agent/action_candidate", self._on_candidate, 10
        )
        self.create_subscription(RobotCommandResult, "/robot/action_result", self._on_result, 10)
        self.create_subscription(
            RobotCommandFeedback, "/robot/action_feedback", self._on_feedback_event, 10
        )
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)

    def _on_state(self, message):
        self.states.append(message.data)

    def _on_session_state(self, message):
        self.session_states.append(message.data)

    def _on_wake_event(self, message):
        self.wake_events.append(wake_event_message_to_dict(message))

    def _on_queue_event(self, message):
        self.queue_events.append(queue_event_message_to_dict(message))

    def _on_execution_event(self, message):
        self.execution_events.append(execution_event_message_to_dict(message))

    def _on_feedback(self, message):
        self.recognition_feedback.append(recognition_feedback_message_to_dict(message))

    def _on_candidate(self, message):
        self.candidates.append(candidate_dict(message))

    def _on_result(self, message):
        self.results.append(result_dict(message))

    def _on_feedback_event(self, message):
        self.feedback.append(
            {
                "command_id": message.command_id,
                "phase": message.phase,
                "progress": message.progress,
                "detail": message.detail,
            }
        )

    def _on_velocity(self, message):
        self.velocities.append((message.linear.x, message.angular.z))


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = ContinuousVoiceProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.text_pub.get_subscription_count() > 0
                and node.wake_input_pub.get_subscription_count() > 0
                and node.count_publishers("/robot/action_result") > 0
                and node.count_subscribers("/robot/action_command_typed") > 0
            ),
            15.0,
            "continuous voice pipeline was not discovered",
        )
        time.sleep(0.5)
        # 这里故意使用常见 ASR 错词，验证命令归一化接在 Agent 主链路里，
        # 而不是只在单元测试里“看起来可用”。
        for phrase in ["小智", "嗯。", "钱进一秒", "钱进一秒。", "作转九十度", "让圈"]:
            node.text_pub.publish(String(data=phrase))
            time.sleep(0.2)

        wait_until(
            lambda: len(node.candidates) >= 3 and len(node.results) >= 3,
            25.0,
            "continuous queued commands did not finish",
        )
        names = [candidate["name"] for candidate in node.candidates[:3]]
        expected = ["move", "turn", "arc"]
        if names != expected:
            raise RuntimeError(f"unexpected continuous command order: {names}")

        sleep_candidate_start = len(node.candidates)
        node.text_pub.publish(String(data="退出控制"))
        wait_until(
            lambda: "sleeping" in node.states and "sleeping" in node.session_states,
            5.0,
            "session did not sleep",
        )
        wait_until(
            lambda: any(
                candidate.get("name") == "stop"
                for candidate in node.candidates[sleep_candidate_start:]
            ),
            5.0,
            "session sleep did not publish a safety stop",
        )
        before = len(node.candidates)
        node.text_pub.publish(String(data="向前走一秒"))
        time.sleep(1.0)
        if len(node.candidates) != before:
            raise RuntimeError("command without wake word was accepted after sleep")

        wake = WakeEvent()
        wake.kind = WakeEvent.KIND_WAKE
        wake.provider = "test_kws"
        node.wake_input_pub.publish(wake)
        time.sleep(0.1)
        stop_candidate_start = len(node.candidates)
        node.text_pub.publish(String(data="走正方形"))
        time.sleep(0.15)
        node.text_pub.publish(String(data="亭下"))
        wait_until(
            lambda: any(
                candidate.get("name") == "stop"
                for candidate in node.candidates[stop_candidate_start:]
            ),
            5.0,
            "priority stop candidate was not published",
        )
        wait_until(
            lambda: any(
                event.get("event") == "clear" and event.get("priority_stop") is True
                for event in node.queue_events
            ),
            5.0,
            "priority stop did not clear the command queue",
        )
        wait_until(
            lambda: node.velocities
            and abs(node.velocities[-1][0]) < 1e-6
            and abs(node.velocities[-1][1]) < 1e-6,
            8.0,
            "cmd_vel did not return to zero after priority stop",
        )
        wake_kinds = [event.get("kind") for event in node.wake_events]
        if "wake" not in wake_kinds or "sleep" not in wake_kinds:
            raise RuntimeError(f"wake/session events were not published: {node.wake_events}")
        if "test_kws" not in [event.get("provider") for event in node.wake_events]:
            raise RuntimeError(f"external KWS wake event was not bridged: {node.wake_events}")

        wake = WakeEvent()
        wake.kind = WakeEvent.KIND_WAKE
        wake.provider = "test_kws_sleep"
        node.wake_input_pub.publish(wake)
        time.sleep(0.1)
        external_sleep_start = len(node.candidates)
        node.text_pub.publish(String(data="走正方形"))
        time.sleep(0.15)
        sleep = WakeEvent()
        sleep.kind = WakeEvent.KIND_SLEEP
        sleep.provider = "test_kws_sleep"
        node.wake_input_pub.publish(sleep)
        wait_until(
            lambda: any(
                event.get("kind") == "sleep"
                and event.get("provider") == "test_kws_sleep"
                for event in node.wake_events
            ),
            5.0,
            "external KWS sleep event was not bridged",
        )
        wait_until(
            lambda: "sleeping" in node.session_states,
            5.0,
            "external KWS sleep did not close the session",
        )
        wait_until(
            lambda: any(
                candidate.get("name") == "stop"
                for candidate in node.candidates[external_sleep_start:]
            ),
            5.0,
            "external KWS sleep did not publish a safety stop",
        )
        queue_sizes = [
            event.get("size")
            for event in node.queue_events
            if event.get("event") == "enqueue"
        ]
        execution_kinds = [event.get("event") for event in node.execution_events]
        if not queue_sizes or max(queue_sizes) < 1:
            raise RuntimeError(f"queue events were not published: {node.queue_events}")
        if "started" not in execution_kinds or "finished" not in execution_kinds:
            raise RuntimeError(
                f"execution events were not published: {node.execution_events}"
            )
        if not node.feedback:
            raise RuntimeError("typed Action feedback was not published")
        normalized = [
            event for event in node.recognition_feedback
            if event.get("reason") == "command_normalized"
        ]
        if not normalized:
            raise RuntimeError(
                f"normalization feedback was not published: {node.recognition_feedback}"
            )
        ignored_reasons = {
            event.get("reason")
            for event in node.recognition_feedback
            if event.get("status") == "ignored"
        }
        if not {"filler", "duplicate_command"}.issubset(ignored_reasons):
            raise RuntimeError(
                f"ignored recognition feedback was not published: {node.recognition_feedback}"
            )

        print(json.dumps({
            "candidate_sequence": names,
            "result_count": len(node.results),
            "states_tail": node.states[-6:],
            "session_states_tail": node.session_states[-6:],
            "wake_event_kinds": wake_kinds,
            "wake_event_providers": [event.get("provider") for event in node.wake_events],
            "queue_sizes": queue_sizes,
            "execution_event_kinds": execution_kinds,
            "feedback_count": len(node.feedback),
            "normalization_count": len(normalized),
            "ignored_reasons": sorted(ignored_reasons),
            "final_cmd_vel": node.velocities[-1] if node.velocities else None,
            "status": "PASS",
        }, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
