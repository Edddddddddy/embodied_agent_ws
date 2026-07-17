#!/usr/bin/env python3
"""Verify continuous voice sessions require a new wake word after timeout."""

import json
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import RecognitionFeedback, RobotCommand, RobotCommandResult, WakeEvent
from embodied_agent_core.ros_event_transport import (
    recognition_feedback_message_to_dict,
    wake_event_message_to_dict,
)
from embodied_agent_core.ros_qos import event_qos, state_qos
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from tools.acceptance.typed_action_probe_utils import candidate_dict, result_dict


class SessionTimeoutProbe(Node):
    def __init__(self):
        super().__init__("continuous_session_timeout_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.states = []
        self.session_states = []
        self.wake_events = []
        self.feedback = []
        self.candidates = []
        self.results = []
        self.velocities = []
        self.create_subscription(String, "/agent/state", self._on_state, 10)
        self.create_subscription(String, "/agent/session_state", self._on_session, state_qos())
        self.create_subscription(WakeEvent, "/agent/wake_event", self._on_wake, event_qos())
        self.create_subscription(
            RecognitionFeedback,
            "/agent/recognition_feedback",
            self._on_feedback,
            event_qos(),
        )
        self.create_subscription(RobotCommand, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(RobotCommandResult, "/robot/action_result", self._on_result, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)

    def _on_state(self, message):
        self.states.append(message.data)

    def _on_session(self, message):
        self.session_states.append(message.data)

    def _on_wake(self, message):
        self.wake_events.append(wake_event_message_to_dict(message))

    def _on_feedback(self, message):
        self.feedback.append(recognition_feedback_message_to_dict(message))

    def _on_candidate(self, message):
        self.candidates.append(candidate_dict(message))

    def _on_result(self, message):
        self.results.append(result_dict(message))

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
    node = SessionTimeoutProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.text_pub.get_subscription_count() > 0
                and node.count_publishers("/robot/action_result") > 0
                and node.count_subscribers("/robot/action_command_typed") > 0
            ),
            15.0,
            "continuous session timeout pipeline was not discovered",
        )
        time.sleep(0.5)

        node.text_pub.publish(String(data="小智"))
        wait_until(
            lambda: any(event.get("kind") == "wake" for event in node.wake_events),
            5.0,
            "wake-only utterance did not open the session",
        )

        time.sleep(1.2)
        candidate_count = len(node.candidates)
        node.text_pub.publish(String(data="向前走一秒"))
        time.sleep(0.8)
        if len(node.candidates) != candidate_count:
            raise RuntimeError("command was accepted after voice session timeout")
        if "retry_listening" not in node.states:
            raise RuntimeError(f"timeout rejection did not publish retry state: {node.states}")
        if not any(event.get("kind") == "rejected" for event in node.wake_events):
            raise RuntimeError(f"timeout rejection wake event missing: {node.wake_events}")
        if not any(item.get("status") == "retry" for item in node.feedback):
            raise RuntimeError(f"retry feedback missing after timeout: {node.feedback}")
        if not any(item.get("status") == "session_timeout" for item in node.feedback):
            raise RuntimeError(
                f"session timeout feedback missing after timeout: {node.feedback}"
            )

        node.text_pub.publish(String(data="小智向前走一秒"))
        wait_until(
            lambda: any(candidate.get("name") == "move" for candidate in node.candidates),
            8.0,
            "new wake word did not reopen the session",
        )
        wait_until(lambda: node.results, 15.0, "move command did not finish")
        wait_until(
            lambda: node.velocities
            and abs(node.velocities[-1][0]) < 1e-6
            and abs(node.velocities[-1][1]) < 1e-6,
            8.0,
            "cmd_vel did not settle to zero after reopened session command",
        )

        print(
            json.dumps(
                {
                    "wake_event_kinds": [event.get("kind") for event in node.wake_events],
                    "states_tail": node.states[-6:],
                    "candidate_names": [candidate.get("name") for candidate in node.candidates],
                    "result_count": len(node.results),
                    "final_cmd_vel": node.velocities[-1] if node.velocities else None,
                    "status": "PASS",
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
