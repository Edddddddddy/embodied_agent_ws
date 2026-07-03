#!/usr/bin/env python3
"""Verify one wake word opens a multi-command voice-control session."""

import json
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class ContinuousVoiceProbe(Node):
    def __init__(self):
        super().__init__("continuous_voice_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.states = []
        self.session_states = []
        self.wake_events = []
        self.queue_events = []
        self.execution_events = []
        self.candidates = []
        self.results = []
        self.velocities = []
        self.create_subscription(String, "/agent/state", self._on_state, 10)
        self.create_subscription(String, "/agent/session_state", self._on_session_state, 10)
        self.create_subscription(String, "/agent/wake_event", self._on_wake_event, 10)
        self.create_subscription(String, "/agent/command_queue", self._on_queue_event, 10)
        self.create_subscription(
            String, "/agent/command_execution", self._on_execution_event, 10
        )
        self.create_subscription(
            String, "/agent/action_candidate", self._on_candidate, 10
        )
        self.create_subscription(String, "/robot/action_result", self._on_result, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)

    def _on_state(self, message):
        self.states.append(message.data)

    def _on_session_state(self, message):
        self.session_states.append(message.data)

    def _on_wake_event(self, message):
        self.wake_events.append(json.loads(message.data))

    def _on_queue_event(self, message):
        self.queue_events.append(json.loads(message.data))

    def _on_execution_event(self, message):
        self.execution_events.append(json.loads(message.data))

    def _on_candidate(self, message):
        self.candidates.append(json.loads(message.data))

    def _on_result(self, message):
        self.results.append(json.loads(message.data))

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
            lambda: node.text_pub.get_subscription_count() > 0
            and node.count_publishers("/robot/action_result") > 0,
            15.0,
            "continuous voice pipeline was not discovered",
        )
        time.sleep(0.5)
        for phrase in ["小智", "向前走一秒", "左转九十度", "绕圈"]:
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

        node.text_pub.publish(String(data="退出控制"))
        wait_until(
            lambda: "sleeping" in node.states and "sleeping" in node.session_states,
            5.0,
            "session did not sleep",
        )
        before = len(node.candidates)
        node.text_pub.publish(String(data="向前走一秒"))
        time.sleep(1.0)
        if len(node.candidates) != before:
            raise RuntimeError("command without wake word was accepted after sleep")

        node.text_pub.publish(String(data="小智"))
        time.sleep(0.1)
        stop_candidate_start = len(node.candidates)
        node.text_pub.publish(String(data="走正方形"))
        time.sleep(0.15)
        node.text_pub.publish(String(data="急停"))
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

        print(json.dumps({
            "candidate_sequence": names,
            "result_count": len(node.results),
            "states_tail": node.states[-6:],
            "session_states_tail": node.session_states[-6:],
            "wake_event_kinds": wake_kinds,
            "queue_sizes": queue_sizes,
            "execution_event_kinds": execution_kinds,
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
