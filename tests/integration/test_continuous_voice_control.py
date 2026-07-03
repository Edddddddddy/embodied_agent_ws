#!/usr/bin/env python3
"""Verify one wake word opens a multi-command voice-control session."""

import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class ContinuousVoiceProbe(Node):
    def __init__(self):
        super().__init__("continuous_voice_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.states = []
        self.candidates = []
        self.results = []
        self.create_subscription(String, "/agent/state", self._on_state, 10)
        self.create_subscription(
            String, "/agent/action_candidate", self._on_candidate, 10
        )
        self.create_subscription(String, "/robot/action_result", self._on_result, 10)

    def _on_state(self, message):
        self.states.append(message.data)

    def _on_candidate(self, message):
        self.candidates.append(json.loads(message.data))

    def _on_result(self, message):
        self.results.append(json.loads(message.data))


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
        wait_until(lambda: "sleeping" in node.states, 5.0, "session did not sleep")
        before = len(node.candidates)
        node.text_pub.publish(String(data="向前走一秒"))
        time.sleep(1.0)
        if len(node.candidates) != before:
            raise RuntimeError("command without wake word was accepted after sleep")

        print(json.dumps({
            "candidate_sequence": names,
            "result_count": len(node.results),
            "states_tail": node.states[-6:],
            "status": "PASS",
        }, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
