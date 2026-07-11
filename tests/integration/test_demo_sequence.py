#!/usr/bin/env python3
"""Verify the rich demo phrase becomes an ordered Action/BT execution sequence."""

import json
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import RobotCommand, RobotCommandResult
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from typed_action_test_utils import candidate_dict, result_dict


class DemoSequenceProbe(Node):
    def __init__(self):
        super().__init__("demo_sequence_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.asr_final = None
        self.candidates = []
        self.results = []
        self.velocities = []
        self.create_subscription(String, "/agent/asr_final", self._on_asr_final, 10)
        self.create_subscription(
            RobotCommand, "/agent/action_candidate", self._on_candidate, 10
        )
        self.create_subscription(RobotCommandResult, "/robot/action_result", self._on_result, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)

    def _on_asr_final(self, message):
        self.asr_final = message.data

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
    node = DemoSequenceProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.text_pub.get_subscription_count() > 0
            and node.count_publishers("/robot/action_result") > 0,
            15.0,
            "demo sequence pipeline was not discovered",
        )
        time.sleep(0.5)
        node.text_pub.publish(String(data="演示一下"))
        wait_until(
            lambda: len(node.results) >= 6
            and len(node.candidates) >= 6
            and any(abs(x) > 0.01 and abs(z) > 0.01 for x, z in node.velocities),
            25.0,
            "demo sequence did not finish or no arc velocity was observed",
        )
        names = [candidate["name"] for candidate in node.candidates[:6]]
        expected = ["set_led", "wave", "move", "turn", "arc", "stop"]
        if names != expected:
            raise RuntimeError(f"unexpected demo sequence: {names}")
        failed = [result for result in node.results if result.get("success") is not True]
        if failed:
            raise RuntimeError(f"demo action failed: {failed}")
        print(json.dumps({
            "asr_text": node.asr_final,
            "candidate_sequence": names,
            "result_count": len(node.results),
            "arc_velocity_observed": True,
            "status": "PASS",
        }, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
