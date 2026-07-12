#!/usr/bin/env python3
"""Reliably verify the mock online Agent action and metrics topics."""

import json
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import AgentTurnMetrics, RobotActionAck
from embodied_agent_core.metrics_transport import agent_turn_metrics_message_to_dict
from embodied_agent_core.runtime_status_transport import action_ack_to_dict
from rclpy.node import Node
from std_msgs.msg import String


class MockOnlineProbe(Node):
    def __init__(self):
        super().__init__("mock_online_probe")
        self.input_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.action = None
        self.metrics = None
        self.done = threading.Event()
        self.create_subscription(RobotActionAck, "/robot/action_ack", self._on_action, 10)
        self.create_subscription(
            AgentTurnMetrics, "/agent/metrics", self._on_metrics, 10
        )

    def _on_action(self, message):
        self.action = action_ack_to_dict(message)
        self._complete_if_ready()

    def _on_metrics(self, message):
        self.metrics = agent_turn_metrics_message_to_dict(message)
        self._complete_if_ready()

    def _complete_if_ready(self):
        if self.action is not None and self.metrics is not None:
            self.done.set()


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = MockOnlineProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.input_pub.get_subscription_count() > 0
            and node.count_publishers("/robot/action_ack") > 0
            and node.count_publishers("/agent/metrics") > 0,
            10.0,
            "mock Agent topics were not discovered",
        )
        time.sleep(0.5)
        node.input_pub.publish(String(data="小智，向前走一秒"))
        if not node.done.wait(10.0):
            raise TimeoutError(
                f"mock turn incomplete: action={node.action}, metrics={node.metrics}"
            )
        if node.action.get("action") != "move":
            raise RuntimeError(f"unexpected action: {node.action}")
        if "llm_first_token_ms" not in node.metrics:
            raise RuntimeError(f"incomplete metrics: {node.metrics}")
        print(json.dumps({
            "action": node.action,
            "metrics": node.metrics,
        }, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
