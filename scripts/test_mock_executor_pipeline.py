#!/usr/bin/env python3
"""Verify pluginlib mock executor through the unchanged ROS Action/BT pipeline."""

import json
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class MockExecutorProbe(Node):
    def __init__(self):
        super().__init__("mock_executor_probe")
        self.candidate_pub = self.create_publisher(
            String, "/agent/action_candidate", 10
        )
        self.velocities = []
        self.ack = None
        self.result = None
        self.bt_status = None
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)
        self.create_subscription(String, "/robot/action_ack", self._on_ack, 10)
        self.create_subscription(
            String, "/robot/action_result", self._on_result, 10
        )
        self.create_subscription(String, "/robot/bt_status", self._on_bt, 10)

    def _on_velocity(self, message):
        self.velocities.append((message.linear.x, message.angular.z))

    def _on_ack(self, message):
        payload = json.loads(message.data)
        if payload.get("action") == "move":
            self.ack = payload

    def _on_result(self, message):
        payload = json.loads(message.data)
        if payload.get("message") == "succeeded":
            self.result = payload

    def _on_bt(self, message):
        payload = json.loads(message.data)
        if payload.get("outcome") == "succeeded":
            self.bt_status = payload


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = MockExecutorProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.candidate_pub.get_subscription_count() > 0
            and node.count_publishers("/robot/action_ack") > 0,
            10.0,
            "mock executor pipeline was not discovered",
        )
        time.sleep(0.5)
        command = {
            "name": "move",
            "arguments": {"linear_x": 0.15, "duration_s": 0.3},
        }
        node.candidate_pub.publish(String(data=json.dumps(command)))
        wait_until(
            lambda: node.result is not None
            and node.bt_status is not None
            and node.ack is not None
            and any(abs(linear) > 0.01 for linear, _ in node.velocities),
            5.0,
            "mock plugin did not complete the Action/BT pipeline",
        )
        wait_until(
            lambda: node.velocities
            and abs(node.velocities[-1][0]) < 1e-6
            and abs(node.velocities[-1][1]) < 1e-6,
            2.0,
            "mock plugin did not stop after success",
        )
        if node.ack.get("backend") != "mock":
            raise RuntimeError(f"unexpected executor backend: {node.ack}")
        print(json.dumps({
            "backend": node.ack.get("backend"),
            "action_result": node.result,
            "bt_result": node.bt_status,
            "observed_motion": True,
            "stopped": True,
        }, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
