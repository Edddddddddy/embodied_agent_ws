#!/usr/bin/env python3
"""Verify that a trusted voice-style action physically moves TurtleBot3 in Gazebo."""

import json
import math
import os
import threading
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class GazeboProbe(Node):
    def __init__(self):
        super().__init__("gazebo_motion_probe")
        self.action_pub = self.create_publisher(String, "/agent/action_candidate", 10)
        self.position = None
        self.scan_received = False
        self.ack = None
        self.action_result = None
        self.move_result = None
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(String, "/robot/action_ack", self._on_ack, 10)
        self.create_subscription(
            String, "/robot/action_result", self._on_result, 10
        )

    def _on_odom(self, message):
        self.position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )

    def _on_scan(self, _message):
        self.scan_received = True

    def _on_ack(self, message):
        self.ack = json.loads(message.data)

    def _on_result(self, message):
        self.action_result = json.loads(message.data)
        if self.action_result.get("message") == "succeeded":
            self.move_result = self.action_result

    def action(self, name, arguments):
        data = json.dumps({"name": name, "arguments": arguments})
        self.action_pub.publish(String(data=data))


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.1)
    raise TimeoutError(description)


def main():
    require_typed_result = os.getenv("REQUIRE_TYPED_ACTION_RESULT") == "true"
    rclpy.init()
    node = GazeboProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.scan_received and node.position is not None
            and node.action_pub.get_subscription_count() > 0
            and node.count_publishers("/robot/action_ack") > 0,
            30.0,
            "Gazebo scan/odom/action pipeline was not ready",
        )
        time.sleep(1.0)
        start = node.position
        node.action("move", {"linear_x": 0.18, "duration_s": 2.0})
        wait_until(
            lambda: (
                node.position is not None
                and math.hypot(
                    node.position[0] - start[0], node.position[1] - start[1]
                ) > 0.08
                and node.ack is not None
                and node.ack.get("action") == "move"
                and node.ack.get("backend") == "simulation"
                and (
                not require_typed_result
                or (
                    node.move_result is not None
                    and node.move_result.get("success") is True
                )
                )
            ),
            10.0,
            "TurtleBot3 did not move after action command",
        )
        distance = math.hypot(
            node.position[0] - start[0], node.position[1] - start[1]
        )
        if distance > 0.5:
            raise RuntimeError(f"implausible odometry jump: {distance:.3f} m")
        node.action("stop", {})
        wait_until(
            lambda: node.ack is not None and node.ack.get("action") == "stop",
            5.0,
            "simulation produced no stop ACK",
        )
        print(json.dumps({
            "scan_received": node.scan_received,
            "distance_m": round(distance, 3),
            "action_ack": node.ack,
            "move_action_result": node.move_result,
        }, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
