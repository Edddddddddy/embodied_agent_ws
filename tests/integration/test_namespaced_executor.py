#!/usr/bin/env python3
"""Verify relative topics and Action names inside a ROS namespace."""

import json
import threading
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node
from embodied_agent_interfaces.msg import BehaviorTreeStatus, RobotCommand, RobotCommandResult
from embodied_agent_core.runtime_status_transport import behavior_tree_status_to_dict
from typed_action_test_utils import result_dict


class NamespacedProbe(Node):
    def __init__(self):
        super().__init__("namespaced_executor_probe")
        prefix = "/robot1"
        self.command_pub = self.create_publisher(
            RobotCommand, prefix + "/robot/action_command_typed", 10
        )
        self.moved = False
        self.result = None
        self.bt = None
        self.diagnostic = None
        self.create_subscription(
            Twist, prefix + "/cmd_vel", self._on_velocity, 10
        )
        self.create_subscription(
            RobotCommandResult, prefix + "/robot/action_result", self._on_result, 10
        )
        self.create_subscription(
            BehaviorTreeStatus, prefix + "/robot/bt_status", self._on_bt, 10
        )
        self.create_subscription(
            DiagnosticArray, prefix + "/diagnostics", self._on_diagnostics, 10
        )

    def _on_velocity(self, message):
        self.moved = self.moved or abs(message.linear.x) > 0.01

    def _on_result(self, message):
        payload = result_dict(message)
        if payload.get("command_id") == "namespace-test":
            self.result = payload

    def _on_bt(self, message):
        payload = behavior_tree_status_to_dict(message)
        if payload.get("command_id") == "namespace-test":
            self.bt = payload

    def _on_diagnostics(self, message):
        if any(item.name == "/robot1/simulation_control" for item in message.status):
            self.diagnostic = True


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = NamespacedProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.command_pub.get_subscription_count() > 0,
            12.0,
            "namespaced typed bridge was not discovered",
        )
        wait_until(
            lambda: node.diagnostic is True,
            12.0,
            "namespaced simulation control did not become active",
        )
        command = RobotCommand()
        command.command_id = "namespace-test"
        command.source = "test"
        command.action_type = RobotCommand.MOVE
        command.linear_x = 0.15
        command.duration_s = 0.3
        node.command_pub.publish(command)
        wait_until(
            lambda: node.moved
            and node.result is not None
            and node.result.get("success") is True
            and node.bt is not None
            and node.bt.get("outcome") == "succeeded"
            and node.diagnostic is True,
            8.0,
            "namespaced execution did not complete",
        )
        print("PASS: /robot1 typed Action -> BT -> component -> diagnostics")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
