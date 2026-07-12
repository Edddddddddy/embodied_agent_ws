#!/usr/bin/env python3
"""Verify that a trusted voice-style action physically moves TurtleBot3 in Gazebo."""

import json
import math
import os
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import (
    BehaviorTreeStatus,
    RobotActionAck,
    RobotCommand,
    RobotCommandResult,
)
from embodied_agent_core.runtime_status_transport import (
    action_ack_to_dict,
    behavior_tree_status_to_dict,
)
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from typed_action_test_utils import candidate_message, result_dict


class GazeboProbe(Node):
    def __init__(self):
        super().__init__("gazebo_motion_probe")
        self.action_pub = self.create_publisher(
            RobotCommand, "/agent/action_candidate", 10
        )
        self.position = None
        self.scan_received = False
        self.ack = None
        self.action_result = None
        self.move_result = None
        self.move_command_id = ""
        self.move_bt_result = None
        self.command_sequence = 0
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(RobotActionAck, "/robot/action_ack", self._on_ack, 10)
        self.create_subscription(
            RobotCommandResult, "/robot/action_result", self._on_result, 10
        )
        self.create_subscription(BehaviorTreeStatus, "/robot/bt_status", self._on_bt, 10)

    def _on_odom(self, message):
        self.position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )

    def _on_scan(self, _message):
        self.scan_received = True

    def _on_ack(self, message):
        self.ack = action_ack_to_dict(message)

    def _on_result(self, message):
        self.action_result = result_dict(message)
        if self.action_result.get("command_id") == self.move_command_id:
            self.move_result = self.action_result

    def _on_bt(self, message):
        status = behavior_tree_status_to_dict(message)
        if status.get("outcome") == "succeeded":
            self.move_bt_result = status

    def action(self, name, arguments, *, priority=False):
        self.command_sequence += 1
        command_id = f"gazebo-probe-{self.command_sequence}"
        self.action_pub.publish(
            candidate_message(
                name,
                arguments,
                request_id=command_id,
                priority=priority,
            )
        )
        return command_id


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
        node.move_command_id = node.action(
            "move", {"linear_x": 0.18, "duration_s": 2.0}
        )
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
                    and node.move_bt_result is not None
                    and node.move_bt_result.get("stage") == "confirm"
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
        node.action("stop", {}, priority=True)
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
            "move_bt_result": node.move_bt_result,
        }, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
