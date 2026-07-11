#!/usr/bin/env python3
"""Exercise action guard, lidar safety, mode switching, and cmd_vel output."""

import json
import math
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import RobotCommand
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from typed_action_test_utils import candidate_message


class SimulationProbe(Node):
    def __init__(self):
        super().__init__("simulation_pipeline_probe")
        self.action_pub = self.create_publisher(
            RobotCommand, "/agent/action_candidate", 10
        )
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.last_velocity = None
        self.last_state = None
        self.mode = None
        self.action_ack = None
        self.velocity_event = threading.Event()
        self.state_event = threading.Event()
        self.mode_event = threading.Event()
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)
        self.create_subscription(String, "/robot/simulation_state", self._on_state, 10)
        self.create_subscription(String, "/robot/control_mode", self._on_mode, 10)
        self.create_subscription(String, "/robot/action_ack", self._on_ack, 10)

    def _on_velocity(self, message):
        self.last_velocity = message
        self.velocity_event.set()

    def _on_state(self, message):
        self.last_state = json.loads(message.data)
        self.state_event.set()

    def _on_mode(self, message):
        self.mode = message.data
        self.mode_event.set()

    def _on_ack(self, message):
        self.action_ack = json.loads(message.data)

    def publish_scan(self, front):
        message = LaserScan()
        message.angle_min = -math.pi
        message.angle_max = math.pi
        message.angle_increment = math.pi / 180.0
        message.range_min = 0.12
        message.range_max = 3.5
        message.ranges = [3.0] * 360
        message.ranges[180] = front
        self.scan_pub.publish(message)

    def publish_action(self, name, arguments):
        self.action_pub.publish(candidate_message(name, arguments))


def wait_until(predicate, timeout, description, tick=None):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if tick is not None:
            tick()
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = SimulationProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    try:
        wait_until(
            lambda: node.action_pub.get_subscription_count() > 0
            and node.scan_pub.get_subscription_count() > 0
            and node.count_publishers("/robot/action_ack") > 0,
            8.0,
            "simulation subscribers were not discovered",
        )
        time.sleep(1.0)
        node.publish_scan(2.0)
        node.publish_action("move", {"linear_x": 0.2, "duration_s": 2.0})
        wait_until(
            lambda: node.last_velocity is not None
            and node.last_velocity.linear.x > 0.01
            and node.action_ack is not None
            and node.action_ack.get("action") == "move",
            4.0,
            "manual action produced no forward cmd_vel",
            lambda: node.publish_scan(2.0),
        )

        node.publish_scan(0.15)
        wait_until(
            lambda: node.last_state is not None
            and node.last_state["safety_stopped"]
            and abs(node.last_state["linear_x"]) < 1e-6,
            4.0,
            "lidar emergency did not stop forward motion",
            lambda: node.publish_scan(0.15),
        )

        node.publish_action("set_mode", {"mode": "obstacle_avoidance"})
        node.publish_scan(2.0)
        wait_until(
            lambda: node.mode == "obstacle_avoidance"
            and node.last_state is not None
            and node.last_state["linear_x"] > 0.01,
            4.0,
            "obstacle avoidance mode did not move in clear space",
            lambda: node.publish_scan(2.0),
        )
        node.publish_action("stop", {})
        print(json.dumps({
            "manual_forward": True,
            "lidar_emergency_stop": True,
            "mode": node.mode,
            "action_ack": node.action_ack,
            "state": node.last_state,
        }, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
