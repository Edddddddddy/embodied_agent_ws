#!/usr/bin/env python3
"""Interactively verify microphone speech through to Gazebo odometry."""

import argparse
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class MicrophoneAcceptanceProbe(Node):
    def __init__(self, min_distance):
        super().__init__("microphone_acceptance_probe")
        self.min_distance = min_distance
        self.asr_text = None
        self.action_candidate = None
        self.action_command = None
        self.action_ack = None
        self.nonzero_cmd_vel = None
        self.initial_position = None
        self.position = None
        self.scan_received = False
        self.passed = threading.Event()
        self.create_subscription(String, "/agent/asr_final", self._on_asr, 10)
        self.create_subscription(
            String, "/agent/action_candidate", self._on_candidate, 10
        )
        self.create_subscription(
            String, "/robot/action_command", self._on_command, 10
        )
        self.create_subscription(String, "/robot/action_ack", self._on_ack, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)

    def _on_asr(self, message):
        self.asr_text = message.data
        print(f"[1/6] ASR final: {self.asr_text}", flush=True)

    def _on_candidate(self, message):
        self.action_candidate = self._decode(message.data)
        print(f"[2/6] action candidate: {self.action_candidate}", flush=True)

    def _on_command(self, message):
        self.action_command = self._decode(message.data)
        print(f"[3/6] guarded command: {self.action_command}", flush=True)

    def _on_ack(self, message):
        payload = self._decode(message.data)
        if payload.get("backend") == "simulation":
            self.action_ack = payload
            print(f"[4/6] simulation ACK: {self.action_ack}", flush=True)
            self._check_passed()

    def _on_velocity(self, message):
        if abs(message.linear.x) > 0.01 or abs(message.angular.z) > 0.01:
            first_velocity = self.nonzero_cmd_vel is None
            self.nonzero_cmd_vel = {
                "linear_x": message.linear.x,
                "angular_z": message.angular.z,
            }
            if first_velocity:
                print(f"[5/6] cmd_vel: {self.nonzero_cmd_vel}", flush=True)
            self._check_passed()

    def _on_odom(self, message):
        self.position = (
            message.pose.pose.position.x,
            message.pose.pose.position.y,
        )
        if self.initial_position is None:
            self.initial_position = self.position
        self._check_passed()

    def _on_scan(self, _message):
        self.scan_received = True

    @staticmethod
    def _decode(value):
        try:
            return json.loads(value)
        except json.JSONDecodeError:
            return {"raw": value}

    def ready(self):
        return (
            self.scan_received
            and self.position is not None
            and self.count_publishers("/agent/asr_final") > 0
            and self.count_publishers("/robot/action_ack") > 0
            and self.count_publishers("/audio/clean_pcm") > 0
        )

    def distance(self):
        if self.initial_position is None or self.position is None:
            return 0.0
        return math.hypot(
            self.position[0] - self.initial_position[0],
            self.position[1] - self.initial_position[1],
        )

    def _check_passed(self):
        if (
            self.asr_text
            and self.action_candidate
            and self.action_command
            and self.action_ack
            and self.action_ack.get("action") == "move"
            and self.action_ack.get("status") == "accepted"
            and self.nonzero_cmd_vel
            and self.min_distance <= self.distance() <= 0.5
        ):
            self.passed.set()

    def report(self):
        return {
            "asr_text": self.asr_text,
            "action_candidate": self.action_candidate,
            "action_command": self.action_command,
            "action_ack": self.action_ack,
            "cmd_vel": self.nonzero_cmd_vel,
            "distance_m": round(self.distance(), 3),
        }

    def missing_stages(self):
        checks = {
            "ASR final": self.asr_text,
            "action candidate": self.action_candidate,
            "guarded action command": self.action_command,
            "simulation ACK": self.action_ack,
            "nonzero cmd_vel": self.nonzero_cmd_vel,
            "Gazebo odometry movement": self.distance() >= self.min_distance,
        }
        return [name for name, value in checks.items() if not value]


def wait_until(predicate, timeout):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.1)
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--ready-timeout", type=float, default=40.0)
    parser.add_argument("--min-distance", type=float, default=0.03)
    parser.add_argument("--wake-word", action="store_true")
    args = parser.parse_args()

    rclpy.init()
    node = MicrophoneAcceptanceProbe(args.min_distance)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        if not wait_until(node.ready, args.ready_timeout):
            raise TimeoutError("Agent, audio frontend, scan or odometry was not ready")
        time.sleep(1.0)
        phrase = "小智，向前走一秒" if args.wake_word else "向前走一秒"
        print("\n系统已就绪。请靠近麦克风，清晰说：", flush=True)
        print(f"    {phrase}\n", flush=True)
        if not node.passed.wait(args.timeout):
            print(json.dumps(node.report(), ensure_ascii=False, indent=2))
            missing = "、".join(node.missing_stages())
            raise TimeoutError(f"验收超时，未完成环节：{missing}")
        print("[6/6] Gazebo odometry movement confirmed", flush=True)
        print(json.dumps(node.report(), ensure_ascii=False, indent=2))
        print("PASS: microphone speech -> LLM action -> Gazebo motion")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
