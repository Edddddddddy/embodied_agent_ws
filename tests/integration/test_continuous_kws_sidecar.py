#!/usr/bin/env python3
"""Verify keyword_wake sidecar opens the Agent continuous voice session."""

import json
import threading
import time

import rclpy
from embodied_agent_interfaces.msg import RobotCommand, RobotCommandResult
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from typed_action_test_utils import candidate_dict, result_dict


class ContinuousKwsProbe(Node):
    def __init__(self):
        super().__init__("continuous_kws_probe")
        self.kws_text_pub = self.create_publisher(String, "/agent/kws_text_input", 10)
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.kws_events = []
        self.wake_events = []
        self.candidates = []
        self.results = []
        self.velocities = []
        self.create_subscription(String, "/agent/kws_event", self._on_kws_event, 10)
        self.create_subscription(String, "/agent/wake_event", self._on_wake_event, 10)
        self.create_subscription(RobotCommand, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(RobotCommandResult, "/robot/action_result", self._on_result, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)

    def _on_kws_event(self, message):
        self.kws_events.append(json.loads(message.data))

    def _on_wake_event(self, message):
        self.wake_events.append(json.loads(message.data))

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
    node = ContinuousKwsProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.kws_text_pub.get_subscription_count() > 0
            and node.text_pub.get_subscription_count() > 0
            and node.count_publishers("/robot/action_result") > 0
            and node.count_subscribers("/robot/action_command_typed") > 0,
            15.0,
            "continuous KWS pipeline was not discovered",
        )
        time.sleep(0.5)

        # 未被 KWS 唤醒前，不带“小智”的命令必须被拒绝。
        node.text_pub.publish(String(data="向前走一秒"))
        time.sleep(0.8)
        if node.candidates:
            raise RuntimeError("command was accepted before KWS wake")

        node.kws_text_pub.publish(String(data="你好小志"))
        wait_until(
            lambda: any(event.get("provider") == "mock_kws" for event in node.kws_events),
            5.0,
            "keyword_wake sidecar did not detect the wake word",
        )
        wait_until(
            lambda: any(event.get("provider") == "mock_kws" for event in node.wake_events),
            5.0,
            "Agent did not bridge mock_kws wake event into session",
        )
        time.sleep(0.5)

        node.text_pub.publish(String(data="向前走一秒"))
        wait_until(
            lambda: any(candidate.get("name") == "move" for candidate in node.candidates),
            8.0,
            "KWS-opened session did not accept move command",
        )
        wait_until(lambda: node.results, 15.0, "move command did not finish")
        wait_until(
            lambda: node.velocities
            and abs(node.velocities[-1][0]) < 1e-6
            and abs(node.velocities[-1][1]) < 1e-6,
            8.0,
            "cmd_vel did not settle to zero",
        )

        print(json.dumps({
            "kws_events": node.kws_events,
            "wake_events": node.wake_events,
            "candidate_names": [candidate.get("name") for candidate in node.candidates],
            "result_count": len(node.results),
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
