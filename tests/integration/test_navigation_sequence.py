#!/usr/bin/env python3
"""Verify voice-style navigation commands reach the typed simulation pipeline.

本测试不依赖真实麦克风和真实 Nav2 地图，而是用 `/agent/text_input` 模拟
ASR final。它验证的是本阶段最重要的工程闭环：

文本命令 → 轻量 NLU → ActionGuard → RobotCommand → ROS 2 Action →
仿真 executor → /cmd_vel 与 /robot/action_result。
"""

import json
import threading
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class NavigationSequenceProbe(Node):
    def __init__(self):
        super().__init__("navigation_sequence_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.candidates = []
        self.results = []
        self.velocities = []
        self.create_subscription(String, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(String, "/robot/action_result", self._on_result, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_velocity, 10)

    def _on_candidate(self, message):
        self.candidates.append(json.loads(message.data))

    def _on_result(self, message):
        self.results.append(json.loads(message.data))

    def _on_velocity(self, message):
        self.velocities.append((message.linear.x, message.angular.z))


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def _has_result_for(node, command_id: str) -> bool:
    return any(
        result.get("command_id") == command_id and result.get("success") is True
        for result in node.results
    )


def _latest_candidate(node, name: str):
    for candidate in reversed(node.candidates):
        if candidate.get("name") == name:
            return candidate
    return None


def main():
    rclpy.init()
    node = NavigationSequenceProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.text_pub.get_subscription_count() > 0
                and node.count_publishers("/robot/action_result") > 0
            ),
            15.0,
            "navigation pipeline was not discovered",
        )
        time.sleep(0.5)

        # 目标点导航：当前版本用语义地点 target 驱动仿真 executor，
        # 后续接 Nav2 时这里可以自然替换成 NavigateToPose action client。
        node.text_pub.publish(String(data="去门口"))
        wait_until(
            lambda: _latest_candidate(node, "navigate_to") is not None,
            10.0,
            "navigate_to candidate was not published",
        )
        navigate_candidate = _latest_candidate(node, "navigate_to")
        navigate_id = navigate_candidate["request_id"]
        wait_until(
            lambda: _has_result_for(node, navigate_id)
            and any(abs(linear) > 0.01 for linear, _ in node.velocities),
            15.0,
            "navigate_to did not finish or did not produce forward velocity",
        )

        # 多目标点巡航：一句话里带出多个语义地点，NLU 输出 follow_waypoints。
        node.text_pub.publish(String(data="依次去门口、书桌、起点"))
        wait_until(
            lambda: _latest_candidate(node, "follow_waypoints") is not None,
            10.0,
            "follow_waypoints candidate was not published",
        )
        patrol_candidate = _latest_candidate(node, "follow_waypoints")
        patrol_id = patrol_candidate["request_id"]
        wait_until(
            lambda: _has_result_for(node, patrol_id)
            and any(abs(linear) > 0.01 and abs(angular) > 0.01 for linear, angular in node.velocities),
            20.0,
            "follow_waypoints did not finish or no patrol velocity was observed",
        )

        navigate_args = navigate_candidate.get("arguments", {})
        patrol_args = patrol_candidate.get("arguments", {})
        if navigate_args.get("target") != "door":
            raise RuntimeError(f"unexpected navigation target: {navigate_candidate}")
        if patrol_args.get("waypoints") != ["door", "desk", "home"]:
            raise RuntimeError(f"unexpected patrol waypoints: {patrol_candidate}")

        print(
            json.dumps(
                {
                    "navigate_to": navigate_args,
                    "follow_waypoints": patrol_args,
                    "results": [result.get("command_id") for result in node.results],
                    "motion": {
                        "forward_observed": True,
                        "arc_patrol_observed": True,
                    },
                    "status": "PASS",
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
