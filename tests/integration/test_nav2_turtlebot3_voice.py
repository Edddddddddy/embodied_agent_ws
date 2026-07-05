#!/usr/bin/env python3
"""Drive the real Nav2/TurtleBot3 voice launch with text-input commands.

该测试面向人工/重型验收：外层脚本会启动官方 Nav2 TurtleBot3 仿真，
这里只模拟 ASR final，把“去门口”“依次去门口、书桌、起点”注入 Agent。
通过标准不是简单看到 candidate，而是等待 Nav2 executor 返回 action result，
并观察 /odom 有运动，证明语音语义已经进入真实 Nav2 控制链路。
"""

import argparse
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String


class Nav2TurtleBot3VoiceProbe(Node):
    def __init__(self):
        super().__init__("nav2_turtlebot3_voice_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.candidates = []
        self.results = []
        self.positions = []
        self.create_subscription(String, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(String, "/robot/action_result", self._on_result, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_candidate(self, message):
        self.candidates.append(json.loads(message.data))

    def _on_result(self, message):
        self.results.append(json.loads(message.data))

    def _on_odom(self, message):
        position = message.pose.pose.position
        self.positions.append((position.x, position.y))


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.1)
    raise TimeoutError(description)


def latest_candidate(node, name):
    for candidate in reversed(node.candidates):
        if candidate.get("name") == name:
            return candidate
    return None


def has_success_result(node, command_id):
    return any(
        result.get("command_id") == command_id and result.get("success") is True
        for result in node.results
    )


def traveled_distance(positions):
    if len(positions) < 2:
        return 0.0
    start = positions[0]
    return max(math.hypot(x - start[0], y - start[1]) for x, y in positions)


def quaternion_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def publish_initial_pose(node, x, y, yaw):
    """Seed AMCL before sending Nav2 goals.

    官方 TurtleBot3/Nav2 bringup 不一定自动给 AMCL 初始位姿；没有 map->odom TF 时
    Nav2 goal 会一直卡在 costmap/transform 等待。这里用 launch 默认出生点给 AMCL
    一个近似初值，足够支撑演示级目标点导航验收。
    """

    qz, qw = quaternion_from_yaw(yaw)
    message = PoseWithCovarianceStamped()
    message.header.frame_id = "map"
    message.header.stamp = node.get_clock().now().to_msg()
    message.pose.pose.position.x = x
    message.pose.pose.position.y = y
    message.pose.pose.orientation.z = qz
    message.pose.pose.orientation.w = qw
    message.pose.covariance[0] = 0.25
    message.pose.covariance[7] = 0.25
    message.pose.covariance[35] = 0.0685
    for _ in range(10):
        message.header.stamp = node.get_clock().now().to_msg()
        node.initial_pose_pub.publish(message)
        time.sleep(0.2)


def run_command(node, text, candidate_name, timeout):
    before = len(node.results)
    node.text_pub.publish(String(data=text))
    wait_until(
        lambda: latest_candidate(node, candidate_name) is not None,
        20.0,
        f"{candidate_name} candidate was not published for {text!r}",
    )
    candidate = latest_candidate(node, candidate_name)
    command_id = candidate["request_id"]
    wait_until(
        lambda: len(node.results) > before and has_success_result(node, command_id),
        timeout,
        f"{candidate_name} did not finish through Nav2 for {text!r}",
    )
    return candidate


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--navigate-timeout", type=float, default=120.0)
    parser.add_argument("--patrol-timeout", type=float, default=240.0)
    parser.add_argument("--skip-patrol", action="store_true")
    parser.add_argument("--initial-x", type=float, default=-2.0)
    parser.add_argument("--initial-y", type=float, default=-0.5)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    args = parser.parse_args()

    rclpy.init()
    node = Nav2TurtleBot3VoiceProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.text_pub.get_subscription_count() > 0
                and node.count_publishers("/robot/action_result") > 0
                and node.positions
            ),
            60.0,
            "voice/Nav2/TurtleBot3 topics were not ready",
        )
        time.sleep(2.0)
        publish_initial_pose(node, args.initial_x, args.initial_y, args.initial_yaw)
        time.sleep(3.0)

        navigate_candidate = run_command(
            node, "去门口", "navigate_to", args.navigate_timeout
        )
        distance_after_nav = traveled_distance(node.positions)
        if distance_after_nav < 0.05:
            raise RuntimeError(
                f"Nav2 navigate_to returned but odom movement was too small: "
                f"{distance_after_nav:.3f} m"
            )

        patrol_candidate = None
        if not args.skip_patrol:
            patrol_candidate = run_command(
                node, "依次去门口、书桌、起点", "follow_waypoints", args.patrol_timeout
            )

        print(json.dumps({
            "navigate_to": navigate_candidate.get("arguments", {}),
            "follow_waypoints": (
                None if patrol_candidate is None else patrol_candidate.get("arguments", {})
            ),
            "result_count": len(node.results),
            "distance_m": round(traveled_distance(node.positions), 3),
            "status": "PASS",
        }, ensure_ascii=False, indent=2))
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
