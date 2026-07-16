#!/usr/bin/env python3
"""验证 PoseArray 感知输入会产生带速度的 typed 动态障碍轨迹。"""

from __future__ import annotations

import time

import rclpy
from embodied_agent_interfaces.msg import DynamicObstacleArray
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node


class TrackerProbe(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_obstacle_tracker_probe")
        self.publisher = self.create_publisher(
            PoseArray, "/perception/dynamic_obstacle_detections", 10
        )
        self.messages: list[DynamicObstacleArray] = []
        self.create_subscription(
            DynamicObstacleArray,
            "/perception/dynamic_obstacles",
            self.messages.append,
            10,
        )

    def publish_detection(self, x: float, y: float) -> None:
        message = PoseArray()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.w = 1.0
        message.poses.append(pose)
        self.publisher.publish(message)


def spin_for(node: Node, duration_s: float) -> None:
    deadline = time.monotonic() + duration_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def main() -> int:
    rclpy.init()
    node = TrackerProbe()
    try:
        # 等待 DDS discovery 后再发送两个时刻的位置，速度必须由跟踪器估计而非测试注入。
        spin_for(node, 0.8)
        node.publish_detection(0.0, 0.0)
        spin_for(node, 0.35)
        node.publish_detection(0.20, 0.0)
        spin_for(node, 0.6)
        assert len(node.messages) >= 2, "tracker did not publish typed tracks"
        track = node.messages[-1].obstacles[0]
        assert track.track_id == "track_1", track.track_id
        assert track.velocity.x > 0.15, track.velocity.x
        assert abs(track.velocity.y) < 0.05, track.velocity.y
        assert track.confidence >= 0.6, track.confidence
        print(
            "PASS: PoseArray -> global gated association -> typed velocity "
            f"id={track.track_id} vx={track.velocity.x:.3f} confidence={track.confidence:.2f}"
        )
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
