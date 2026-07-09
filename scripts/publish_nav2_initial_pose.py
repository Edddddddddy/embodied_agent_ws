#!/usr/bin/env python3
"""Publish AMCL initial pose for the Nav2 TurtleBot3 voice demo."""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node


def quaternion_from_yaw(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def build_initial_pose(node: Node, x: float, y: float, yaw: float) -> PoseWithCovarianceStamped:
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
    return message


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--x", type=float, default=-2.0)
    parser.add_argument("--y", type=float, default=-0.5)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument("--repeat", type=int, default=10)
    parser.add_argument("--interval", type=float, default=0.2)
    args = parser.parse_args()

    rclpy.init()
    node = Node("nav2_initial_pose_publisher")
    publisher = node.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
    try:
        # 等待 AMCL 订阅者出现；如果等待不到也继续发布，便于人工启动时容错。
        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        for _ in range(max(1, args.repeat)):
            message = build_initial_pose(node, args.x, args.y, args.yaw)
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.05)
            time.sleep(max(0.0, args.interval))
        print(
            f"published /initialpose x={args.x:.3f} y={args.y:.3f} yaw={args.yaw:.3f}",
            flush=True,
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
