#!/usr/bin/env python3
"""用建图产物完成 AMCL 定位与 Nav2 目标规划/执行。"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class LocalizationNavigationProbe(Node):
    def __init__(self) -> None:
        super().__init__("slam_localization_navigation_probe")
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.nav_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.bt_state_client = self.create_client(GetState, "/bt_navigator/get_state")
        self.map: OccupancyGrid | None = None
        self.amcl_poses: list[tuple[float, float]] = []
        self.odom_positions: list[tuple[float, float]] = []
        self.plan_count = 0
        self.longest_plan_points = 0
        self.last_cmd: Twist | None = None
        self.localization_tf_count = 0
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._on_amcl, 10
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(NavPath, "/plan", self._on_plan, 10)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 20)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 100)

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map = message

    def _on_amcl(self, message: PoseWithCovarianceStamped) -> None:
        position = message.pose.pose.position
        self.amcl_poses.append((position.x, position.y))

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.odom_positions.append((position.x, position.y))

    def _on_plan(self, message: NavPath) -> None:
        self.plan_count += 1
        self.longest_plan_points = max(self.longest_plan_points, len(message.poses))

    def _on_cmd(self, message: Twist) -> None:
        self.last_cmd = message

    def _on_tf(self, message: TFMessage) -> None:
        self.localization_tf_count += sum(
            transform.header.frame_id == "map" and transform.child_frame_id == "odom"
            for transform in message.transforms
        )

    def publish_initial_pose(self, x: float, y: float, yaw: float) -> None:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(yaw * 0.5)
        message.pose.covariance[0] = 0.09
        message.pose.covariance[7] = 0.09
        message.pose.covariance[35] = 0.03
        for _ in range(12):
            message.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.10)


def spin_until(node: Node, predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise TimeoutError(description)


def traveled_distance(positions: list[tuple[float, float]]) -> float:
    if len(positions) < 2:
        return 0.0
    start = positions[0]
    return max(math.hypot(x - start[0], y - start[1]) for x, y in positions)


def wait_for_bt_active(node: LocalizationNavigationProbe, timeout: float) -> None:
    if not node.bt_state_client.wait_for_service(timeout_sec=timeout):
        raise TimeoutError("bt_navigator lifecycle service unavailable")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        future = node.bt_state_client.call_async(GetState.Request())
        spin_until(node, future.done, 3.0, "bt_navigator lifecycle response timeout")
        if future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
            return
        time.sleep(0.2)
    raise TimeoutError("bt_navigator did not reach ACTIVE")


def main() -> int:
    parser = argparse.ArgumentParser()
    # slam_toolbox 把建图首帧定义为 map 原点；它与 Gazebo 世界坐标不是同一坐标系。
    parser.add_argument("--start-x", type=float, default=0.0)
    parser.add_argument("--start-y", type=float, default=0.0)
    parser.add_argument("--goal-x", type=float, default=2.10)
    parser.add_argument("--goal-y", type=float, default=0.0)
    parser.add_argument("--timeout", type=float, default=140.0)
    parser.add_argument("--output", type=Path, default=Path("logs/slam_navigation_report.json"))
    args = parser.parse_args()

    rclpy.init()
    node = LocalizationNavigationProbe()
    started = time.monotonic()
    status = GoalStatus.STATUS_UNKNOWN
    try:
        spin_until(node, lambda: node.map is not None, 45.0, "saved map was not published")
        if not node.nav_client.wait_for_server(timeout_sec=45.0):
            raise TimeoutError("NavigateToPose action server unavailable")
        node.publish_initial_pose(args.start_x, args.start_y, 0.0)
        # AMCL 静止时不一定发布 /amcl_pose，但只要 map->odom 出现，定位链就已可供 Nav2 使用。
        spin_until(
            node,
            lambda: node.localization_tf_count > 0,
            30.0,
            "AMCL did not publish map->odom",
        )
        # Action server 在 Lifecycle INACTIVE 时已经可发现，但会拒绝 goal；必须显式等 ACTIVE。
        wait_for_bt_active(node, 45.0)

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = node.get_clock().now().to_msg()
        goal.pose.pose.position.x = args.goal_x
        goal.pose.pose.position.y = args.goal_y
        goal.pose.pose.orientation.w = 1.0
        goal_future = node.nav_client.send_goal_async(goal)
        spin_until(node, goal_future.done, 15.0, "Nav2 goal response timeout")
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("Nav2 rejected the navigation goal")
        result_future = goal_handle.get_result_async()
        spin_until(node, result_future.done, args.timeout, "Nav2 navigation result timeout")
        status = result_future.result().status
        # result 到达后继续观察零速，验证控制器确实安全收尾。
        spin_until(
            node,
            lambda: node.last_cmd is not None
            and abs(node.last_cmd.linear.x) < 1e-3
            and abs(node.last_cmd.angular.z) < 1e-3,
            8.0,
            "cmd_vel did not return to zero",
        )
        distance = traveled_distance(node.odom_positions)
        checks = {
            "saved_map_loaded": node.map is not None,
            "amcl_localization_tf_published": node.localization_tf_count > 0,
            "global_plan_published": node.plan_count > 0 and node.longest_plan_points >= 5,
            "navigate_to_pose_succeeded": status == GoalStatus.STATUS_SUCCEEDED,
            "robot_moved_at_least_1m": distance >= 1.0,
            "cmd_vel_returned_to_zero": node.last_cmd is not None
            and abs(node.last_cmd.linear.x) < 1e-3
            and abs(node.last_cmd.angular.z) < 1e-3,
        }
        report = {
            "passed": all(checks.values()),
            "elapsed_s": round(time.monotonic() - started, 3),
            "checks": checks,
            "map": {
                "width": node.map.info.width if node.map else 0,
                "height": node.map.info.height if node.map else 0,
                "resolution_m": node.map.info.resolution if node.map else 0.0,
            },
            "amcl_pose_count": len(node.amcl_poses),
            "localization_tf_count": node.localization_tf_count,
            "global_plan_count": node.plan_count,
            "longest_plan_points": node.longest_plan_points,
            "odom_traveled_distance_m": round(distance, 4),
            "navigate_to_pose_status": status,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print("PASS: saved map -> AMCL -> planner -> controller" if report["passed"] else "FAIL: localization/navigation gate")
        return 0 if report["passed"] else 1
    except Exception as error:  # noqa: BLE001 - integration probe must persist failure evidence
        report = {"passed": False, "error": str(error), "elapsed_s": round(time.monotonic() - started, 3)}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
