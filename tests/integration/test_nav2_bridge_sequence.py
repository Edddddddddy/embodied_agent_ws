#!/usr/bin/env python3
"""Verify voice navigation commands are bridged to Nav2 action goals.

这里用轻量 fake Nav2 action server 代替完整 Nav2 bringup。它不做路径规划，
只检查 `Nav2RobotExecutor` 是否真的把语义地点转换为 NavigateToPose /
FollowWaypoints goal。这样 CI 不需要地图和 Gazebo，也能覆盖最关键的 bridge seam。
"""

import json
import threading
import time

import rclpy
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from rclpy.action import ActionServer
from rclpy.node import Node
from std_msgs.msg import String


class FakeNav2BridgeProbe(Node):
    def __init__(self):
        super().__init__("fake_nav2_bridge_probe")
        self.text_pub = self.create_publisher(String, "/agent/text_input", 10)
        self.candidates = []
        self.results = []
        self.navigate_goals = []
        self.follow_goals = []
        self.create_subscription(String, "/agent/action_candidate", self._on_candidate, 10)
        self.create_subscription(String, "/robot/action_result", self._on_result, 10)
        self.navigate_server = ActionServer(
            self, NavigateToPose, "navigate_to_pose", self._execute_navigate
        )
        self.follow_server = ActionServer(
            self, FollowWaypoints, "follow_waypoints", self._execute_follow
        )

    def _on_candidate(self, message):
        self.candidates.append(json.loads(message.data))

    def _on_result(self, message):
        self.results.append(json.loads(message.data))

    def _execute_navigate(self, goal_handle):
        self.navigate_goals.append(goal_handle.request.pose)
        goal_handle.succeed()
        result = NavigateToPose.Result()
        result.error_code = NavigateToPose.Result.NONE
        return result

    def _execute_follow(self, goal_handle):
        self.follow_goals.append(goal_handle.request)
        goal_handle.succeed()
        result = FollowWaypoints.Result()
        result.error_code = FollowWaypoints.Result.NONE
        return result


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def main():
    rclpy.init()
    node = FakeNav2BridgeProbe()
    executor = rclpy.executors.MultiThreadedExecutor()
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
            "Nav2 bridge pipeline was not discovered",
        )
        time.sleep(0.5)

        navigate_started_at = time.monotonic()
        node.text_pub.publish(String(data="去门口"))
        wait_until(
            lambda: node.navigate_goals
            and any(candidate.get("name") == "navigate_to" for candidate in node.candidates),
            10.0,
            "NavigateToPose goal was not received",
        )
        navigate_candidate = next(
            candidate for candidate in node.candidates
            if candidate.get("name") == "navigate_to"
        )
        navigate_id = navigate_candidate["request_id"]
        wait_until(
            lambda: any(
                result.get("command_id") == navigate_id and result.get("success") is True
                for result in node.results
            ),
            12.0,
            "navigate_to command did not finish",
        )
        navigate_elapsed = time.monotonic() - navigate_started_at
        if navigate_elapsed > 2.5:
            raise RuntimeError(
                "navigate_to result waited for local duration instead of Nav2 result: "
                f"{navigate_elapsed:.2f}s"
            )
        pose = node.navigate_goals[-1]
        if pose.header.frame_id != "map":
            raise RuntimeError(f"unexpected nav frame: {pose.header.frame_id}")
        if abs(pose.pose.position.x - 1.2) > 1e-6 or abs(pose.pose.position.y) > 1e-6:
            raise RuntimeError(f"unexpected nav pose: {pose}")

        follow_started_at = time.monotonic()
        node.text_pub.publish(String(data="依次去门口、书桌、起点"))
        wait_until(
            lambda: node.follow_goals
            and any(candidate.get("name") == "follow_waypoints" for candidate in node.candidates),
            10.0,
            "FollowWaypoints goal was not received",
        )
        follow_candidate = next(
            candidate for candidate in node.candidates
            if candidate.get("name") == "follow_waypoints"
        )
        follow_id = follow_candidate["request_id"]
        wait_until(
            lambda: any(
                result.get("command_id") == follow_id and result.get("success") is True
                for result in node.results
            ),
            15.0,
            "follow_waypoints command did not finish",
        )
        follow_elapsed = time.monotonic() - follow_started_at
        if follow_elapsed > 3.0:
            raise RuntimeError(
                "follow_waypoints result waited for local duration instead of Nav2 result: "
                f"{follow_elapsed:.2f}s"
            )
        follow = node.follow_goals[-1]
        if follow.number_of_loops != 1 or len(follow.poses) != 3:
            raise RuntimeError(f"unexpected follow goal: {follow}")
        positions = [(pose.pose.position.x, pose.pose.position.y) for pose in follow.poses]
        expected = [(1.2, 0.0), (1.2, 1.0), (0.0, 0.0)]
        for actual, want in zip(positions, expected):
            if abs(actual[0] - want[0]) > 1e-6 or abs(actual[1] - want[1]) > 1e-6:
                raise RuntimeError(f"unexpected waypoint positions: {positions}")

        print(
            json.dumps(
                {
                    "navigate_pose": {
                        "frame_id": pose.header.frame_id,
                        "x": pose.pose.position.x,
                        "y": pose.pose.position.y,
                    },
                    "follow_waypoints": positions,
                    "result_latency_s": {
                        "navigate_to": round(navigate_elapsed, 3),
                        "follow_waypoints": round(follow_elapsed, 3),
                    },
                    "result_count": len(node.results),
                    "status": "PASS",
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    finally:
        node.navigate_server.destroy()
        node.follow_server.destroy()
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
