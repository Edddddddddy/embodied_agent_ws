#!/usr/bin/env python3
"""验证 LiDAR 回环候选 Lifecycle 组件的 typed ROS 2 运行时契约。"""

from __future__ import annotations

import math
import time

import rclpy
from embodied_agent_interfaces.msg import LidarLoopCandidateArray
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("lidar_loop_runtime_probe")
        self.scans = self.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        self.batches: list[LidarLoopCandidateArray] = []
        self.create_subscription(
            LidarLoopCandidateArray,
            "/slam/loop_candidates",
            self.batches.append,
            10,
        )
        self.change_state = self.create_client(
            ChangeState, "/lidar_loop_candidate/change_state"
        )

    def transition(self, transition_id: int) -> None:
        if not self.change_state.wait_for_service(timeout_sec=5.0):
            raise TimeoutError("Lifecycle change_state service not discovered")
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = self.change_state.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError(f"Lifecycle transition {transition_id} failed")

    def publish_scan(self, stamp_s: float) -> None:
        message = LaserScan()
        message.header.frame_id = "laser"
        message.header.stamp.sec = int(stamp_s)
        message.header.stamp.nanosec = int((stamp_s - int(stamp_s)) * 1e9)
        message.angle_min = -math.pi
        message.angle_increment = 2.0 * math.pi / 60.0
        message.angle_max = message.angle_min + 59.0 * message.angle_increment
        message.range_min = 0.1
        message.range_max = 10.0
        # 非对称半径分布避免测试只覆盖完全对称、偏航不可观的退化输入。
        message.ranges = [1.0 + 0.04 * float((index * 3) % 11) for index in range(60)]
        self.scans.publish(message)

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> None:
    rclpy.init()
    probe = Probe()
    try:
        probe.transition(Transition.TRANSITION_CONFIGURE)
        deadline = time.monotonic() + 3.0
        while probe.scans.get_subscription_count() < 1 and time.monotonic() < deadline:
            probe.spin_for(0.05)
        if probe.scans.get_subscription_count() < 1:
            raise TimeoutError("LaserScan subscriber not discovered")

        # configure 后仍是 inactive：输入不能偷偷改变索引或产生候选证据。
        probe.publish_scan(0.0)
        probe.spin_for(0.4)
        if probe.batches:
            raise AssertionError("inactive component published a candidate batch")

        probe.transition(Transition.TRANSITION_ACTIVATE)
        for stamp_s in (1.0, 1.2, 2.2):
            probe.publish_scan(stamp_s)
            probe.spin_for(0.25)

        if len(probe.batches) != 2:
            raise AssertionError(f"expected 2 sampled batches, got {len(probe.batches)}")
        first, second = probe.batches
        if first.query_id != 0 or first.indexed_scans != 1 or first.candidates:
            raise AssertionError("first scan must initialize the index without self-match")
        if second.query_id != 1 or second.indexed_scans != 2:
            raise AssertionError("sample interval or deterministic query ids regressed")
        if not second.shadow_only or not second.candidates:
            raise AssertionError("eligible revisit must publish shadow-only typed candidates")
        if second.candidates[0].candidate_id != 0 or second.candidates[0].rank != 1:
            raise AssertionError("candidate correlation/ranking is incorrect")

        probe.transition(Transition.TRANSITION_DEACTIVATE)
        probe.transition(Transition.TRANSITION_CLEANUP)
        probe.transition(Transition.TRANSITION_CONFIGURE)
        probe.transition(Transition.TRANSITION_ACTIVATE)
        previous_count = len(probe.batches)
        probe.publish_scan(10.0)
        probe.spin_for(0.4)
        if len(probe.batches) != previous_count + 1:
            raise AssertionError("reactivated component did not publish")
        reset_batch = probe.batches[-1]
        if reset_batch.query_id != 0 or reset_batch.candidates:
            raise AssertionError("cleanup must clear the in-memory descriptor index")

        print(
            "PASS: Lifecycle LaserScan -> typed shadow loop candidates; "
            "sampling, correlation and cleanup verified"
        )
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
