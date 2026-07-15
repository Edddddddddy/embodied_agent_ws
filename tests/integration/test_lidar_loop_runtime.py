#!/usr/bin/env python3
"""验证 LiDAR 回环候选与几何验证组件的 typed ROS 2 运行时契约。"""

from __future__ import annotations

import math
import time

import rclpy
from embodied_agent_interfaces.msg import (
    LidarLoopCandidate,
    LidarLoopCandidateArray,
    LidarLoopVerificationArray,
)
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Probe(Node):
    def __init__(self) -> None:
        super().__init__("lidar_loop_runtime_probe")
        self.scans = self.create_publisher(
            LaserScan, "/scan", qos_profile_sensor_data
        )
        self.odometry = self.create_publisher(
            Odometry, "/odom", qos_profile_sensor_data
        )
        self.batches: list[LidarLoopCandidateArray] = []
        self.create_subscription(
            LidarLoopCandidateArray,
            "/slam/loop_candidates",
            self.batches.append,
            10,
        )
        self.verifications: list[LidarLoopVerificationArray] = []
        self.create_subscription(
            LidarLoopVerificationArray,
            "/slam/loop_verifications",
            self.verifications.append,
            10,
        )
        self.candidates = self.create_publisher(
            LidarLoopCandidateArray, "/slam/loop_candidates", 10
        )
        self.candidate_change_state = self.create_client(
            ChangeState, "/lidar_loop_candidate/change_state"
        )
        self.verifier_change_state = self.create_client(
            ChangeState, "/lidar_loop_verifier/change_state"
        )

    def transition(self, client, transition_id: int) -> None:
        if not client.wait_for_service(timeout_sec=5.0):
            raise TimeoutError(f"Lifecycle service not discovered: {client.srv_name}")
        request = ChangeState.Request()
        request.transition.id = transition_id
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError(f"Lifecycle transition {transition_id} failed")

    @staticmethod
    def set_stamp(message, stamp_s: float) -> None:
        message.sec = int(stamp_s)
        message.nanosec = int((stamp_s - int(stamp_s)) * 1e9)

    def publish_scan(self, stamp_s: float, nanosec: int | None = None) -> None:
        message = LaserScan()
        message.header.frame_id = "laser"
        if nanosec is None:
            self.set_stamp(message.header.stamp, stamp_s)
        else:
            message.header.stamp.sec = int(stamp_s)
            message.header.stamp.nanosec = nanosec
        message.angle_min = -math.pi
        message.angle_increment = 2.0 * math.pi / 60.0
        message.angle_max = message.angle_min + 59.0 * message.angle_increment
        message.range_min = 0.1
        message.range_max = 10.0
        # 非对称半径分布避免测试只覆盖完全对称、偏航不可观的退化输入。
        message.ranges = [1.0 + 0.04 * float((index * 3) % 11) for index in range(60)]
        self.scans.publish(message)

    def publish_odometry(
        self, stamp_s: float, nanosec: int | None = None, x_m: float = 0.0
    ) -> None:
        message = Odometry()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        if nanosec is None:
            self.set_stamp(message.header.stamp, stamp_s)
        else:
            message.header.stamp.sec = int(stamp_s)
            message.header.stamp.nanosec = nanosec
        message.pose.pose.position.x = x_m
        message.pose.pose.orientation.w = 1.0
        self.odometry.publish(message)

    def publish_observation(
        self, stamp_s: float, nanosec: int | None = None, x_m: float = 0.0
    ) -> None:
        # 先发里程计再发激光，主测试验证正常顺序；后续 pending 用例再显式打乱顺序。
        self.publish_odometry(stamp_s, nanosec, x_m)
        self.spin_for(0.05)
        self.publish_scan(stamp_s, nanosec)

    def publish_candidate_before_query_scan(
        self,
        query_sec: int,
        query_nanosec: int,
        candidate_sec: int,
        candidate_nanosec: int,
        query_id: int = 99,
    ) -> None:
        batch = LidarLoopCandidateArray()
        batch.header.frame_id = "laser"
        batch.header.stamp.sec = query_sec
        batch.header.stamp.nanosec = query_nanosec
        batch.query_id = query_id
        batch.indexed_scans = 2
        batch.shadow_only = True
        item = LidarLoopCandidate()
        item.candidate_id = 0
        item.candidate_stamp.sec = candidate_sec
        item.candidate_stamp.nanosec = candidate_nanosec
        item.rank = 1
        item.similarity = 1.0
        item.yaw_offset_rad = 0.0
        batch.candidates = [item]
        self.candidates.publish(batch)

    def spin_for(self, duration_s: float) -> None:
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)


def main() -> None:
    rclpy.init()
    probe = Probe()
    try:
        probe.transition(probe.candidate_change_state, Transition.TRANSITION_CONFIGURE)
        probe.transition(probe.verifier_change_state, Transition.TRANSITION_CONFIGURE)
        deadline = time.monotonic() + 3.0
        while (
            probe.scans.get_subscription_count() < 2
            or probe.odometry.get_subscription_count() < 1
        ) and time.monotonic() < deadline:
            probe.spin_for(0.05)
        if (
            probe.scans.get_subscription_count() < 2
            or probe.odometry.get_subscription_count() < 1
        ):
            raise TimeoutError("candidate/verifier LaserScan/Odometry subscribers not discovered")

        # configure 后仍是 inactive：输入不能偷偷改变索引或产生候选证据。
        probe.publish_observation(1_700_000_000, 111_111_111)
        probe.spin_for(0.4)
        if probe.batches or probe.verifications:
            raise AssertionError("inactive components published loop evidence")

        probe.transition(probe.verifier_change_state, Transition.TRANSITION_ACTIVATE)
        probe.transition(probe.candidate_change_state, Transition.TRANSITION_ACTIVATE)
        scan_stamps = (
            (1_700_000_001, 123_456_789),
            (1_700_000_001, 323_456_789),
            # 中间帧与首帧相隔不足 sample_interval，不产生候选批次；它只为
            # 最后查询帧提供 0.75s 内的局部子图上下文。
            (1_700_000_001, 523_456_789),
            (1_700_000_002, 123_456_789),
        )
        for stamp_sec, stamp_nanosec in scan_stamps:
            probe.publish_observation(stamp_sec, stamp_nanosec)
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
        if (
            second.candidates[0].candidate_stamp.sec != scan_stamps[0][0]
            or second.candidates[0].candidate_stamp.nanosec != scan_stamps[0][1]
        ):
            raise AssertionError("epoch-scale ROS timestamp lost nanosecond precision")
        if len(probe.verifications) != 2:
            raise AssertionError(
                f"expected 2 verification batches, got {len(probe.verifications)}"
            )
        verified = probe.verifications[-1]
        if verified.query_id != 1 or not verified.shadow_only:
            raise AssertionError("verification batch lost query correlation/safety flag")
        if verified.matching_mode != "scan_to_submap" or not verified.verifications:
            raise AssertionError("geometry verification did not run")
        match = verified.verifications[0]
        if match.query_submap_scans < 2 or match.candidate_submap_scans < 2:
            raise AssertionError("runtime verifier did not construct two short local submaps")
        if match.source_points < 60 or match.target_points < 60:
            raise AssertionError("submap geometry did not contribute the expected points")
        if not match.available or not match.accepted:
            raise AssertionError(
                f"identical revisit should pass geometry: {match.rejection_reason}"
            )

        # 强制候选先于同时间戳查询帧到达，验证跨 topic 无全序时的 pending 逻辑。
        # 给即将到来的查询帧准备一个短时邻帧，使 pending 恢复后验证的仍是子图而非单帧。
        probe.publish_observation(1_700_000_005, 355_555_555)
        probe.spin_for(0.2)
        probe.publish_candidate_before_query_scan(
            1_700_000_005,
            555_555_555,
            scan_stamps[0][0],
            scan_stamps[0][1],
        )
        probe.spin_for(0.2)
        if any(batch.query_id == 99 for batch in probe.verifications):
            raise AssertionError("candidate should wait briefly for its query scan")
        probe.publish_scan(1_700_000_005, 555_555_555)
        probe.spin_for(0.2)
        if any(batch.query_id == 99 for batch in probe.verifications):
            raise AssertionError("candidate must also wait for late query odometry")
        probe.publish_odometry(1_700_000_005, 555_555_555)
        probe.spin_for(0.4)
        pending_result = next(
            (batch for batch in probe.verifications if batch.query_id == 99), None
        )
        if pending_result is None or not pending_result.verifications[0].accepted:
            raise AssertionError("pending candidate was not resumed after query scan arrived")

        # 查询端已就绪、候选端扫描尚未到达时也必须等待，而不是过早发布 unavailable。
        probe.publish_candidate_before_query_scan(
            1_700_000_005,
            555_555_555,
            1_700_000_006,
            100_000_000,
            query_id=100,
        )
        probe.spin_for(0.2)
        if any(batch.query_id == 100 for batch in probe.verifications):
            raise AssertionError("batch must wait for late candidate geometry")
        probe.publish_observation(1_700_000_005, 900_000_000)
        probe.spin_for(0.1)
        probe.publish_observation(1_700_000_006, 100_000_000)
        probe.spin_for(0.4)
        late_candidate = next(
            (batch for batch in probe.verifications if batch.query_id == 100), None
        )
        if late_candidate is None or not late_candidate.verifications[0].accepted:
            raise AssertionError("late candidate geometry did not resume the pending batch")

        # 数据流停止时不能无限 pending；steady-clock timer 应发布明确的缺帧结果。
        probe.publish_candidate_before_query_scan(
            1_700_000_008,
            800_000_000,
            scan_stamps[0][0],
            scan_stamps[0][1],
            query_id=101,
        )
        probe.spin_for(0.8)
        expired = next(
            (batch for batch in probe.verifications if batch.query_id == 101), None
        )
        if (
            expired is None
            or expired.verifications[0].rejection_reason != "query_scan_not_cached"
        ):
            raise AssertionError("wall-clock timeout did not flush a stalled pending batch")

        probe.transition(probe.candidate_change_state, Transition.TRANSITION_DEACTIVATE)
        probe.transition(probe.verifier_change_state, Transition.TRANSITION_DEACTIVATE)
        probe.transition(probe.candidate_change_state, Transition.TRANSITION_CLEANUP)
        probe.transition(probe.verifier_change_state, Transition.TRANSITION_CLEANUP)
        probe.transition(probe.candidate_change_state, Transition.TRANSITION_CONFIGURE)
        probe.transition(probe.verifier_change_state, Transition.TRANSITION_CONFIGURE)
        probe.transition(probe.verifier_change_state, Transition.TRANSITION_ACTIVATE)
        probe.transition(probe.candidate_change_state, Transition.TRANSITION_ACTIVATE)
        previous_count = len(probe.batches)
        probe.publish_observation(1_700_000_010, 101_010_101)
        probe.spin_for(0.4)
        if len(probe.batches) != previous_count + 1:
            raise AssertionError("reactivated component did not publish")
        reset_batch = probe.batches[-1]
        if reset_batch.query_id != 0 or reset_batch.candidates:
            raise AssertionError("cleanup must clear the in-memory descriptor index")

        print(
            "PASS: Lifecycle LaserScan+Odometry -> typed candidates -> shadow submap geometry; "
            "sampling, timestamp association, pending correlation, ICP gates and cleanup verified"
        )
    finally:
        probe.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
