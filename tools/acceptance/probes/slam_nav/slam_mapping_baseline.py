#!/usr/bin/env python3
"""采集一次可复现闭环建图的量化证据。

该探针只观察 ROS topic，不读取 Gazebo 内部状态，也不参与控制。这样测试结果能证明
真实的 `/scan -> slam_toolbox -> /map` 和 `/cmd_vel -> /odom` 链路已经连通。
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage


def path_length(path: RosPath | None) -> float:
    if path is None:
        return 0.0
    return sum(
        math.hypot(
            current.pose.position.x - previous.pose.position.x,
            current.pose.position.y - previous.pose.position.y,
        )
        for previous, current in zip(path.poses, path.poses[1:])
    )


def closure_error(path: RosPath | None) -> float:
    if path is None or len(path.poses) < 2:
        return 0.0
    first = path.poses[0].pose.position
    last = path.poses[-1].pose.position
    return math.hypot(last.x - first.x, last.y - first.y)


def paired_ate(reference: RosPath | None, estimate: RosPath | None) -> float:
    if reference is None or estimate is None:
        return 0.0
    count = min(len(reference.poses), len(estimate.poses))
    if count == 0:
        return 0.0
    squared = 0.0
    for index in range(count):
        expected = reference.poses[index].pose.position
        actual = estimate.poses[index].pose.position
        squared += (expected.x - actual.x) ** 2 + (expected.y - actual.y) ** 2
    return math.sqrt(squared / count)


class MappingProbe(Node):
    def __init__(self) -> None:
        super().__init__("slam_mapping_baseline_probe")
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.reference_path: RosPath | None = None
        self.raw_path: RosPath | None = None
        self.map: OccupancyGrid | None = None
        self.route_complete = False
        self.route_complete_at: float | None = None
        self.last_cmd: Twist | None = None
        self.map_to_odom: tuple[float, float, float] | None = None
        self.corrected_samples: list[tuple[float, float, float, float]] = []
        self.create_subscription(
            RosPath, "/slam/reference_path", self._on_reference_path, latched
        )
        self.create_subscription(RosPath, "/slam/raw_path", self._on_raw_path, latched)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, latched)
        self.create_subscription(Bool, "/slam/route_complete", self._on_complete, latched)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 20)
        self.create_subscription(TFMessage, "/tf", self._on_tf, 100)

    def _on_reference_path(self, message: RosPath) -> None:
        self.reference_path = message
        self._capture_corrected_pose()

    def _on_raw_path(self, message: RosPath) -> None:
        self.raw_path = message
        self._capture_corrected_pose()

    def _on_map(self, message: OccupancyGrid) -> None:
        self.map = message

    def _on_complete(self, message: Bool) -> None:
        if message.data and not self.route_complete:
            self.route_complete = True
            self.route_complete_at = time.monotonic()

    def _on_cmd(self, message: Twist) -> None:
        self.last_cmd = message

    def _on_tf(self, message: TFMessage) -> None:
        for transform in message.transforms:
            if transform.header.frame_id == "map" and transform.child_frame_id == "slam_odom":
                rotation = transform.transform.rotation
                yaw = math.atan2(
                    2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                    1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
                )
                translation = transform.transform.translation
                self.map_to_odom = (translation.x, translation.y, yaw)
                self._capture_corrected_pose()

    def _capture_corrected_pose(self) -> None:
        if self.map_to_odom is None or self.reference_path is None or self.raw_path is None:
            return
        if not self.reference_path.poses or not self.raw_path.poses:
            return
        reference = self.reference_path.poses[-1].pose.position
        raw = self.raw_path.poses[-1].pose.position
        tx, ty, yaw = self.map_to_odom
        corrected_x = tx + math.cos(yaw) * raw.x - math.sin(yaw) * raw.y
        corrected_y = ty + math.sin(yaw) * raw.x + math.cos(yaw) * raw.y
        sample = (reference.x, reference.y, corrected_x, corrected_y)
        if self.corrected_samples:
            previous = self.corrected_samples[-1]
            if math.hypot(reference.x - previous[0], reference.y - previous[1]) < 0.01:
                # 机器人停止后的回环优化仍会更新 map->odom；替换末样本才能记录最终校正量。
                self.corrected_samples[-1] = sample
                return
        self.corrected_samples.append(sample)


def build_report(node: MappingProbe, elapsed_s: float, solver: str) -> dict[str, object]:
    grid = node.map
    cells = list(grid.data) if grid is not None else []
    known_cells = sum(value >= 0 for value in cells)
    occupied_cells = sum(value >= 50 for value in cells)
    free_cells = sum(0 <= value < 50 for value in cells)
    resolution = float(grid.info.resolution) if grid is not None else 0.0
    final_cmd_zero = node.last_cmd is not None and all(
        abs(value) <= 1e-3
        for value in (
            node.last_cmd.linear.x,
            node.last_cmd.linear.y,
            node.last_cmd.angular.z,
        )
    )
    reference_length = path_length(node.reference_path)
    raw_closure = closure_error(node.raw_path)
    corrected_closure = 0.0
    corrected_ate = 0.0
    if len(node.corrected_samples) >= 2:
        first = node.corrected_samples[0]
        last = node.corrected_samples[-1]
        corrected_closure = math.hypot(last[2] - first[2], last[3] - first[3])
        corrected_ate = math.sqrt(
            sum((item[0] - item[2]) ** 2 + (item[1] - item[3]) ** 2 for item in node.corrected_samples)
            / len(node.corrected_samples)
        )
    checks = {
        "route_completed": node.route_complete,
        "map_received": grid is not None,
        "map_resolution_is_5cm": abs(resolution - 0.05) <= 1e-6,
        "map_has_known_area": known_cells >= 500,
        "closed_loop_distance_at_least_8m": reference_length >= 8.0,
        # 漂移必须可测，否则后端优化 A/B 只是在比较两条几乎相同的输入。
        "controlled_raw_drift_visible": raw_closure >= 0.20,
        "backend_correction_observed": len(node.corrected_samples) >= 50,
        "backend_reduces_closure_error": corrected_closure <= raw_closure * 0.85,
        "robot_stopped_after_route": final_cmd_zero,
    }
    return {
        "passed": all(checks.values()),
        "solver": solver,
        "elapsed_s": round(elapsed_s, 3),
        "checks": checks,
        "trajectory": {
            "reference_samples": len(node.reference_path.poses) if node.reference_path else 0,
            "raw_samples": len(node.raw_path.poses) if node.raw_path else 0,
            "reference_path_length_m": round(reference_length, 4),
            "reference_closure_error_m": round(closure_error(node.reference_path), 4),
            "raw_path_length_m": round(path_length(node.raw_path), 4),
            "raw_closure_error_m": round(raw_closure, 4),
            "raw_ate_rmse_m": round(paired_ate(node.reference_path, node.raw_path), 4),
            "corrected_samples": len(node.corrected_samples),
            "corrected_closure_error_m": round(corrected_closure, 4),
            "corrected_ate_rmse_m": round(corrected_ate, 4),
        },
        "map": {
            "width": int(grid.info.width) if grid is not None else 0,
            "height": int(grid.info.height) if grid is not None else 0,
            "resolution_m": resolution,
            "known_cells": known_cells,
            "occupied_cells": occupied_cells,
            "free_cells": free_cells,
            "known_area_m2": round(known_cells * resolution * resolution, 4),
        },
        "final_cmd_vel_zero": final_cmd_zero,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=150.0)
    parser.add_argument("--settle", type=float, default=5.0)
    parser.add_argument("--solver", choices=("ceres", "gtsam"), default="ceres")
    parser.add_argument("--output", type=Path, default=Path("logs/slam_baseline_report.json"))
    args = parser.parse_args()

    rclpy.init()
    node = MappingProbe()
    started = time.monotonic()
    try:
        while rclpy.ok() and time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                node.route_complete_at is not None
                and time.monotonic() - node.route_complete_at >= args.settle
            ):
                break
        report = build_report(node, time.monotonic() - started, args.solver)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print(
            "PASS: Gazebo closed loop -> controlled odom drift -> slam_toolbox map"
            if report["passed"]
            else "FAIL: SLAM baseline did not meet the quantitative gate"
        )
        return 0 if report["passed"] else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
