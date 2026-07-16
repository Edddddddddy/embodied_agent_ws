#!/usr/bin/env python3
"""验证动态障碍跟踪、未来占用注入、Nav2 重规划与安全停车闭环。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from embodied_agent_interfaces.msg import DynamicObstacleArray
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


ROOT = Path(__file__).resolve().parents[3]
DEFAULT_SCENARIO = (
    ROOT
    / "src"
    / "embodied_navigation"
    / "config"
    / "dynamic_obstacle_crossing_scenario.json"
)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_scenario(path: Path) -> tuple[dict, str]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    required = {
        "schema_version",
        "scenario_id",
        "frame_id",
        "goal",
        "obstacle_x",
        "prediction_horizon_s",
        "warmup",
        "navigation",
        "thresholds",
        "evidence_scope",
    }
    missing = required - payload.keys()
    if missing:
        raise ValueError(f"scenario contract missing fields: {sorted(missing)}")
    for phase in ("warmup", "navigation"):
        if not payload[phase].get("y_positions"):
            raise ValueError(f"scenario {phase} needs at least one detection")
        if float(payload[phase].get("interval_s", 0.0)) <= 0.0:
            raise ValueError(f"scenario {phase} interval must be positive")
    return payload, sha256_file(path)


def map_artifact_sha256(map_yaml: Path) -> str:
    """同时绑定 YAML 和栅格图，避免四轮实验悄悄换了同名地图。"""
    digest = hashlib.sha256()
    digest.update(map_yaml.read_bytes())
    image_path: Path | None = None
    for line in map_yaml.read_text(encoding="utf-8").splitlines():
        key, separator, value = line.partition(":")
        if separator and key.strip() == "image":
            candidate = Path(value.strip().strip("'\""))
            image_path = candidate if candidate.is_absolute() else map_yaml.parent / candidate
            break
    if image_path is None or not image_path.is_file():
        raise ValueError(f"map image referenced by {map_yaml} is missing")
    digest.update(image_path.read_bytes())
    return digest.hexdigest()


class DynamicNavigationProbe(Node):
    def __init__(self) -> None:
        super().__init__("predicted_dynamic_obstacle_navigation_probe")
        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10
        )
        self.detection_pub = self.create_publisher(
            PoseArray, "/perception/dynamic_obstacle_detections", 10
        )
        self.navigation = ActionClient(self, NavigateToPose, "/navigate_to_pose")
        self.compute_path = ActionClient(self, ComputePathToPose, "/compute_path_to_pose")
        self.bt_state = self.create_client(GetState, "/bt_navigator/get_state")
        self.localization_tf_count = 0
        self.latest_tracks: DynamicObstacleArray | None = None
        self.latest_costmap: Costmap | None = None
        self.plans: list[NavPath] = []
        self.odom_positions: list[tuple[float, float]] = []
        self.last_cmd: Twist | None = None
        self.create_subscription(TFMessage, "/tf", self._on_tf, 100)
        self.create_subscription(
            DynamicObstacleArray,
            "/perception/dynamic_obstacles",
            self._on_tracks,
            10,
        )
        self.create_subscription(Costmap, "/global_costmap/costmap_raw", self._on_costmap, latched)
        self.create_subscription(NavPath, "/plan", self.plans.append, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 20)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd, 20)

    def _on_tf(self, message: TFMessage) -> None:
        self.localization_tf_count += sum(
            item.header.frame_id == "map" and item.child_frame_id == "odom"
            for item in message.transforms
        )

    def _on_tracks(self, message: DynamicObstacleArray) -> None:
        self.latest_tracks = message

    def _on_costmap(self, message: Costmap) -> None:
        self.latest_costmap = message

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.odom_positions.append((position.x, position.y))

    def _on_cmd(self, message: Twist) -> None:
        self.last_cmd = message

    def publish_initial_pose(self) -> None:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.09
        message.pose.covariance[7] = 0.09
        message.pose.covariance[35] = 0.03
        for _ in range(12):
            message.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

    def publish_detection(self, x: float, y: float) -> None:
        message = PoseArray()
        message.header.frame_id = "map"
        # 留空 stamp，使使用仿真时钟的跟踪器以自己的 now() 计算稳定速度。
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.w = 1.0
        message.poses.append(pose)
        self.detection_pub.publish(message)

    def cost_at(self, x: float, y: float) -> int:
        if self.latest_costmap is None:
            return -1
        metadata = self.latest_costmap.metadata
        mx = int((x - metadata.origin.position.x) / metadata.resolution)
        my = int((y - metadata.origin.position.y) / metadata.resolution)
        if mx < 0 or my < 0 or mx >= metadata.size_x or my >= metadata.size_y:
            return -1
        return self.latest_costmap.data[my * metadata.size_x + mx]


def spin_until(node: Node, predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if predicate():
            return
    raise TimeoutError(description)


def spin_for(node: Node, duration_s: float) -> None:
    deadline = time.monotonic() + duration_s
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)


def wait_for_bt_active(node: DynamicNavigationProbe, timeout: float) -> None:
    if not node.bt_state.wait_for_service(timeout_sec=timeout):
        raise TimeoutError("bt_navigator lifecycle service unavailable")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        future = node.bt_state.call_async(GetState.Request())
        spin_until(node, future.done, 3.0, "bt_navigator lifecycle response timeout")
        if future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
            return
        time.sleep(0.2)
    raise TimeoutError("bt_navigator did not reach ACTIVE")


def request_path(node: DynamicNavigationProbe, goal_x: float, goal_y: float) -> NavPath:
    goal = ComputePathToPose.Goal()
    goal.goal.header.frame_id = "map"
    goal.goal.header.stamp = node.get_clock().now().to_msg()
    goal.goal.pose.position.x = goal_x
    goal.goal.pose.position.y = goal_y
    goal.goal.pose.orientation.w = 1.0
    goal.use_start = False
    future = node.compute_path.send_goal_async(goal)
    spin_until(node, future.done, 10.0, "ComputePathToPose response timeout")
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError("planner rejected ComputePathToPose")
    result_future = handle.get_result_async()
    spin_until(node, result_future.done, 15.0, "ComputePathToPose result timeout")
    result = result_future.result()
    if result.status != GoalStatus.STATUS_SUCCEEDED or len(result.result.path.poses) < 5:
        raise RuntimeError(
            f"planner failed: status={result.status} error={result.result.error_code} "
            f"message={result.result.error_msg}"
        )
    return result.result.path


def path_clearance(path: NavPath, x: float, y: float) -> float:
    return min(
        math.hypot(pose.pose.position.x - x, pose.pose.position.y - y)
        for pose in path.poses
    )


def traveled_distance(positions: list[tuple[float, float]]) -> float:
    if len(positions) < 2:
        return 0.0
    start_x, start_y = positions[0]
    return max(math.hypot(x - start_x, y - start_y) for x, y in positions)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=160.0)
    parser.add_argument(
        "--motion-model",
        choices=("current_only", "constant_velocity", "kalman", "imm"),
        default="constant_velocity",
    )
    parser.add_argument(
        "--output", type=Path, default=Path("logs/dynamic_obstacle_navigation_report.json")
    )
    parser.add_argument("--scenario", type=Path, default=DEFAULT_SCENARIO)
    parser.add_argument("--map-file", type=Path, required=True)
    parser.add_argument("--params-file", type=Path, required=True)
    args = parser.parse_args()
    started = time.monotonic()
    scenario, scenario_sha256 = load_scenario(args.scenario)
    goal_x = float(scenario["goal"]["x"])
    goal_y = float(scenario["goal"]["y"])
    obstacle_x = float(scenario["obstacle_x"])
    prediction_horizon_s = float(scenario["prediction_horizon_s"])
    thresholds = scenario["thresholds"]
    provenance = {
        "scenario_id": scenario["scenario_id"],
        "scenario_sha256": scenario_sha256,
        "map_artifact_sha256": map_artifact_sha256(args.map_file),
        "nav2_params_sha256": sha256_file(args.params_file),
    }
    rclpy.init()
    node = DynamicNavigationProbe()
    try:
        if not node.navigation.wait_for_server(timeout_sec=45.0):
            raise TimeoutError("NavigateToPose action server unavailable")
        if not node.compute_path.wait_for_server(timeout_sec=45.0):
            raise TimeoutError("ComputePathToPose action server unavailable")
        node.publish_initial_pose()
        spin_until(node, lambda: node.localization_tf_count > 0, 30.0, "AMCL map->odom missing")
        wait_for_bt_active(node, 45.0)
        spin_until(node, lambda: node.latest_costmap is not None, 20.0, "global costmap missing")

        baseline_path = request_path(node, goal_x, goal_y)
        baseline_clearance = path_clearance(baseline_path, obstacle_x, 0.0)

        # 四种模型读取同一份带哈希的检测日程；输入只有位置，速度由 tracker 自己估计。
        for y in scenario["warmup"]["y_positions"]:
            node.publish_detection(obstacle_x, float(y))
            spin_for(node, float(scenario["warmup"]["interval_s"]))
        spin_until(
            node,
            lambda: node.latest_tracks is not None
            and bool(node.latest_tracks.obstacles)
            and node.latest_tracks.obstacles[0].confidence
            >= float(thresholds["minimum_track_confidence"]),
            4.0,
            "typed dynamic track missing",
        )
        track = node.latest_tracks.obstacles[0]
        predicted_x = track.position.x + track.velocity.x * prediction_horizon_s
        predicted_y = track.position.y + track.velocity.y * prediction_horizon_s
        spin_until(
            node,
            lambda: node.cost_at(predicted_x, predicted_y)
            >= int(thresholds["minimum_predicted_cost"]),
            4.0,
            "future predicted cell was not marked lethal",
        )
        predicted_cost = node.cost_at(predicted_x, predicted_y)
        dynamic_path = request_path(node, goal_x, goal_y)
        dynamic_clearance = path_clearance(dynamic_path, predicted_x, predicted_y)

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = "map"
        nav_goal.pose.header.stamp = node.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = goal_x
        nav_goal.pose.pose.position.y = goal_y
        nav_goal.pose.pose.orientation.w = 1.0
        goal_future = node.navigation.send_goal_async(nav_goal)
        spin_until(node, goal_future.done, 10.0, "NavigateToPose response timeout")
        goal_handle = goal_future.result()
        if not goal_handle.accepted:
            raise RuntimeError("NavigateToPose goal rejected")
        # 再发布少量横穿位置，使导航开始阶段必须看到障碍；随后停止发布，超时清障。
        for y in scenario["navigation"]["y_positions"]:
            node.publish_detection(obstacle_x, float(y))
            spin_for(node, float(scenario["navigation"]["interval_s"]))
        result_future = goal_handle.get_result_async()
        spin_until(node, result_future.done, args.timeout, "dynamic navigation result timeout")
        nav_status = result_future.result().status
        spin_until(
            node,
            lambda: node.last_cmd is not None
            and abs(node.last_cmd.linear.x) < 1e-3
            and abs(node.last_cmd.angular.z) < 1e-3,
            8.0,
            "cmd_vel did not return to zero",
        )
        distance = traveled_distance(node.odom_positions)
        model_behavior_ok = (
            abs(track.velocity.y) < float(thresholds["maximum_current_only_velocity_mps"])
            if args.motion_model == "current_only"
            else abs(track.velocity.y) >= float(thresholds["minimum_moving_velocity_mps"])
        )
        checks = {
            "tracker_model_behavior": model_behavior_ok,
            "future_cell_marked_lethal": predicted_cost
            >= int(thresholds["minimum_predicted_cost"]),
            "dynamic_plan_increased_clearance": dynamic_clearance
            >= baseline_clearance + float(thresholds["minimum_clearance_gain_m"]),
            "navigate_to_pose_succeeded": nav_status == GoalStatus.STATUS_SUCCEEDED,
            "robot_moved_at_least_1m": distance
            >= float(thresholds["minimum_travel_distance_m"]),
            "cmd_vel_returned_to_zero": node.last_cmd is not None
            and abs(node.last_cmd.linear.x) < 1e-3
            and abs(node.last_cmd.angular.z) < 1e-3,
        }
        report = {
            "schema_version": 2,
            "passed": all(checks.values()),
            "motion_model": args.motion_model,
            "provenance": provenance,
            "scenario": scenario,
            "elapsed_s": round(time.monotonic() - started, 3),
            "checks": checks,
            "track": {
                "id": track.track_id,
                "velocity_x_mps": round(track.velocity.x, 4),
                "velocity_y_mps": round(track.velocity.y, 4),
                "confidence": round(track.confidence, 3),
            },
            "prediction": {
                "x": round(predicted_x, 4),
                "y": round(predicted_y, 4),
                "cost": predicted_cost,
            },
            "baseline_clearance_m": round(baseline_clearance, 4),
            "dynamic_clearance_m": round(dynamic_clearance, 4),
            "published_plan_count": len(node.plans),
            "odom_traveled_distance_m": round(distance, 4),
            "navigate_to_pose_status": nav_status,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print("PASS: track -> predict -> costmap -> replan -> control" if report["passed"] else "FAIL: dynamic obstacle gate")
        return 0 if report["passed"] else 1
    except Exception as error:  # noqa: BLE001 - 重型探针必须保存失败证据
        report = {
            "schema_version": 2,
            "passed": False,
            "motion_model": args.motion_model,
            "provenance": provenance,
            "error": str(error),
            "elapsed_s": round(time.monotonic() - started, 3),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
