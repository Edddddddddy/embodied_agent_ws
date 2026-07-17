#!/usr/bin/env python3
"""验证 typed Action 与 ASR 系统意图共同驱动 SLAM 会话状态机。"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import re
import subprocess
import threading
import time
from pathlib import Path

from action_msgs.msg import GoalStatus, GoalStatusArray
from embodied_agent_interfaces.action import ExecuteRobotCommand, ManageSlamSession
from embodied_agent_interfaces.msg import (
    DynamicObstacleArray,
    RobotCommand,
    RobotCommandResult,
    SlamSessionState,
)
from embodied_agent_core.ros_qos import command_qos, event_qos, sensor_qos, state_qos
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from tests.integration.typed_action_test_utils import candidate_dict, result_dict
from tools.acceptance.dynamic_route import (
    ReplanRouteRejected,
    select_replannable_route,
)
from tools.acceptance.progress import AcceptanceProgress
from tools.acceptance.slam_nav_evidence import (
    AutomaticMissionObservation,
    AutomaticMissionThresholds,
    DynamicNavigationObservation,
    DynamicNavigationThresholds,
    build_automatic_mission_report,
    cumulative_distance as evidence_cumulative_distance,
    evaluate_dynamic_navigation,
    path_clearance as evidence_path_clearance,
)
import yaml


class SessionProbe(Node):
    def __init__(self, progress: AcceptanceProgress | None = None) -> None:
        super().__init__("voice_slam_session_probe")
        self.progress = progress
        self.states: list[SlamSessionState] = []
        self.feedback_phases: list[int] = []
        self.candidates: list[dict] = []
        self.results: list[dict] = []
        self.positions: list[tuple[float, float]] = []
        self.mapping_positions: list[tuple[float, float]] = []
        self.map_stats: dict | None = None
        self.scan_count = 0
        self.current_phase = SlamSessionState.STOPPED
        self.current_detail = ""
        self.frontier_goal_ids: set[bytes] = set()
        self.localization_tf_count = 0
        self.amcl_pose_count = 0
        self.latest_tracks: DynamicObstacleArray | None = None
        self.latest_costmap: Costmap | None = None
        self.navigation_plans: list[NavPath] = []
        self.last_cmd_vel = {"linear_x": 0.0, "angular_z": 0.0}
        self.asr_pub = self.create_publisher(
            String, "/agent/asr_final", command_qos(depth=10)
        )
        self.text_pub = self.create_publisher(
            String, "/agent/text_input", command_qos(depth=10)
        )
        self.create_subscription(
            SlamSessionState,
            "/slam/session_state",
            self._on_state,
            state_qos(),
        )
        self.create_subscription(
            RobotCommand,
            "/agent/action_candidate",
            lambda message: self.candidates.append(candidate_dict(message)),
            command_qos(depth=20),
        )
        self.create_subscription(
            GoalStatusArray,
            "/navigate_to_pose/_action/status",
            self._on_nav_status,
            event_qos(depth=20),
        )
        self.create_subscription(TFMessage, "/tf", self._on_tf, sensor_qos())
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            state_qos(),
        )
        self.create_subscription(
            DynamicObstacleArray,
            "/perception/dynamic_obstacles",
            self._on_tracks,
            event_qos(depth=20),
        )
        self.create_subscription(
            Costmap,
            "/global_costmap/costmap_raw",
            self._on_costmap,
            state_qos(),
        )
        self.create_subscription(
            NavPath,
            "/plan",
            self.navigation_plans.append,
            event_qos(depth=20),
        )
        self.detection_pub = self.create_publisher(
            PoseArray,
            "/perception/dynamic_obstacle_detections",
            sensor_qos(),
        )
        self.create_subscription(
            RobotCommandResult,
            "/robot/action_result",
            lambda message: self.results.append(result_dict(message)),
            event_qos(depth=20),
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self._on_odom,
            sensor_qos(),
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            state_qos(),
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            sensor_qos(),
        )
        self.create_subscription(
            Twist,
            "/cmd_vel",
            self._on_cmd_vel,
            command_qos(depth=20),
        )
        self.client = ActionClient(
            self, ManageSlamSession, "/slam/manage_session"
        )
        self.robot_client = ActionClient(
            self, ExecuteRobotCommand, "/robot/execute_command"
        )
        self.compute_path_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose"
        )
        self.navigation_client = ActionClient(
            self, NavigateToPose, "/navigate_to_pose"
        )
        self.lifecycle_clients = {
            name: self.create_client(GetState, f"/{name}/get_state")
            for name in (
                "planner_server",
                "controller_server",
                "bt_navigator",
                "waypoint_follower",
            )
        }

    def has_phase(self, phase: int) -> bool:
        return any(state.phase == phase for state in self.states)

    def _on_state(self, message: SlamSessionState) -> None:
        self.states.append(message)
        self.current_phase = int(message.phase)
        self.current_detail = str(message.detail)
        if self.progress is None:
            return
        # 将内部状态机压缩成操作者真正关心的验收里程碑。重复状态由
        # AcceptanceProgress 去重，避免 ROS transient-local 重投递刷屏。
        milestone = {
            SlamSessionState.STARTING_MAPPING: (1, "runtime_startup"),
            SlamSessionState.MAPPING: (1, "runtime_startup"),
            SlamSessionState.AUTOMATIC_MAPPING: (2, "frontier_slam"),
            SlamSessionState.SAVING_MAP: (3, "map_save"),
            SlamSessionState.MAP_SAVED: (3, "map_save"),
            SlamSessionState.SWITCHING_TO_NAVIGATION: (
                4,
                "localization_and_semantic_nav",
            ),
            SlamSessionState.STARTING_NAVIGATION: (
                4,
                "localization_and_semantic_nav",
            ),
            SlamSessionState.NAVIGATING: (
                4,
                "localization_and_semantic_nav",
            ),
            SlamSessionState.AUTOMATIC_NAVIGATING: (
                4,
                "localization_and_semantic_nav",
            ),
        }.get(int(message.phase))
        if milestone is not None:
            self.progress.stage(*milestone, detail=str(message.detail))

    def _on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        self.positions.append((position.x, position.y))
        if self.current_phase in {
            SlamSessionState.MAPPING,
            SlamSessionState.AUTOMATIC_MAPPING,
        }:
            self.mapping_positions.append((position.x, position.y))

    def _on_map(self, message: OccupancyGrid) -> None:
        known = sum(1 for value in message.data if value >= 0)
        occupied = sum(1 for value in message.data if value >= 65)
        self.map_stats = {
            "frame_id": message.header.frame_id,
            "width": int(message.info.width),
            "height": int(message.info.height),
            "resolution": float(message.info.resolution),
            "known_cells": known,
            "occupied_cells": occupied,
        }

    def _on_scan(self, _message: LaserScan) -> None:
        self.scan_count += 1

    def _on_cmd_vel(self, message: Twist) -> None:
        self.last_cmd_vel = {
            "linear_x": float(message.linear.x),
            "angular_z": float(message.angular.z),
        }

    def _on_nav_status(self, message: GoalStatusArray) -> None:
        if (
            self.current_phase != SlamSessionState.AUTOMATIC_MAPPING
            or "frontier exploration" not in self.current_detail
        ):
            return
        for status in message.status_list:
            if status.status in {
                GoalStatus.STATUS_ACCEPTED,
                GoalStatus.STATUS_EXECUTING,
                GoalStatus.STATUS_SUCCEEDED,
            }:
                self.frontier_goal_ids.add(bytes(status.goal_info.goal_id.uuid))

    def _on_tf(self, message: TFMessage) -> None:
        self.localization_tf_count += sum(
            transform.header.frame_id == "map"
            and transform.child_frame_id == "odom"
            for transform in message.transforms
        )

    def _on_amcl_pose(self, _message: PoseWithCovarianceStamped) -> None:
        self.amcl_pose_count += 1

    def _on_tracks(self, message: DynamicObstacleArray) -> None:
        self.latest_tracks = message

    def _on_costmap(self, message: Costmap) -> None:
        self.latest_costmap = message

    def publish_detection(self, x: float, y: float) -> None:
        message = PoseArray()
        message.header.frame_id = "map"
        pose = Pose()
        message.poses.append(pose)
        pose.position.x = x
        pose.position.y = y
        pose.orientation.w = 1.0
        self.detection_pub.publish(message)

    def publish_empty_detection(self) -> None:
        """推进 tracker TTL 并发布空轨迹，使下一候选不继承旧动态代价。"""

        message = PoseArray()
        message.header.frame_id = "map"
        self.detection_pub.publish(message)

    def cost_at(self, x: float, y: float) -> int:
        if self.latest_costmap is None:
            return -1
        metadata = self.latest_costmap.metadata
        mx = int((x - metadata.origin.position.x) / metadata.resolution)
        my = int((y - metadata.origin.position.y) / metadata.resolution)
        if mx < 0 or my < 0 or mx >= metadata.size_x or my >= metadata.size_y:
            return -1
        return int(self.latest_costmap.data[my * metadata.size_x + mx])

    def result_for(self, command_id: str) -> dict | None:
        for result in reversed(self.results):
            if result.get("command_id") == command_id:
                return result
        return None


def wait_until(predicate, timeout: float, description: str) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def robot_traveled_distance(positions: list[tuple[float, float]]) -> float:
    """累计 mapping 里程，忽略 Gazebo 重启或里程计重置产生的瞬时跳变。"""

    if not positions:
        return 0.0
    segments: list[tuple[float, float]] = [positions[0]]
    distance = 0.0
    for previous, current in zip(positions, positions[1:]):
        if math.hypot(current[0] - previous[0], current[1] - previous[1]) <= 0.5:
            segments.append(current)
            continue
        distance += evidence_cumulative_distance(segments)
        segments = [current]
    return distance + evidence_cumulative_distance(segments)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def map_artifact_sha256(map_yaml: Path) -> tuple[str, Path]:
    """把 YAML 与其引用的栅格绑定，防止同名旧图替换本次验收产物。"""

    payload = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image = Path(str(payload["image"]))
    image_path = image if image.is_absolute() else map_yaml.parent / image
    digest = hashlib.sha256()
    digest.update(map_yaml.read_bytes())
    digest.update(image_path.read_bytes())
    return digest.hexdigest(), image_path


def request_path(node: SessionProbe, goal_x: float, goal_y: float) -> NavPath:
    assert node.compute_path_client.wait_for_server(timeout_sec=20.0)
    goal = ComputePathToPose.Goal()
    goal.goal.header.frame_id = "map"
    goal.goal.header.stamp = node.get_clock().now().to_msg()
    goal.goal.pose.position.x = goal_x
    goal.goal.pose.position.y = goal_y
    goal.goal.pose.orientation.w = 1.0
    future = node.compute_path_client.send_goal_async(goal)
    wait_until(future.done, 10.0, "ComputePathToPose response timeout")
    handle = future.result()
    if not handle.accepted:
        raise RuntimeError("ComputePathToPose goal rejected")
    result_future = handle.get_result_async()
    wait_until(result_future.done, 20.0, "ComputePathToPose result timeout")
    wrapped = result_future.result()
    if (
        wrapped.status != GoalStatus.STATUS_SUCCEEDED
        or len(wrapped.result.path.poses) < 5
    ):
        raise RuntimeError(
            f"ComputePathToPose failed status={wrapped.status} "
            f"error={wrapped.result.error_code}: {wrapped.result.error_msg}"
        )
    return wrapped.result.path


def nav_path_points(
    path: NavPath, *, sampled: bool = False
) -> tuple[tuple[float, float], ...]:
    """ROS Path Adapter：纯证据模块只接收二维点，不依赖 nav_msgs。"""

    if not path.poses:
        return ()
    stride = max(1, len(path.poses) // 20) if sampled else 1
    return tuple(
        (
            float(pose.pose.position.x),
            float(pose.pose.position.y),
        )
        for pose in path.poses[::stride]
    )


def path_relative_motion_positions(
    node: SessionProbe, path: NavPath, scenario: dict
) -> tuple[list[list[float]], list[list[float]], dict[str, float]]:
    """按本次基准路径生成障碍运动，避免新地图或起点变化让固定坐标失效。"""

    motion = scenario["path_relative_motion"]
    poses = path.poses
    if len(poses) < 5:
        raise RuntimeError("baseline path is too short for a dynamic scenario")
    fraction = float(motion["path_fraction"])
    preferred_index = max(
        2, min(len(poses) - 3, int((len(poses) - 1) * fraction))
    )
    minimum_clearance = float(
        motion.get("minimum_anchor_lateral_clearance_m", 0.9)
    )
    maximum_clearance = float(
        motion.get("anchor_search_max_clearance_m", 1.5)
    )
    lethal_cost = int(motion.get("anchor_lethal_cost", 253))

    # 动态障碍必须放在存在侧向绕行空间的位置；狭窄门洞被完全封死时，
    # 规划失败是物理上无解，并不能证明 Nav2 的动态重规划能力。
    best: tuple[float, int, float, float] | None = None
    stride = max(1, len(poses) // 80)
    for candidate in range(2, len(poses) - 2, stride):
        span = max(
            1,
            min(candidate, len(poses) - 1 - candidate, len(poses) // 30),
        )
        before = poses[candidate - span].pose.position
        after = poses[candidate + span].pose.position
        norm = math.hypot(after.x - before.x, after.y - before.y)
        if norm < 1e-6:
            continue
        normal_x = -(after.y - before.y) / norm
        normal_y = (after.x - before.x) / norm
        anchor = poses[candidate].pose.position
        side_clearances = []
        for side in (-1.0, 1.0):
            clearance = 0.0
            offset = 0.3
            while offset <= maximum_clearance + 1e-6:
                cost = node.cost_at(
                    anchor.x + side * normal_x * offset,
                    anchor.y + side * normal_y * offset,
                )
                if cost < 0 or cost >= lethal_cost:
                    break
                clearance = offset
                offset += 0.2
            side_clearances.append(clearance)
        lateral_clearance = max(side_clearances)
        score = (lateral_clearance, -abs(candidate - preferred_index))
        if best is None or score > (best[0], best[1]):
            best = (lateral_clearance, -abs(candidate - preferred_index), candidate, span)

    if best is None or best[0] < minimum_clearance:
        available = 0.0 if best is None else best[0]
        raise RuntimeError(
            "baseline path has no safe dynamic-replan anchor: "
            f"lateral_clearance={available:.2f}m required={minimum_clearance:.2f}m"
        )
    lateral_clearance, _, index, span = best
    index = int(index)
    span = int(span)
    before = poses[index - span].pose.position
    after = poses[index + span].pose.position
    norm = math.hypot(after.x - before.x, after.y - before.y)
    if norm < 1e-6:
        raise RuntimeError("baseline path tangent is degenerate")
    tangent_x = (after.x - before.x) / norm
    tangent_y = (after.y - before.y) / norm
    anchor = poses[index].pose.position
    speed = float(motion["speed_mps"])
    horizon = float(scenario["prediction_horizon_s"])

    def position(seconds_before_anchor: float) -> list[float]:
        return [
            float(anchor.x) - tangent_x * speed * seconds_before_anchor,
            float(anchor.y) - tangent_y * speed * seconds_before_anchor,
        ]

    warmup = scenario["warmup"]
    warm_count = int(warmup["sample_count"])
    warm_interval = float(warmup["interval_s"])
    warm_positions = [
        position(horizon + (warm_count - 1 - index) * warm_interval)
        for index in range(warm_count)
    ]
    navigation = scenario["navigation"]
    nav_count = int(navigation["sample_count"])
    nav_interval = float(navigation["interval_s"])
    nav_positions = [
        position(max(0.0, horizon - (index + 1) * nav_interval))
        for index in range(nav_count)
    ]
    return warm_positions, nav_positions, {
        "x": float(anchor.x),
        "y": float(anchor.y),
        "tangent_x": tangent_x,
        "tangent_y": tangent_y,
        "lateral_clearance_m": lateral_clearance,
    }


def set_gazebo_entity_pose(
    *,
    world_name: str,
    entity_name: str,
    x: float,
    y: float,
    z: float = 0.4,
) -> None:
    """通过 Gazebo UserCommands 移动真实碰撞实体，失败必须阻断重型门禁。"""

    request = (
        f'name: "{entity_name}" '
        f'position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}} '
        "orientation: {w: 1.0}"
    )
    completed = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            f"/world/{world_name}/set_pose",
            "--reqtype",
            "gz.msgs.Pose",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "4000",
            "--req",
            request,
        ],
        text=True,
        capture_output=True,
        timeout=8.0,
        check=False,
    )
    output = (completed.stdout + completed.stderr).strip()
    if completed.returncode != 0 or "true" not in output.lower():
        raise RuntimeError(
            f"Gazebo set_pose failed rc={completed.returncode}: {output}"
        )


def lifecycle_states(node: SessionProbe) -> dict[str, int]:
    states: dict[str, int] = {}
    for name, client in node.lifecycle_clients.items():
        if not client.wait_for_service(timeout_sec=10.0):
            raise TimeoutError(f"{name} lifecycle service unavailable")
        future = client.call_async(GetState.Request())
        wait_until(future.done, 5.0, f"{name} lifecycle response timeout")
        states[name] = int(future.result().current_state.id)
    return states


def run_showcase_dynamic_navigation(
    node: SessionProbe,
    scenario_path: Path,
    *,
    timeout_s: float,
) -> dict:
    """在同一张新地图上验证可视障碍→typed track→costmap→Nav2 重规划。"""

    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    goals = [scenario["goal"], *scenario.get("fallback_goals", [])]
    thresholds = DynamicNavigationThresholds.from_mapping(
        scenario["thresholds"]
    )
    translation = scenario["map_to_world_translation"]
    world_name = str(scenario["world_name"])
    entity_name = str(scenario["entity_name"])
    pose_updates = 0
    last_predicted: tuple[float, float] | None = None
    route_policy = scenario.get("route_selection", {})
    retry_attempts = int(route_policy.get("dynamic_path_retry_attempts", 3))
    retry_interval_s = float(route_policy.get("retry_interval_s", 0.5))
    reset_wait_s = float(route_policy.get("reset_wait_s", 1.2))

    def publish_position(position: list[float]) -> None:
        nonlocal pose_updates
        map_x, map_y = (float(position[0]), float(position[1]))
        set_gazebo_entity_pose(
            world_name=world_name,
            entity_name=entity_name,
            x=map_x + float(translation["x"]),
            y=map_y + float(translation["y"]),
        )
        pose_updates += 1
        node.publish_detection(map_x, map_y)

    def recover_route(_goal: dict, error: ReplanRouteRejected) -> None:
        """清掉上一候选的实体、track 与 costmap，再尝试备用语义目标。"""

        nonlocal pose_updates, last_predicted
        parking = scenario["parking_world_pose"]
        set_gazebo_entity_pose(
            world_name=world_name,
            entity_name=entity_name,
            x=float(parking["x"]),
            y=float(parking["y"]),
            z=float(parking["z"]),
        )
        # tracker 只在新观测到来时执行 TTL 淘汰；等待超时后显式送空检测，
        # 让 costmap plugin 收到“当前没有障碍”的有效观测并擦除旧 bounds。
        time.sleep(reset_wait_s)
        for _ in range(3):
            node.publish_empty_detection()
            time.sleep(0.2)
        wait_until(
            lambda: node.latest_tracks is not None
            and not node.latest_tracks.obstacles,
            5.0,
            "dynamic tracker did not clear rejected route",
        )
        if last_predicted is not None:
            old_x, old_y = last_predicted
            wait_until(
                lambda: node.cost_at(old_x, old_y)
                < thresholds.minimum_predicted_cost,
                5.0,
                "dynamic costmap did not clear rejected route",
            )
        node.get_logger().warning(
            f"dynamic route rejected; trying fallback: {error}"
        )
        pose_updates = 0
        last_predicted = None

    def attempt_route(candidate_goal: dict) -> dict:
        nonlocal last_predicted
        goal_x = float(candidate_goal["x"])
        goal_y = float(candidate_goal["y"])
        try:
            baseline_path = request_path(node, goal_x, goal_y)
            motion = path_relative_motion_positions(node, baseline_path, scenario)
            warmup_positions, navigation_positions, motion_anchor = motion
            for position in warmup_positions:
                publish_position(position)
                time.sleep(float(scenario["warmup"]["interval_s"]))
            wait_until(
                lambda: node.latest_tracks is not None
                and bool(node.latest_tracks.obstacles)
                and node.latest_tracks.obstacles[0].confidence
                >= thresholds.minimum_track_confidence,
                5.0,
                "typed dynamic track missing",
            )
            track = node.latest_tracks.obstacles[0]
            horizon = float(scenario["prediction_horizon_s"])
            predicted_x = track.position.x + track.velocity.x * horizon
            predicted_y = track.position.y + track.velocity.y * horizon
            last_predicted = (predicted_x, predicted_y)
            wait_until(
                lambda: node.cost_at(predicted_x, predicted_y)
                >= thresholds.minimum_predicted_cost,
                5.0,
                "predicted dynamic cost was not marked lethal",
            )
            errors: list[str] = []
            dynamic_path = None
            for attempt_index in range(1, retry_attempts + 1):
                try:
                    candidate_dynamic_path = request_path(node, goal_x, goal_y)
                except RuntimeError as path_error:
                    errors.append(f"attempt {attempt_index}: {path_error}")
                else:
                    baseline_clearance = evidence_path_clearance(
                        nav_path_points(baseline_path), predicted_x, predicted_y
                    )
                    candidate_clearance = evidence_path_clearance(
                        nav_path_points(candidate_dynamic_path),
                        predicted_x,
                        predicted_y,
                    )
                    minimum_gain = thresholds.minimum_clearance_gain_m
                    if candidate_clearance >= baseline_clearance + minimum_gain:
                        dynamic_path = candidate_dynamic_path
                        break
                    errors.append(
                        f"attempt {attempt_index}: dynamic path clearance "
                        f"{candidate_clearance:.3f}m did not exceed baseline "
                        f"{baseline_clearance:.3f}m by {minimum_gain:.3f}m"
                    )
                if attempt_index < retry_attempts:
                    time.sleep(retry_interval_s)
            if dynamic_path is None:
                raise ReplanRouteRejected("; ".join(errors))
        except ReplanRouteRejected:
            raise
        except (AssertionError, RuntimeError, TimeoutError) as route_error:
            raise ReplanRouteRejected(str(route_error)) from route_error
        return {
            "goal": candidate_goal,
            "goal_x": goal_x,
            "goal_y": goal_y,
            "baseline_path": baseline_path,
            "dynamic_path": dynamic_path,
            "warmup_positions": warmup_positions,
            "navigation_positions": navigation_positions,
            "motion_anchor": motion_anchor,
            "track": track,
            "predicted_x": predicted_x,
            "predicted_y": predicted_y,
            "predicted_cost": node.cost_at(predicted_x, predicted_y),
        }

    selected, route_attempts = select_replannable_route(
        goals,
        attempt=attempt_route,
        recover=recover_route,
    )
    goal = selected["goal"]
    goal_x = selected["goal_x"]
    goal_y = selected["goal_y"]
    baseline_path = selected["baseline_path"]
    dynamic_path = selected["dynamic_path"]
    warmup_positions = selected["warmup_positions"]
    navigation_positions = selected["navigation_positions"]
    motion_anchor = selected["motion_anchor"]
    track = selected["track"]
    predicted_x = selected["predicted_x"]
    predicted_y = selected["predicted_y"]
    predicted_cost = selected["predicted_cost"]

    assert node.navigation_client.wait_for_server(timeout_sec=20.0)
    plan_start = len(node.navigation_plans)
    odom_start = len(node.positions)
    nav_goal = NavigateToPose.Goal()
    nav_goal.pose.header.frame_id = "map"
    nav_goal.pose.header.stamp = node.get_clock().now().to_msg()
    nav_goal.pose.pose.position.x = goal_x
    nav_goal.pose.pose.position.y = goal_y
    nav_goal.pose.pose.orientation.w = 1.0
    goal_future = node.navigation_client.send_goal_async(nav_goal)
    wait_until(goal_future.done, 10.0, "dynamic NavigateToPose response timeout")
    handle = goal_future.result()
    if not handle.accepted:
        raise RuntimeError("dynamic NavigateToPose goal rejected")
    result_future = handle.get_result_async()
    for position in navigation_positions:
        publish_position(position)
        time.sleep(float(scenario["navigation"]["interval_s"]))
    time.sleep(float(scenario["navigation"].get("hold_s", 0.0)))
    parking = scenario["parking_world_pose"]
    set_gazebo_entity_pose(
        world_name=world_name,
        entity_name=entity_name,
        x=float(parking["x"]),
        y=float(parking["y"]),
        z=float(parking["z"]),
    )
    pose_updates += 1
    wait_until(result_future.done, timeout_s, "dynamic NavigateToPose result timeout")
    nav_status = int(result_future.result().status)
    wait_until(
        lambda: abs(node.last_cmd_vel["linear_x"]) < 1e-3
        and abs(node.last_cmd_vel["angular_z"]) < 1e-3,
        8.0,
        "dynamic navigation did not finish with zero velocity",
    )

    positions = node.positions[odom_start:]
    traveled = robot_traveled_distance(positions)
    plans = node.navigation_plans[plan_start:]
    # Adapter 到此只负责把 ROS 消息翻译成普通值；全部 PASS 条件和报告格式
    # 由纯证据模块统一拥有，其他验收入口不能再复制一套阈值判断。
    observation = DynamicNavigationObservation(
        scenario_id=str(scenario["scenario_id"]),
        goal=goal,
        route_attempts=tuple(route_attempts),
        motion_anchor=motion_anchor,
        gazebo_pose_updates=pose_updates,
        expected_pose_updates=(
            len(warmup_positions) + len(navigation_positions) + 1
        ),
        track_id=str(track.track_id),
        track_confidence=float(track.confidence),
        track_velocity_x_mps=float(track.velocity.x),
        track_velocity_y_mps=float(track.velocity.y),
        predicted_x=float(predicted_x),
        predicted_y=float(predicted_y),
        predicted_cost=int(predicted_cost),
        baseline_path=nav_path_points(baseline_path),
        dynamic_path=nav_path_points(dynamic_path),
        published_plans=tuple(
            nav_path_points(path, sampled=True) for path in plans if path.poses
        ),
        odom_traveled_distance_m=traveled,
        navigate_to_pose_status=nav_status,
        navigation_succeeded=nav_status == GoalStatus.STATUS_SUCCEEDED,
        final_linear_x=float(node.last_cmd_vel["linear_x"]),
        final_angular_z=float(node.last_cmd_vel["angular_z"]),
    )
    return evaluate_dynamic_navigation(observation, thresholds)


def run_text_action(
    node: SessionProbe,
    *,
    text: str,
    expected_action: str,
    timeout: float,
) -> dict:
    """通过 Agent 文本入口执行一步；自动门禁只替换声学 ASR，不绕过控制链。"""

    candidate_start = len(node.candidates)
    node.text_pub.publish(String(data=text))
    wait_until(
        lambda: any(
            item.get("name") == expected_action
            for item in node.candidates[candidate_start:]
        ),
        15.0,
        f"no {expected_action} candidate for {text!r}",
    )
    candidate = next(
        item
        for item in node.candidates[candidate_start:]
        if item.get("name") == expected_action
    )
    command_id = str(candidate.get("request_id") or "")
    if not command_id:
        raise RuntimeError(f"candidate has no command id: {candidate}")
    wait_until(
        lambda: node.result_for(command_id) is not None,
        timeout,
        f"action result timeout for {text!r}",
    )
    result = node.result_for(command_id)
    if result is None or result.get("success") is not True:
        raise RuntimeError(f"action failed for {text!r}: {result}")
    return {"text": text, "candidate": candidate, "result": result}


def load_survey_plan(path: Path) -> tuple[list[dict], dict]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    route = payload.get("mapping_route") if isinstance(payload, dict) else None
    acceptance = payload.get("acceptance") if isinstance(payload, dict) else None
    if not isinstance(route, list) or not route:
        raise ValueError("survey plan requires a non-empty mapping_route")
    if not isinstance(acceptance, dict):
        raise ValueError("survey plan requires acceptance thresholds")
    return route, acceptance


def print_report(report: dict, *, summary_only: bool) -> None:
    """终端默认只展示决策所需字段；完整证据始终写入 JSON 文件。"""

    if not summary_only:
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        return
    summary = {
        "passed": report.get("passed"),
        "session_id": report.get("session_id"),
        "final_phase": report.get("final_phase"),
        "mapping_path_m": report.get("mapping_path_m"),
        "frontier_goal_count": report.get("frontier_goal_count"),
        "checks": report.get("checks"),
        "error": report.get("error"),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--transition-timeout", type=float, default=10.0)
    parser.add_argument("--evidence-kind", default="dry_run_process_adapter")
    parser.add_argument("--session-id", default="")
    parser.add_argument("--session-start-ns", type=int, default=0)
    parser.add_argument("--world-file", type=Path)
    parser.add_argument("--mission-plan", type=Path)
    parser.add_argument("--dynamic-scenario", type=Path)
    parser.add_argument("--dynamic-navigation-timeout", type=float, default=180.0)
    parser.add_argument("--runtime-log", type=Path)
    parser.add_argument("--gate-timeout-s", type=float, default=900.0)
    parser.add_argument("--progress-heartbeat-s", type=float, default=15.0)
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="终端打印验收摘要，完整字段仅写入 --output",
    )
    parser.add_argument("--explore-before-save", action="store_true")
    parser.add_argument(
        "--automatic-mission",
        action="store_true",
        help="一句系统意图触发自动探索、存图、定位切换和语义导航",
    )
    parser.add_argument(
        "--cancel-automatic-mission",
        action="store_true",
        help="在 dry-run 自动探索期间发送急停并验证恢复到 MAPPING",
    )
    parser.add_argument(
        "--survey-plan",
        type=Path,
        help="通过 Agent 顺序执行工作场景 mapping_route，并验证地图覆盖与里程",
    )
    args = parser.parse_args()
    progress = (
        AcceptanceProgress(
            label="slam-nav-e2e",
            total_stages=6,
            heartbeat_s=args.progress_heartbeat_s,
        )
        if args.automatic_mission
        else None
    )
    progress_outcome = "FAIL"
    rclpy.init()
    node = SessionProbe(progress=progress)
    if progress is not None:
        progress.start(
            session_id=args.session_id or "unassigned",
            timeout_s=args.gate_timeout_s,
            log_path=str(args.runtime_log or args.output),
            detail_supplier=lambda: (
                f"phase={node.current_phase} "
                f"detail={node.current_detail or '-'}"
            ),
        )
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: node.has_phase(SlamSessionState.MAPPING),
            args.transition_timeout,
            "orchestrator did not enter MAPPING",
        )
        wait_until(
            lambda: node.asr_pub.get_subscription_count() > 0,
            5.0,
            "ASR final subscriber unavailable",
        )

        if args.automatic_mission or args.cancel_automatic_mission:
            state_start = len(node.states)
            node.asr_pub.publish(String(data="开始自动巡检建图"))
            if args.cancel_automatic_mission:
                wait_until(
                    lambda: any(
                        state.phase == SlamSessionState.AUTOMATIC_MAPPING
                        for state in node.states[state_start:]
                    ),
                    args.transition_timeout,
                    "automatic mission did not enter AUTOMATIC_MAPPING",
                )
                node.asr_pub.publish(String(data="急停"))
                wait_until(
                    lambda: any(
                        state.phase == SlamSessionState.MAPPING
                        and "canceled" in state.detail
                        for state in node.states[state_start:]
                    ),
                    args.transition_timeout,
                    "urgent stop did not recover automatic mission to MAPPING",
                )
                assert not node.has_phase(SlamSessionState.MISSION_COMPLETED)
                report = {
                    "passed": True,
                    "state_sequence": [int(state.phase) for state in node.states],
                    "automatic_mission_canceled": True,
                    "final_phase": SlamSessionState.MAPPING,
                    "final_cmd_vel": node.last_cmd_vel,
                }
                args.output.parent.mkdir(parents=True, exist_ok=True)
                args.output.write_text(
                    json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                    encoding="utf-8",
                )
                print(json.dumps(report, ensure_ascii=False, indent=2))
                return
            wait_until(
                lambda: (
                    node.has_phase(SlamSessionState.MISSION_COMPLETED)
                    or any(
                        state.phase == SlamSessionState.FAILED
                        or (
                            state.phase == SlamSessionState.MAPPING
                            and "automatic mission failed" in state.detail
                        )
                        for state in node.states[state_start:]
                    )
                ),
                args.transition_timeout,
                "automatic mission did not reach a terminal state",
            )
            failed_state = next(
                (
                    state
                    for state in reversed(node.states[state_start:])
                    if state.phase == SlamSessionState.FAILED
                    or (
                        state.phase == SlamSessionState.MAPPING
                        and "automatic mission failed" in state.detail
                    )
                ),
                None,
            )
            if failed_state is not None:
                # 重型进程已经给出确定失败时立即结束，避免继续等待完整超时，
                # 同时让 smoke 脚本及时打印 orchestrator/Nav2 原始日志。
                raise RuntimeError(failed_state.detail)
            observed = [int(state.phase) for state in node.states]
            # 状态 topic 使用 depth=1 + transient-local：dry-run 阶段切换仅数毫秒，
            # 订阅者可能合理地只收到最新快照。最终状态同时携带 map_saved，故可作为
            # 一条语音已经跨越探索、保存和导航事务边界的稳定验收契约。
            final_state = next(
                state
                for state in reversed(node.states)
                if state.phase == SlamSessionState.MISSION_COMPLETED
            )
            lifecycle = {}
            dynamic_navigation = None
            if args.evidence_kind != "dry_run_process_adapter":
                wait_until(
                    lambda: (
                        abs(node.last_cmd_vel["linear_x"]) < 1e-6
                        and abs(node.last_cmd_vel["angular_z"]) < 1e-6
                    ),
                    5.0,
                    "automatic mission finished without a final zero velocity",
                )
                wait_until(
                    lambda: node.localization_tf_count > 0 and node.amcl_pose_count > 0,
                    20.0,
                    "saved-map localization evidence missing",
                )
                lifecycle = lifecycle_states(node)
                if args.dynamic_scenario is not None:
                    if progress is not None:
                        progress.stage(
                            5,
                            "dynamic_obstacle_replan",
                            "semantic patrol complete; injecting moving obstacle",
                        )
                    dynamic_navigation = run_showcase_dynamic_navigation(
                        node,
                        args.dynamic_scenario,
                        timeout_s=args.dynamic_navigation_timeout,
                    )

            if progress is not None:
                progress.stage(
                    6,
                    "evidence_validation",
                    "validating fresh map, localization, plans and final stop",
                )

            map_yaml = Path(final_state.map_yaml_path) if final_state.map_yaml_path else None
            map_provenance = None
            mission_thresholds = AutomaticMissionThresholds(0.0, 0, 0)
            if map_yaml is not None and map_yaml.is_file():
                artifact_hash, image_path = map_artifact_sha256(map_yaml)
                map_provenance = {
                    "yaml_path": str(map_yaml.resolve()),
                    "image_path": str(image_path.resolve()),
                    "yaml_sha256": sha256_file(map_yaml),
                    "image_sha256": sha256_file(image_path),
                    "artifact_sha256": artifact_hash,
                    "yaml_mtime_ns": map_yaml.stat().st_mtime_ns,
                    "image_mtime_ns": image_path.stat().st_mtime_ns,
                }
            if args.mission_plan is not None:
                mission = yaml.safe_load(
                    args.mission_plan.read_text(encoding="utf-8")
                )
                mission_thresholds = AutomaticMissionThresholds.from_mapping(
                    mission.get("acceptance", {})
                )

            mapping_distance_m = robot_traveled_distance(node.mapping_positions)
            completion_detail = next(
                (
                    state.detail
                    for state in reversed(node.states)
                    if "reason=" in state.detail
                ),
                "",
            )
            reason_match = re.search(r"reason=([^ ]+)", completion_detail)
            completion_reason = reason_match.group(1) if reason_match else ""
            lifecycle_active = len(lifecycle) == 4 and all(
                state == int(State.PRIMARY_STATE_ACTIVE)
                for state in lifecycle.values()
            )
            observation = AutomaticMissionObservation(
                session_id=args.session_id,
                session_start_ns=args.session_start_ns,
                state_sequence=tuple(observed),
                map_saved=bool(final_state.map_saved),
                map_yaml_path=final_state.map_yaml_path,
                final_phase=int(final_state.phase),
                evidence_kind=args.evidence_kind,
                action_candidates=tuple(node.candidates),
                action_results=tuple(node.results),
                map_stats=node.map_stats,
                map_provenance=map_provenance,
                mapping_path_m=mapping_distance_m,
                frontier_goal_count=len(node.frontier_goal_ids),
                exploration_completion_reason=completion_reason,
                localization_tf_count=node.localization_tf_count,
                amcl_pose_count=node.amcl_pose_count,
                lifecycle_states=lifecycle,
                nav2_lifecycle_active=lifecycle_active,
                dynamic_navigation=dynamic_navigation,
                dynamic_navigation_required=args.dynamic_scenario is not None,
                provenance={
                    "world_sha256": sha256_file(args.world_file)
                    if args.world_file
                    else "",
                    "mission_sha256": sha256_file(args.mission_plan)
                    if args.mission_plan
                    else "",
                    "dynamic_scenario_sha256": sha256_file(args.dynamic_scenario)
                    if args.dynamic_scenario
                    else "",
                },
                final_linear_x=float(node.last_cmd_vel["linear_x"]),
                final_angular_z=float(node.last_cmd_vel["angular_z"]),
            )
            report = build_automatic_mission_report(
                observation, mission_thresholds
            )
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(
                json.dumps(report, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            print_report(report, summary_only=args.summary_only)
            if not report["passed"]:
                raise SystemExit(1)
            progress_outcome = "PASS"
            return

        exploration_succeeded = None
        survey_steps: list[dict] = []
        survey_distance_m = 0.0
        survey_map: dict | None = None
        if args.survey_plan:
            route, thresholds = load_survey_plan(args.survey_plan)
            wait_until(
                lambda: (
                    node.text_pub.get_subscription_count() > 0
                    and node.positions
                    and node.map_stats is not None
                    and node.scan_count >= 3
                ),
                30.0,
                "Agent/odom/map/scan inputs unavailable for mapping survey",
            )
            odom_start = len(node.positions)
            for index, step in enumerate(route, start=1):
                print(
                    f"[mapping {index}/{len(route)}] {step['label']}: {step['text']}",
                    flush=True,
                )
                survey_steps.append(
                    run_text_action(
                        node,
                        text=str(step["text"]),
                        expected_action=str(step["action"]),
                        timeout=25.0,
                    )
                )
            wait_until(
                lambda: node.map_stats is not None
                and node.map_stats["known_cells"]
                >= int(thresholds["min_known_map_cells"]),
                15.0,
                "SLAM map did not reach the required known-cell coverage",
            )
            survey_distance_m = robot_traveled_distance(
                node.positions[odom_start:]
            )
            survey_map = dict(node.map_stats or {})
            assert survey_distance_m >= float(thresholds["min_mapping_path_m"]), (
                survey_distance_m,
                thresholds,
            )
            assert survey_map["occupied_cells"] >= int(
                thresholds["min_occupied_map_cells"]
            )
            exploration_succeeded = True
        elif args.explore_before_save:
            assert node.robot_client.wait_for_server(timeout_sec=10.0)
            explore_goal = ExecuteRobotCommand.Goal()
            explore_goal.command.action_type = RobotCommand.MOVE
            explore_goal.command.linear_x = 0.18
            explore_goal.command.duration_s = 2.0
            explore_goal.command.command_id = "showcase-session-exploration"
            explore_goal.command.source = "slam_session_acceptance"
            explore_future = node.robot_client.send_goal_async(explore_goal)
            wait_until(explore_future.done, 10.0, "exploration goal response missing")
            explore_handle = explore_future.result()
            assert explore_handle.accepted
            explore_result_future = explore_handle.get_result_async()
            wait_until(
                explore_result_future.done,
                30.0,
                "mapping exploration action did not finish",
            )
            exploration_succeeded = bool(
                explore_result_future.result().result.success
            )
            assert exploration_succeeded

        # 语音系统意图负责保存地图；dry-run Adapter 不写伪造文件，但状态契约相同。
        node.asr_pub.publish(String(data="保存地图"))
        wait_until(
            lambda: node.has_phase(SlamSessionState.MAP_SAVED),
            args.transition_timeout,
            "voice save-map command did not reach MAP_SAVED",
        )

        assert node.client.wait_for_server(timeout_sec=5.0)
        goal = ManageSlamSession.Goal()
        goal.command = ManageSlamSession.Goal.START_NAVIGATION

        def on_feedback(message) -> None:
            node.feedback_phases.append(message.feedback.state.phase)

        goal_future = node.client.send_goal_async(goal, feedback_callback=on_feedback)
        wait_until(goal_future.done, 5.0, "session Action goal response missing")
        goal_handle = goal_future.result()
        assert goal_handle.accepted
        result_future = goal_handle.get_result_async()
        wait_until(
            result_future.done,
            args.transition_timeout,
            "session Action result missing",
        )
        result = result_future.result().result
        assert result.success, result.message
        assert result.state.phase == SlamSessionState.NAVIGATING
        assert result.state.map_saved
        observed_phases = {
            *(state.phase for state in node.states),
            *node.feedback_phases,
        }
        assert SlamSessionState.SWITCHING_TO_NAVIGATION in observed_phases
        assert SlamSessionState.STARTING_NAVIGATION in observed_phases

        # ASR 抖动造成的重复“开始导航”是幂等操作，不能把会话打入 FAILED。
        node.asr_pub.publish(String(data="开始导航"))
        node.asr_pub.publish(String(data="开始导航"))
        time.sleep(0.3)
        assert not node.has_phase(SlamSessionState.FAILED)

        report = {
            "passed": True,
            "state_sequence": [int(state.phase) for state in node.states],
            "feedback_phases": node.feedback_phases,
            "map_saved": bool(result.state.map_saved),
            "map_yaml_path": result.state.map_yaml_path,
            "final_phase": int(result.state.phase),
            "evidence_kind": args.evidence_kind,
            "exploration_succeeded": exploration_succeeded,
            "survey_step_count": len(survey_steps),
            "survey_distance_m": round(survey_distance_m, 3),
            "survey_map": survey_map,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
    except Exception as error:
        # 重型门禁即使在中途失败也必须留下机器可读原因，避免现场只看到超时或卡住。
        failure_report = {
            "schema_version": 3,
            "passed": False,
            "session_id": args.session_id,
            "session_start_ns": args.session_start_ns,
            "evidence_kind": args.evidence_kind,
            "error": str(error),
            "state_sequence": [int(state.phase) for state in node.states],
            "last_state_detail": node.current_detail,
            "map": node.map_stats,
            "frontier_goal_count": len(node.frontier_goal_ids),
            "mapping_path_m": round(
                robot_traveled_distance(node.mapping_positions), 3
            ),
            "final_cmd_vel": node.last_cmd_vel,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(
            json.dumps(failure_report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print_report(failure_report, summary_only=args.summary_only)
        raise
    finally:
        if progress is not None:
            progress.stop(progress_outcome)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
