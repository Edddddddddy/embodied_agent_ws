"""Gazebo 动态障碍与 Nav2 重规划场景运行器。"""

from __future__ import annotations

import json
import math
import subprocess
import time
from pathlib import Path

from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path as NavPath

from tools.acceptance.dynamic_route import ReplanRouteRejected, select_replannable_route
from tools.acceptance.probes.slam_nav.artifacts import robot_traveled_distance
from tools.acceptance.probes.slam_nav.session_observer import (
    SessionObserver,
    nav_path_points,
    request_path,
    wait_until,
)
from tools.acceptance.slam_nav_evidence import (
    DynamicNavigationObservation,
    DynamicNavigationThresholds,
    evaluate_dynamic_navigation,
    path_clearance,
)


def path_relative_motion_positions(
    node: SessionObserver, path: NavPath, scenario: dict
) -> tuple[list[list[float]], list[list[float]], dict[str, float]]:
    """按本次规划路径生成障碍运动，避免地图或起点变化使固定坐标失效。"""

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
    maximum_clearance = float(motion.get("anchor_search_max_clearance_m", 1.5))
    lethal_cost = int(motion.get("anchor_lethal_cost", 253))

    # 只有路径两侧存在绕行空间，动态阻塞才是在验证重规划而非制造无解门洞。
    best: tuple[float, int, int, int] | None = None
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
            best = (
                lateral_clearance,
                -abs(candidate - preferred_index),
                candidate,
                span,
            )

    if best is None or best[0] < minimum_clearance:
        available = 0.0 if best is None else best[0]
        raise RuntimeError(
            "baseline path has no safe dynamic-replan anchor: "
            f"lateral_clearance={available:.2f}m required={minimum_clearance:.2f}m"
        )
    lateral_clearance, _, index, span = best
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
        f"position: {{x: {x:.4f}, y: {y:.4f}, z: {z:.4f}}} "
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


def run_showcase_dynamic_navigation(
    node: SessionObserver,
    scenario_path: Path,
    *,
    timeout_s: float,
) -> dict:
    """验证可视实体→typed track→costmap→Nav2 重规划的完整事务。"""

    scenario = json.loads(scenario_path.read_text(encoding="utf-8"))
    goals = [scenario["goal"], *scenario.get("fallback_goals", [])]
    thresholds = DynamicNavigationThresholds.from_mapping(scenario["thresholds"])
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
        map_x, map_y = float(position[0]), float(position[1])
        set_gazebo_entity_pose(
            world_name=world_name,
            entity_name=entity_name,
            x=map_x + float(translation["x"]),
            y=map_y + float(translation["y"]),
        )
        pose_updates += 1
        node.publish_detection(map_x, map_y)

    def recover_route(_goal: dict, error: ReplanRouteRejected) -> None:
        """清理上一候选的实体、track 与 costmap，再尝试备用语义目标。"""

        nonlocal pose_updates, last_predicted
        parking = scenario["parking_world_pose"]
        set_gazebo_entity_pose(
            world_name=world_name,
            entity_name=entity_name,
            x=float(parking["x"]),
            y=float(parking["y"]),
            z=float(parking["z"]),
        )
        # tracker 仅在收到新观测时清理 TTL；显式空检测让 costmap 擦除旧 bounds。
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
        node.get_logger().warning(f"dynamic route rejected; trying fallback: {error}")
        pose_updates = 0
        last_predicted = None

    def attempt_route(candidate_goal: dict) -> dict:
        nonlocal last_predicted
        goal_x = float(candidate_goal["x"])
        goal_y = float(candidate_goal["y"])
        try:
            baseline_path = request_path(node, goal_x, goal_y)
            warmup_positions, navigation_positions, motion_anchor = (
                path_relative_motion_positions(node, baseline_path, scenario)
            )
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
                    candidate_path = request_path(node, goal_x, goal_y)
                except RuntimeError as path_error:
                    errors.append(f"attempt {attempt_index}: {path_error}")
                else:
                    baseline_clearance = path_clearance(
                        nav_path_points(baseline_path), predicted_x, predicted_y
                    )
                    candidate_clearance = path_clearance(
                        nav_path_points(candidate_path), predicted_x, predicted_y
                    )
                    minimum_gain = thresholds.minimum_clearance_gain_m
                    if candidate_clearance >= baseline_clearance + minimum_gain:
                        dynamic_path = candidate_path
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
        except (RuntimeError, TimeoutError) as route_error:
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
        goals, attempt=attempt_route, recover=recover_route
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

    if not node.navigation_client.wait_for_server(timeout_sec=20.0):
        raise TimeoutError("NavigateToPose action server unavailable")
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
    plans = node.navigation_plans[plan_start:]
    # 运行层只生成观测值；所有阈值与 PASS 结论仍由纯证据模块统一拥有。
    observation = DynamicNavigationObservation(
        scenario_id=str(scenario["scenario_id"]),
        goal=goal,
        route_attempts=tuple(route_attempts),
        motion_anchor=motion_anchor,
        gazebo_pose_updates=pose_updates,
        expected_pose_updates=len(warmup_positions) + len(navigation_positions) + 1,
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
        odom_traveled_distance_m=robot_traveled_distance(positions),
        navigate_to_pose_status=nav_status,
        navigation_succeeded=nav_status == GoalStatus.STATUS_SUCCEEDED,
        final_linear_x=float(node.last_cmd_vel["linear_x"]),
        final_angular_z=float(node.last_cmd_vel["angular_z"]),
    )
    return evaluate_dynamic_navigation(observation, thresholds)
