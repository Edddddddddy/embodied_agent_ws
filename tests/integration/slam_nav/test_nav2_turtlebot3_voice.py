#!/usr/bin/env python3
"""Drive the real Nav2/TurtleBot3 voice launch with text-input commands.

该测试面向人工/重型验收：外层脚本会启动官方 Nav2 TurtleBot3 仿真，
这里只模拟 ASR final，把“去门口”“依次去门口、书桌、起点”注入 Agent。
通过标准不是简单看到 candidate，而是等待 Nav2 executor 返回 action result，
并观察 /odom 有运动，证明语音语义已经进入真实 Nav2 控制链路。
"""

import argparse
import json
import math
import subprocess
import threading
import time
from pathlib import Path

import rclpy
from embodied_agent_interfaces.msg import RobotCommand, RobotCommandResult
from embodied_agent_core.ros_qos import command_qos, event_qos, sensor_qos, state_qos
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tools.acceptance.typed_action_probe_utils import candidate_dict, result_dict


class Nav2TurtleBot3VoiceProbe(Node):
    def __init__(self):
        super().__init__("nav2_turtlebot3_voice_probe")
        self.text_pub = self.create_publisher(
            String, "/agent/text_input", command_qos(depth=10)
        )
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", command_qos(depth=10)
        )
        self.candidates = []
        self.results = []
        self.positions = []
        self.map_metadata = None
        self.scan_count = 0
        self.latest_scan_min = math.inf
        self.paths = []
        self.velocities = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.bt_state_client = self.create_client(GetState, "/bt_navigator/get_state")
        self.waypoint_state_client = self.create_client(
            GetState, "/waypoint_follower/get_state"
        )
        self.create_subscription(
            RobotCommand,
            "/agent/action_candidate",
            self._on_candidate,
            command_qos(depth=10),
        )
        self.create_subscription(
            RobotCommandResult,
            "/robot/action_result",
            self._on_result,
            event_qos(depth=10),
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, sensor_qos())
        self.create_subscription(
            NavPath, "/plan", self._on_path, event_qos(depth=10)
        )
        self.create_subscription(
            Twist, "/cmd_vel", self._on_velocity, command_qos(depth=10)
        )
        self.create_subscription(OccupancyGrid, "/map", self._on_map, state_qos())
        self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos())

    def _on_candidate(self, message):
        self.candidates.append(candidate_dict(message))

    def _on_result(self, message):
        self.results.append(result_dict(message))

    def _on_odom(self, message):
        position = message.pose.pose.position
        self.positions.append((position.x, position.y))

    def _on_map(self, message):
        self.map_metadata = {
            "frame_id": message.header.frame_id,
            "width": message.info.width,
            "height": message.info.height,
            "resolution": message.info.resolution,
        }

    def _on_scan(self, message):
        self.scan_count += 1
        valid = [value for value in message.ranges if math.isfinite(value)]
        if valid:
            self.latest_scan_min = min(valid)

    def _on_path(self, message):
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in message.poses]
        if points:
            self.paths.append((time.monotonic(), points))

    def _on_velocity(self, message):
        self.velocities.append((message.linear.x, message.angular.z))

    def localized_base_frame(self):
        for base_frame in ("base_link", "base_footprint"):
            if self.tf_buffer.can_transform("map", base_frame, Time()):
                return base_frame
        return None


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.1)
    raise TimeoutError(description)


def latest_candidate(node, name):
    for candidate in reversed(node.candidates):
        if candidate.get("name") == name:
            return candidate
    return None


def has_success_result(node, command_id):
    return any(
        result.get("command_id") == command_id and result.get("success") is True
        for result in node.results
    )


def traveled_distance(positions):
    if len(positions) < 2:
        return 0.0
    start = positions[0]
    return max(math.hypot(x - start[0], y - start[1]) for x, y in positions)


def quaternion_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def publish_initial_pose(node, x, y, yaw):
    """Seed AMCL before sending Nav2 goals.

    官方 TurtleBot3/Nav2 bringup 不一定自动给 AMCL 初始位姿；没有 map->odom TF 时
    Nav2 goal 会一直卡在 costmap/transform 等待。这里用 launch 默认出生点给 AMCL
    一个近似初值，足够支撑演示级目标点导航验收。
    """

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
    for _ in range(10):
        message.header.stamp = node.get_clock().now().to_msg()
        node.initial_pose_pub.publish(message)
        time.sleep(0.2)


def wait_for_lifecycle_active(client, timeout, node_name):
    """等待 Nav2 Lifecycle 真正 ACTIVE，避免 action server 尚未激活就发 goal。"""

    if not client.wait_for_service(timeout_sec=timeout):
        raise TimeoutError(f"{node_name} lifecycle service unavailable")
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        future = client.call_async(GetState.Request())
        wait_until(future.done, min(2.0, max(0.1, deadline - time.monotonic())), node_name)
        if future.result().current_state.id == State.PRIMARY_STATE_ACTIVE:
            return
        time.sleep(0.2)
    raise TimeoutError(f"{node_name} did not reach ACTIVE")


def run_command(node, text, candidate_name, timeout):
    before = len(node.results)
    node.text_pub.publish(String(data=text))
    wait_until(
        lambda: latest_candidate(node, candidate_name) is not None,
        20.0,
        f"{candidate_name} candidate was not published for {text!r}",
    )
    candidate = latest_candidate(node, candidate_name)
    command_id = candidate["request_id"]
    wait_until(
        lambda: len(node.results) > before and has_success_result(node, command_id),
        timeout,
        f"{candidate_name} did not finish through Nav2 for {text!r}",
    )
    return candidate


def result_for(node, command_id):
    for result in reversed(node.results):
        if result.get("command_id") == command_id:
            return result
    return None


def path_clearance(path, obstacle):
    return min(math.hypot(x - obstacle[0], y - obstacle[1]) for x, y in path)


def point_ahead_on_path(path, current, target, minimum_distance=0.65):
    """从当前全局路径上选一个前方点，确保障碍确实插在原计划路径上。"""

    dx = target[0] - current[0]
    dy = target[1] - current[1]
    length = math.hypot(dx, dy) or 1.0
    ux, uy = dx / length, dy / length
    candidates = []
    for point in path:
        projection = (point[0] - current[0]) * ux + (point[1] - current[1]) * uy
        if projection >= minimum_distance:
            candidates.append((projection, point))
    if not candidates:
        raise RuntimeError("global path contains no point ahead of the robot")
    return min(candidates, key=lambda item: item[0])[1]


def offset_from_path(point, current, target, offset=0.3):
    dx = target[0] - current[0]
    dy = target[1] - current[1]
    length = math.hypot(dx, dy) or 1.0
    # 沿原路径法线偏置：障碍进入 inflation 区但不把小地图通道完全封死。
    return point[0] - dy / length * offset, point[1] + dx / length * offset


def obstacle_sdf(name):
    return f"""
<sdf version='1.7'>
  <model name='{name}'>
    <static>true</static>
    <link name='link'>
      <collision name='collision'>
        <geometry><box><size>0.20 0.20 0.8</size></box></geometry>
      </collision>
      <visual name='visual'>
        <geometry><box><size>0.20 0.20 0.8</size></box></geometry>
        <material><ambient>0.9 0.15 0.15 1</ambient><diffuse>0.9 0.15 0.15 1</diffuse></material>
      </visual>
    </link>
  </model>
</sdf>
""".strip()


def spawn_obstacle(node, name, position):
    # Gazebo Sim 的 world/create 属于 Gazebo Transport，并不自动成为 ROS service。
    # 使用 ros_gz_sim 官方 create 工具，与 Nav2 官方 tb3_simulation_launch.py 保持一致。
    command = [
        "/opt/ros/jazzy/lib/ros_gz_sim/create",
        "-world",
        "voice_nav2_demo",
        "-string",
        obstacle_sdf(name),
        "-name",
        name,
        "-x",
        str(position[0]),
        "-y",
        str(position[1]),
        "-z",
        "0.4",
    ]
    completed = subprocess.run(
        command, capture_output=True, text=True, timeout=25.0, check=False
    )
    if completed.returncode != 0:
        raise RuntimeError(
            "Gazebo rejected dynamic obstacle: "
            + (completed.stderr.strip() or completed.stdout.strip())
        )


def run_dynamic_obstacle_navigation(node, timeout):
    """导航途中把方块插入当前全局路径，并要求 Nav2 重新规划后到达。"""

    baseline_position_index = len(node.positions)
    baseline_path_index = len(node.paths)
    node.text_pub.publish(String(data="去门口"))
    wait_until(
        lambda: latest_candidate(node, "navigate_to") is not None,
        20.0,
        "navigate_to candidate missing for dynamic obstacle scenario",
    )
    candidate = latest_candidate(node, "navigate_to")
    command_id = candidate["request_id"]
    wait_until(
        lambda: len(node.paths) > baseline_path_index and len(node.positions) > baseline_position_index,
        30.0,
        "Nav2 did not publish an initial global path",
    )
    wait_until(
        lambda: traveled_distance(node.positions[baseline_position_index:]) >= 0.12,
        30.0,
        "robot did not start moving before obstacle insertion",
    )
    old_path = node.paths[-1][1]
    current = node.positions[-1]
    target_position = (1.2, 0.0)
    obstacle_position = offset_from_path(
        point_ahead_on_path(old_path, current, target_position),
        current,
        target_position,
    )
    target_dx = target_position[0] - current[0]
    target_dy = target_position[1] - current[1]
    target_length = math.hypot(target_dx, target_dy) or 1.0
    forward_projection = (
        (obstacle_position[0] - current[0]) * target_dx
        + (obstacle_position[1] - current[1]) * target_dy
    ) / target_length
    if forward_projection < 0.5:
        raise RuntimeError(
            f"dynamic obstacle was not placed ahead: projection={forward_projection:.3f}"
        )
    old_clearance = path_clearance(old_path, obstacle_position)
    obstacle_name = "voice_dynamic_obstacle"
    spawn_obstacle(node, obstacle_name, obstacle_position)
    spawned_at = time.monotonic()
    wait_until(
        lambda: any(
            timestamp > spawned_at
            and path_clearance(path, obstacle_position) >= old_clearance + 0.12
            for timestamp, path in node.paths
        ),
        45.0,
        "global path did not replan around the inserted obstacle",
    )
    wait_until(
        lambda: result_for(node, command_id) is not None,
        timeout,
        "dynamic obstacle navigation did not reach a terminal result",
    )
    result = result_for(node, command_id)
    if result.get("success") is not True:
        raise RuntimeError(f"Nav2 failed after dynamic replan: {result}")
    replanned_paths = [
        path for timestamp, path in node.paths
        if timestamp > spawned_at
        and path_clearance(path, obstacle_position) >= old_clearance + 0.12
    ]
    return {
        "obstacle": {
            "name": obstacle_name,
            "x": round(obstacle_position[0], 3),
            "y": round(obstacle_position[1], 3),
            "size_m": 0.2,
            "forward_projection_m": round(forward_projection, 3),
        },
        "old_path_clearance_m": round(old_clearance, 3),
        "replanned_path_clearance_m": round(
            path_clearance(replanned_paths[-1], obstacle_position), 3
        ),
        "paths_after_spawn": len(
            [timestamp for timestamp, _ in node.paths if timestamp > spawned_at]
        ),
        "result": result,
    }, candidate


def run_unreachable_navigation(node, timeout):
    before = len(node.results)
    node.text_pub.publish(String(data="去封闭区"))
    wait_until(
        lambda: any(
            candidate.get("name") == "navigate_to"
            and candidate.get("arguments", {}).get("target") == "unreachable_zone"
            for candidate in node.candidates
        ),
        20.0,
        "unreachable navigation candidate missing",
    )
    candidate = next(
        candidate for candidate in reversed(node.candidates)
        if candidate.get("arguments", {}).get("target") == "unreachable_zone"
    )
    command_id = candidate["request_id"]
    wait_until(
        lambda: len(node.results) > before and result_for(node, command_id) is not None,
        timeout,
        "unreachable Nav2 goal did not return a terminal failure",
    )
    result = result_for(node, command_id)
    if result.get("success") is not False:
        raise RuntimeError(f"unreachable goal unexpectedly succeeded: {result}")
    detail = str(result.get("message") or "")
    if "nav2:" not in detail or not any(
        marker in detail for marker in ("aborted", "error_code=", "timed_out")
    ):
        raise RuntimeError(f"unreachable failure lacks structured Nav2 detail: {result}")
    wait_until(
        lambda: bool(node.velocities)
        and abs(node.velocities[-1][0]) < 1e-6
        and abs(node.velocities[-1][1]) < 1e-6,
        10.0,
        "cmd_vel did not return to zero after unreachable goal",
    )
    return {"candidate": candidate, "result": result, "final_cmd_vel": node.velocities[-1]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--navigate-timeout", type=float, default=120.0)
    parser.add_argument("--patrol-timeout", type=float, default=240.0)
    parser.add_argument("--skip-patrol", action="store_true")
    parser.add_argument(
        "--resilience",
        action="store_true",
        help="动态插入路径障碍，并验证地图外目标的失败反馈",
    )
    parser.add_argument("--unreachable-timeout", type=float, default=150.0)
    parser.add_argument("--initial-x", type=float, default=-2.0)
    parser.add_argument("--initial-y", type=float, default=-0.5)
    parser.add_argument("--initial-yaw", type=float, default=0.0)
    parser.add_argument("--navigate-text", default="去门口")
    parser.add_argument("--patrol-text", default="依次去门口、书桌、起点")
    parser.add_argument(
        "--output",
        type=Path,
        help="覆盖报告路径，便于多个场景复用同一条语音导航探针",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Nav2TurtleBot3VoiceProbe()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: (
                node.text_pub.get_subscription_count() > 0
                and node.count_publishers("/robot/action_result") > 0
                and node.positions
                and node.map_metadata is not None
                and node.scan_count > 0
            ),
            60.0,
            "voice/Nav2/TurtleBot3 topics were not ready",
        )
        time.sleep(2.0)
        publish_initial_pose(node, args.initial_x, args.initial_y, args.initial_yaw)
        wait_until(
            lambda: node.localized_base_frame() is not None,
            20.0,
            "AMCL localization did not produce map->base transform",
        )
        localized_frame = node.localized_base_frame()
        localization = node.tf_buffer.lookup_transform("map", localized_frame, Time())
        wait_for_lifecycle_active(node.bt_state_client, 30.0, "bt_navigator")
        if not args.skip_patrol and not args.resilience:
            wait_for_lifecycle_active(
                node.waypoint_state_client, 30.0, "waypoint_follower"
            )

        dynamic_obstacle = None
        unreachable = None
        if args.resilience:
            dynamic_obstacle, navigate_candidate = run_dynamic_obstacle_navigation(
                node, args.navigate_timeout
            )
        else:
            navigate_candidate = run_command(
                node, args.navigate_text, "navigate_to", args.navigate_timeout
            )
        distance_after_nav = traveled_distance(node.positions)
        if distance_after_nav < 0.05:
            raise RuntimeError(
                f"Nav2 navigate_to returned but odom movement was too small: "
                f"{distance_after_nav:.3f} m"
            )

        patrol_candidate = None
        if not args.skip_patrol and not args.resilience:
            patrol_candidate = run_command(
                node, args.patrol_text, "follow_waypoints", args.patrol_timeout
            )
        if args.resilience:
            unreachable = run_unreachable_navigation(node, args.unreachable_timeout)

        # Action 成功只说明 Nav2 行为树到达终态；主演示还要求控制输出真正归零，
        # 防止 executor/速度平滑器在任务结束后遗留上一帧速度。
        wait_until(
            lambda: bool(node.velocities)
            and abs(node.velocities[-1][0]) < 1e-6
            and abs(node.velocities[-1][1]) < 1e-6,
            10.0,
            "cmd_vel did not return to zero after navigation mission",
        )
        final_cmd_vel = {
            "linear_x": node.velocities[-1][0],
            "angular_z": node.velocities[-1][1],
        }

        report = {
            "navigate_to": navigate_candidate.get("arguments", {}),
            "follow_waypoints": (
                None if patrol_candidate is None else patrol_candidate.get("arguments", {})
            ),
            "result_count": len(node.results),
            "distance_m": round(traveled_distance(node.positions), 3),
            "map": node.map_metadata,
            "scan_count": node.scan_count,
            "final_cmd_vel": final_cmd_vel,
            "dynamic_obstacle": dynamic_obstacle,
            "unreachable_goal": unreachable,
            "localization": {
                "parent_frame": localization.header.frame_id,
                "child_frame": localization.child_frame_id,
                "x": round(localization.transform.translation.x, 3),
                "y": round(localization.transform.translation.y, 3),
            },
            "status": "PASS",
        }
        output_path = args.output or (
            Path(__file__).resolve().parents[3]
            / "logs"
            / (
                "nav2_resilience_report.json"
                if args.resilience
                else "nav2_turtlebot3_voice_report.json"
            )
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        print(json.dumps(report, ensure_ascii=False, indent=2))
        print(f"Evidence: {output_path}")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    main()
