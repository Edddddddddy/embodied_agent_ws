"""ROS 2 SLAM/Nav2 会话观察器与 typed 接口 Adapter。

本模块只负责把 ROS 消息、Action 和 Service 转换成普通 Python 值。场景编排、
Gazebo 操作以及 PASS/FAIL 判定分别属于 ``dynamic_scenario`` 和纯证据模块。
"""

from __future__ import annotations

import time
from collections.abc import Callable

from action_msgs.msg import GoalStatus, GoalStatusArray
from embodied_agent_core.ros_action_transport import (
    command_message_to_dict,
    result_message_to_dict,
)
from embodied_agent_core.ros_qos import command_qos, event_qos, sensor_qos, state_qos
from embodied_agent_interfaces.action import ExecuteRobotCommand, ManageSlamSession
from embodied_agent_interfaces.msg import (
    DynamicObstacleArray,
    RobotCommand,
    RobotCommandResult,
    SlamSessionState,
)
from geometry_msgs.msg import Pose, PoseArray, PoseWithCovarianceStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose, NavigateToPose
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


StateObserver = Callable[[SlamSessionState], None]


class SessionObserver(Node):
    """汇集一次真实验收会话的运行时事实，不拥有验收阈值。"""

    def __init__(self, state_observer: StateObserver | None = None) -> None:
        super().__init__("voice_slam_session_probe")
        self.state_observer = state_observer
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
            lambda message: self.candidates.append(
                command_message_to_dict(message)
            ),
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
            lambda message: self.results.append(
                result_message_to_dict(message)
            ),
            event_qos(depth=20),
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, sensor_qos())
        self.create_subscription(
            OccupancyGrid, "/map", self._on_map, state_qos()
        )
        self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos())
        self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, command_qos(depth=20)
        )

        self.client = ActionClient(self, ManageSlamSession, "/slam/manage_session")
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
        if self.state_observer is not None:
            # 展示层通过回调接收状态，Observer 本身无需知道终端阶段编号。
            self.state_observer(message)

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


def wait_until(predicate: Callable[[], bool], timeout: float, description: str) -> None:
    """等待由 ROS executor 异步更新的条件，超时给出可操作的错误。"""

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise TimeoutError(description)


def request_path(node: SessionObserver, goal_x: float, goal_y: float) -> NavPath:
    if not node.compute_path_client.wait_for_server(timeout_sec=20.0):
        raise TimeoutError("ComputePathToPose action server unavailable")
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
        (float(pose.pose.position.x), float(pose.pose.position.y))
        for pose in path.poses[::stride]
    )


def lifecycle_states(node: SessionObserver) -> dict[str, int]:
    states: dict[str, int] = {}
    for name, client in node.lifecycle_clients.items():
        if not client.wait_for_service(timeout_sec=10.0):
            raise TimeoutError(f"{name} lifecycle service unavailable")
        future = client.call_async(GetState.Request())
        wait_until(future.done, 5.0, f"{name} lifecycle response timeout")
        states[name] = int(future.result().current_state.id)
    return states


def run_text_action(
    node: SessionObserver,
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
    # 不能用“最近一条 result”推进流程：上一个 Action 的迟到结果可能串台。
    # request_id/command_id 相关后，只有本次候选对应的终态才能解除等待。
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
