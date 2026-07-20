"""ROS 2 SLAM/Nav2 会话观察器与 typed 接口 Adapter。

本模块只负责 ROS/Gazebo subscription wiring 和深模块组合；路径关联、定位采样、
场景编排与 PASS/FAIL 判定分别由各自模块拥有。
"""

from __future__ import annotations

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
    SlamNavigationGoalEvidence,
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

from tools.acceptance.probes.slam_nav.localization_sampler import (
    GazeboTruthConfig,
    LocalizationSampler,
    _gazebo_timestamp_s,
    _ros_timestamp_s,
)
from tools.acceptance.probes.slam_nav.motion_evidence import MotionEvidenceTracker
from tools.acceptance.probes.slam_nav.sampled_goal_tracker import (
    SampledGoalPlanSnapshot,
    SampledGoalPlanTracker,
)
from tools.acceptance.probes.slam_nav.session_actions import (
    lifecycle_states,
    nav_path_points,
    request_path,
    run_text_action,
    wait_until,
)
from tools.acceptance.unknown_world_evidence import (
    LocalizationSample,
    MapWorldTransform,
)

try:
    from gz.msgs10.pose_v_pb2 import Pose_V as GazeboPoseVector
    from gz.transport13 import Node as GazeboTransportNode
except ImportError:  # GitHub 的轻量 repository test 不强制安装 Gazebo binding。
    GazeboPoseVector = None
    GazeboTransportNode = None


StateObserver = Callable[[SlamSessionState], None]

__all__ = (
    "GazeboTruthConfig",
    "LocalizationSample",
    "MapWorldTransform",
    "SessionObserver",
    "SlamNavigationGoalEvidence",
    "StateObserver",
    "_gazebo_timestamp_s",
    "_ros_timestamp_s",
    "lifecycle_states",
    "nav_path_points",
    "request_path",
    "run_text_action",
    "wait_until",
)


def _frontier_message_to_dict(message) -> dict[str, object]:
    return {
        name: getattr(message, name)
        for name in (
            "valid",
            "status",
            "detected_frontier_count",
            "available_frontier_count",
            "blacklisted_frontier_count",
            "active_goal_count",
            "active_goal_id",
            "accepted_goal_count",
            "succeeded_goal_count",
            "aborted_goal_count",
            "canceled_goal_count",
            "rejected_goal_count",
            "last_goal_terminal",
            "provider_completion_reason",
            "mission_completion_reason",
        )
    }


class SessionObserver(Node):
    """汇集一次真实验收会话的运行时事实，不拥有验收阈值。"""

    def __init__(
        self,
        state_observer: StateObserver | None = None,
        *,
        gazebo_truth_config: GazeboTruthConfig | None = None,
    ) -> None:
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
        self.mission_profile = SlamSessionState.PROFILE_UNSPECIFIED
        self.frontier_evidence: dict[str, object] = {}
        self.frontier_goal_ids: set[bytes] = set()
        self.localization_tf_count = 0
        self.latest_tracks: DynamicObstacleArray | None = None
        self.latest_costmap: Costmap | None = None
        self._motion_evidence_tracker = MotionEvidenceTracker()
        self._sampled_goal_tracker = SampledGoalPlanTracker()
        self._localization_sampler = LocalizationSampler(gazebo_truth_config)

        self.asr_pub = self.create_publisher(
            String, "/agent/asr_final", command_qos(depth=10)
        )
        self.text_pub = self.create_publisher(
            String, "/agent/text_input", command_qos(depth=10)
        )
        self.detection_pub = self.create_publisher(
            PoseArray, "/perception/dynamic_obstacle_detections", sensor_qos()
        )
        self._wire_subscriptions()
        self._wire_clients()
        self._gazebo_transport_node = self._wire_gazebo_truth(gazebo_truth_config)

    def _wire_subscriptions(self) -> None:
        self.create_subscription(
            SlamSessionState, "/slam/session_state", self._on_state, state_qos()
        )
        self.create_subscription(
            RobotCommand,
            "/agent/action_candidate",
            lambda message: self.candidates.append(command_message_to_dict(message)),
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
            NavPath, "/plan", self._on_navigation_plan, event_qos(depth=20)
        )
        self.create_subscription(
            RobotCommandResult,
            "/robot/action_result",
            lambda message: self.results.append(result_message_to_dict(message)),
            event_qos(depth=20),
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, sensor_qos())
        self.create_subscription(OccupancyGrid, "/map", self._on_map, state_qos())
        self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos())
        self.create_subscription(
            Twist, "/cmd_vel", self._on_cmd_vel, command_qos(depth=20)
        )

    def _wire_clients(self) -> None:
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

    def _wire_gazebo_truth(self, config: GazeboTruthConfig | None):
        if config is None:
            return None
        if GazeboTransportNode is None or GazeboPoseVector is None:
            raise RuntimeError(
                "Gazebo Python transport binding is required for localization truth"
            )
        node = GazeboTransportNode()
        node.subscribe(GazeboPoseVector, config.topic, self._on_gazebo_pose_vector)
        return node

    @property
    def sampled_goal_plans(self) -> dict[int, list[NavPath]]:
        return {
            key: list(value)
            for key, value in self._sampled_goal_tracker.snapshot().plans.items()
        }

    @property
    def navigation_plans(self) -> list[NavPath]:
        return list(self._sampled_goal_tracker.snapshot().navigation_plans)

    def sampled_goal_evidence_snapshot(self) -> SampledGoalPlanSnapshot:
        """一次加锁读取 goal/plan，避免报告跨两个 callback 代际拼接。"""

        return self._sampled_goal_tracker.snapshot()

    @property
    def last_cmd_vel(self) -> dict[str, float]:
        evidence = self._motion_evidence_tracker.snapshot()
        return {"linear_x": evidence.linear_x, "angular_z": evidence.angular_z}

    @property
    def cmd_vel_sample_count(self) -> int:
        return self._motion_evidence_tracker.snapshot().sample_count

    @property
    def nonzero_cmd_vel_sample_count(self) -> int:
        return self._motion_evidence_tracker.snapshot().nonzero_sample_count

    def motion_evidence(self) -> dict[str, float | int]:
        """返回自洽的速度终止证据，供 evaluator Adapter 一次性消费。"""

        return self._motion_evidence_tracker.snapshot().as_dict()

    def mark_final_stop_boundary(self) -> float:
        """标记最后一个会驱动机器人的任务起点，后续零速必须晚于它。"""

        return self._motion_evidence_tracker.mark_boundary()

    def has_fresh_terminal_stop(self) -> bool:
        return self._motion_evidence_tracker.snapshot().has_fresh_stop()

    def has_fresh_final_motion_stop(self) -> bool:
        """最终任务必须在边界后确实运动，并以边界后的新零速结束。"""

        return self._motion_evidence_tracker.snapshot().has_fresh_motion_stop()

    @property
    def amcl_pose_count(self) -> int:
        return self._localization_sampler.amcl_pose_count

    def has_phase(self, phase: int) -> bool:
        return any(state.phase == phase for state in self.states)

    def _update_sampled_goal_evidence(self, message: SlamSessionState) -> None:
        self._sampled_goal_tracker.update_state(message)

    def _on_state(self, message: SlamSessionState) -> None:
        self.states.append(message)
        self.current_phase = int(message.phase)
        self.current_detail = str(message.detail)
        self.mission_profile = int(
            getattr(message, "mission_profile", SlamSessionState.PROFILE_UNSPECIFIED)
        )
        self._sampled_goal_tracker.update_state(message)
        frontier = getattr(message, "frontier", None)
        if frontier is not None:
            self.frontier_evidence = _frontier_message_to_dict(frontier)
        self._localization_sampler.transition_phase(self.current_phase)
        if int(message.mission_outcome) in {
            SlamSessionState.MISSION_SUCCEEDED,
            SlamSessionState.MISSION_FAILED,
            SlamSessionState.MISSION_CANCELED,
        }:
            # transient-local 或周期重发不能把终态边界推到 stop 之后。
            self._motion_evidence_tracker.mark_boundary(only_if_unset=True)
        if self.state_observer is not None:
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
        self._motion_evidence_tracker.observe(
            float(message.linear.x),
            float(message.angular.z),
        )

    def _on_navigation_plan(self, message: NavPath) -> None:
        self._sampled_goal_tracker.observe_plan(message)

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

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        self._localization_sampler.observe_amcl(message)

    def _on_gazebo_pose_vector(self, message) -> None:
        self._localization_sampler.observe_gazebo(message)

    def _on_tracks(self, message: DynamicObstacleArray) -> None:
        self.latest_tracks = message

    def _on_costmap(self, message: Costmap) -> None:
        self.latest_costmap = message

    def publish_detection(self, x: float, y: float) -> None:
        message = PoseArray()
        message.header.frame_id = "map"
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.orientation.w = 1.0
        message.poses.append(pose)
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

    def localization_evidence(
        self,
    ) -> tuple[
        tuple[LocalizationSample, ...],
        tuple[LocalizationSample, ...],
        str,
    ]:
        return self._localization_sampler.evidence()
