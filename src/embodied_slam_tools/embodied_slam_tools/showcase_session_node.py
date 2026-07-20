"""单终端编排真实感语音建图、保存地图和导航阶段。"""

from __future__ import annotations

import math
import os
from pathlib import Path
import queue
import threading
import time
from typing import Any, Callable

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time as TimeMessage
from embodied_agent_interfaces.action import ManageSlamSession
from embodied_agent_interfaces.msg import (
    FrontierExplorationEvidence,
    RobotCommand,
    RobotCommandResult,
    SlamNavigationGoalEvidence,
    SlamSessionState,
    SystemReadiness,
)
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import ComputePathToPose, FollowWaypoints, NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

try:
    from explore_lite_msgs.msg import ExploreStatus
except ImportError:  # 可选运行时由 setup_frontier_exploration.sh 安装。
    ExploreStatus = None

from .agent_action_gateway import AgentActionGateway, AgentActionOutcome
from .frontier_monitor import FrontierExplorationMonitor
from .mapped_goal_sampler import (
    OccupancySnapshot,
    inspect_path_occupancy,
    rank_mapped_goal_candidates,
)
from .mapping_evidence import (
    FrontierTelemetry,
    MappingEvidenceTracker,
    NavigationGoalEvidence,
    NavigationGoalLedger,
    NavigationGoalStatus,
)
from .mission_configuration import MissionConfiguration
from .mission_executor import (
    AutomaticMissionCancelled,
    AutomaticMissionExecutor,
    CommandRequest,
    UnknownWorldMissionExecutor,
    UnknownWorldMissionSpec,
)
from .stage_process_manager import StageProcessManager
from .showcase_session import (
    SessionCommand,
    SessionPhase,
    ShowcaseSessionStateMachine,
    is_automatic_mission_cancel_text,
    is_truncated_automatic_mission_text,
    parse_session_command,
)


_ACTION_TYPES = {
    "stop": RobotCommand.STOP,
    "move": RobotCommand.MOVE,
    "turn": RobotCommand.TURN,
    "navigate_to": RobotCommand.NAVIGATE_TO,
    "follow_waypoints": RobotCommand.FOLLOW_WAYPOINTS,
}
_PLAN_GOAL_TOLERANCE_M = 0.75
_NAV2_SAFETY_STOP_TIMEOUT_S = 10.0
_NAV2_TERMINAL_STATUSES = frozenset(
    {
        GoalStatus.STATUS_SUCCEEDED,
        GoalStatus.STATUS_ABORTED,
        GoalStatus.STATUS_CANCELED,
    }
)
_SKIPPABLE_PLANNER_ERROR_CODES = frozenset(
    {
        ComputePathToPose.Result.GOAL_OUTSIDE_MAP,
        ComputePathToPose.Result.GOAL_OCCUPIED,
        ComputePathToPose.Result.NO_VALID_PATH,
    }
)


class _NavigationGoalRejected(RuntimeError):
    """Nav2 在执行前拒绝目标；与执行中 ABORTED 分开记录。"""


class _GoalCandidateRejected(RuntimeError):
    """单个候选不可达或路径不安全；允许 admission 继续尝试下一点。"""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = str(reason)


def _navigation_safety_stop_request() -> CommandRequest:
    """创建不继承用户取消位的内部停车事务。"""

    # 用户 request 在 cancel 分支中必然 canceled=True；复用它会让 gateway 在
    # 发布 priority STOP 前直接退出。因此安全停车始终使用独立、未取消 request。
    return CommandRequest(
        command=SessionCommand.STOP_SESSION,
        source="navigation_safety_stop",
    )


def _raise_navigation_cleanup_failure(
    manager: StageProcessManager,
    *,
    reason: str,
    cleanup_errors: list[str],
) -> None:
    """最后防线：无法证明 goal 终态时停止整个导航 stage 并显式失败。"""

    try:
        manager.stop()
        shutdown_detail = "navigation stage stopped by fail-safe"
    except Exception as exc:  # manager.stop 本身失败也必须进入最终错误证据。
        shutdown_detail = f"navigation stage stop failed: {exc}"
    raise RuntimeError(
        f"{reason}; navigation safety cleanup failed: "
        + "; ".join((*cleanup_errors, shutdown_detail))
    )


def _read_nav2_terminal_wrapper(result_future) -> tuple[Any | None, str | None]:
    """读取可审计的 Nav2 Action 终态；返回 wrapper 或明确错误。"""

    if result_future is None:
        return None, "Nav2 result future unavailable"
    try:
        if not result_future.done():
            return None, "Nav2 goal did not reach terminal after cancel"
    except Exception as exc:
        return None, f"Nav2 result readiness check failed: {exc}"
    try:
        wrapped = result_future.result()
    except Exception as exc:
        return None, f"Nav2 result read failed: {exc}"
    if wrapped is None:
        return None, "Nav2 result future returned no wrapper"
    try:
        status = int(wrapped.status)
    except Exception as exc:
        return None, f"Nav2 terminal status is unreadable: {exc}"
    if status not in _NAV2_TERMINAL_STATUSES:
        return None, f"Nav2 wrapper has non-terminal status: {status}"
    try:
        result = wrapped.result
    except Exception as exc:
        return None, f"Nav2 terminal result payload is unreadable: {exc}"
    if result is None:
        return None, "Nav2 terminal result payload is missing"
    return wrapped, None


def _admit_goal_candidates(
    candidates: tuple[tuple[float, float], ...],
    *,
    count: int,
    minimum_separation_m: float,
    admit: Callable[[tuple[float, float]], None],
) -> tuple[tuple[float, float], ...]:
    """按候选顺序选择已通过实时规划的目标。"""

    selected: list[tuple[float, float]] = []
    attempted = 0
    rejections: dict[str, int] = {}
    for candidate in candidates:
        if any(
            math.dist(candidate, accepted) < minimum_separation_m
            for accepted in selected
        ):
            continue
        attempted += 1
        try:
            admit(candidate)
        except _GoalCandidateRejected as exc:
            rejections[exc.reason] = rejections.get(exc.reason, 0) + 1
            continue
        selected.append(candidate)
        if len(selected) == count:
            return tuple(selected)
    rejection_summary = ",".join(
        f"{reason}:{amount}"
        for reason, amount in sorted(rejections.items())
    ) or "none"
    raise RuntimeError(
        "insufficient_nav2_reachable_goals: "
        f"required={count} admitted={len(selected)} attempted={attempted} "
        f"rejections={rejection_summary}"
    )


def _path_occupancy_for_goal(
    snapshot: OccupancySnapshot,
    message: NavPath,
    goal_xy: tuple[float, float],
):
    """在指定地图快照上审计完整路径，避免 admission 偷用可变缓存。"""

    points = _path_points_for_goal(message, goal_xy)
    return inspect_path_occupancy(snapshot, points)


def _get_lifecycle_state(client, timeout_s: float) -> int | None:
    """在非 executor 工作线程中同步查询 lifecycle，超时返回 ``None``。"""

    response = client.call(GetState.Request(), timeout_sec=timeout_s)
    if response is None:
        return None
    return int(response.current_state.id)


def _path_points_for_goal(
    message: NavPath,
    goal_xy: tuple[float, float],
    *,
    tolerance_m: float = _PLAN_GOAL_TOLERANCE_M,
) -> tuple[tuple[float, float], ...]:
    """校验 `/plan` 坐标系与终点，拒绝迟到或无关目标的路径。"""

    if message.header.frame_id != "map":
        raise ValueError(
            f"navigation plan frame must be map, got {message.header.frame_id!r}"
        )
    points = tuple(
        (float(pose.pose.position.x), float(pose.pose.position.y))
        for pose in message.poses
    )
    if not points:
        raise ValueError("navigation plan must not be empty")
    endpoint_error_m = math.hypot(
        points[-1][0] - float(goal_xy[0]),
        points[-1][1] - float(goal_xy[1]),
    )
    if endpoint_error_m > tolerance_m:
        raise ValueError(
            "navigation plan endpoint does not match active sampled goal: "
            f"error={endpoint_error_m:.3f}m"
        )
    return points


def _frontier_telemetry_from_message(message: Any) -> FrontierTelemetry:
    """把可选版本的 ExploreStatus 收紧为稳定领域值对象。"""

    def counter(name: str) -> int:
        return int(getattr(message, name, 0))

    # getattr 默认值只为保留 known-world 旧安装兼容；unknown-world 启动检查会
    # 要求扩展 schema，不能把一串 0 当作完整 frontier 证据。
    return FrontierTelemetry(
        status=str(getattr(message, "status", "")),
        detected_frontier_count=counter("detected_frontier_count"),
        available_frontier_count=counter("available_frontier_count"),
        blacklisted_frontier_count=counter("blacklisted_frontier_count"),
        active_goal_count=counter("active_goal_count"),
        active_goal_id=str(getattr(message, "active_goal_id", "")),
        accepted_goal_count=counter("accepted_goal_count"),
        succeeded_goal_count=counter("succeeded_goal_count"),
        aborted_goal_count=counter("aborted_goal_count"),
        canceled_goal_count=counter("canceled_goal_count"),
        rejected_goal_count=counter("rejected_goal_count"),
        last_goal_terminal=str(getattr(message, "last_goal_terminal", "")),
        completion_reason=str(getattr(message, "completion_reason", "")),
    )


def _occupancy_snapshot_from_message(message: Any) -> OccupancySnapshot:
    """把 ROS OccupancyGrid 转成目标采样器的无 ROS 快照。"""

    orientation = message.info.origin.orientation
    if abs(float(orientation.x)) > 1e-6 or abs(float(orientation.y)) > 1e-6:
        raise ValueError("occupancy map origin roll/pitch is unsupported")
    if abs(float(orientation.z)) > 1e-6 or abs(float(orientation.w) - 1.0) > 1e-6:
        # 当前采样器刻意只支持 axis-aligned OccupancyGrid；静默忽略 yaw 会把目标
        # 投到错误 cell，显式失败比在未知区域导航更安全。
        raise ValueError("occupancy map origin yaw must be zero")
    return OccupancySnapshot(
        width=int(message.info.width),
        height=int(message.info.height),
        resolution_m=float(message.info.resolution),
        origin_xy=(
            float(message.info.origin.position.x),
            float(message.info.origin.position.y),
        ),
        cells=tuple(int(value) for value in message.data),
    )


def _time_message_from_ns(timestamp_ns: int) -> TimeMessage:
    message = TimeMessage()
    message.sec = int(timestamp_ns // 1_000_000_000)
    message.nanosec = int(timestamp_ns % 1_000_000_000)
    return message


def _frontier_evidence_message(
    telemetry: FrontierTelemetry,
    *,
    mission_completion_reason: str,
) -> FrontierExplorationEvidence:
    """领域 telemetry 到稳定项目接口的唯一 Adapter。"""

    message = FrontierExplorationEvidence()
    message.valid = bool(telemetry.status or telemetry.completion_reason)
    message.status = telemetry.status
    message.detected_frontier_count = telemetry.detected_frontier_count
    message.available_frontier_count = telemetry.available_frontier_count
    message.blacklisted_frontier_count = telemetry.blacklisted_frontier_count
    message.active_goal_count = telemetry.active_goal_count
    message.active_goal_id = telemetry.active_goal_id
    message.accepted_goal_count = telemetry.accepted_goal_count
    message.succeeded_goal_count = telemetry.succeeded_goal_count
    message.aborted_goal_count = telemetry.aborted_goal_count
    message.canceled_goal_count = telemetry.canceled_goal_count
    message.rejected_goal_count = telemetry.rejected_goal_count
    message.last_goal_terminal = telemetry.last_goal_terminal
    message.provider_completion_reason = telemetry.completion_reason
    message.mission_completion_reason = mission_completion_reason
    return message


def _navigation_goal_evidence_message(
    evidence: NavigationGoalEvidence,
) -> SlamNavigationGoalEvidence:
    message = SlamNavigationGoalEvidence()
    message.sequence = evidence.sequence
    message.goal.header.frame_id = "map"
    message.goal.pose.position.x = evidence.goal_xy[0]
    message.goal.pose.position.y = evidence.goal_xy[1]
    message.goal.pose.orientation.w = 1.0
    message.started_at = _time_message_from_ns(evidence.started_at_ns)
    message.finished_at = _time_message_from_ns(evidence.finished_at_ns)
    message.status = int(evidence.status)
    message.nav2_status = evidence.nav2_status
    message.nav2_error_code = evidence.nav2_error_code
    message.plan_count = evidence.plan_count
    message.max_unknown_cell_count = evidence.max_unknown_cell_count
    message.max_occupied_cell_count = evidence.max_occupied_cell_count
    message.max_outside_map_cell_count = evidence.max_outside_map_cell_count
    message.all_plans_known_free = evidence.all_plans_known_free
    message.detail = evidence.detail
    return message


class SessionOrchestratorNode(Node):
    def __init__(self) -> None:
        super().__init__("voice_slam_session_orchestrator")
        default_workspace = os.environ.get(
            "WORKSPACE", "/home/ubuntu/embodied_agent_ws"
        )
        workspace = Path(self.declare_parameter("workspace", default_workspace).value)
        mode = str(self.declare_parameter("mode", "offline").value)
        if mode not in {"offline", "online"}:
            raise ValueError("mode must be offline or online")
        default_prefix = workspace / "logs/showcase/voice_built_map"
        map_prefix = Path(
            self.declare_parameter("map_prefix", str(default_prefix)).value
        )
        default_mission_plan = (
            workspace
            / "src/embodied_simulation/config/showcase_workplace_mission.yaml"
        )
        mission_plan_path = Path(
            self.declare_parameter(
                "mission_plan", str(default_mission_plan)
            ).value
        )
        readiness_topic = str(
            self.declare_parameter("readiness_topic", "/system/readiness").value
        )
        self._startup_timeout_s = float(
            self.declare_parameter("startup_timeout_s", 120.0).value
        )
        scan_startup_timeout_s = float(
            self.declare_parameter("scan_startup_timeout_s", 20.0).value
        )
        stop_timeout_s = float(
            self.declare_parameter("stop_timeout_s", 15.0).value
        )
        dry_run = bool(self.declare_parameter("dry_run", False).value)
        dry_run_exploration_delay_s = float(
            self.declare_parameter("dry_run_exploration_delay_s", 0.0).value
        )
        # YAML key、默认值和阈值校验全部封装在配置深模块中；Node 只消费
        # 已冻结的任务值对象，避免运行期间继续查询松散字典。
        mission_configuration = MissionConfiguration.load(
            workspace=workspace,
            mission_plan_path=mission_plan_path,
            scan_startup_timeout_s=scan_startup_timeout_s,
            status_available=ExploreStatus is not None,
            dry_run=dry_run,
            dry_run_delay_s=dry_run_exploration_delay_s,
        )
        self._mission_profile = mission_configuration.profile
        if self._mission_profile == "unknown_world":
            required_fields = (
                "detected_frontier_count",
                "available_frontier_count",
                "blacklisted_frontier_count",
                "active_goal_count",
                "completion_reason",
            )
            if ExploreStatus is None or not all(
                hasattr(ExploreStatus(), field) for field in required_fields
            ):
                raise RuntimeError(
                    "unknown-world profile requires patched ExploreStatus; run "
                    "bash scripts/setup_frontier_exploration.sh"
                )
        self._fsm = ShowcaseSessionStateMachine()
        self._manager = StageProcessManager(
            workspace,
            mode,
            map_prefix,
            stop_timeout_s=stop_timeout_s,
            dry_run=dry_run,
            mission_profile=self._mission_profile,
        )
        self._mapping_evidence = MappingEvidenceTracker(
            mission_configuration.evidence_min_growth_cells
        )
        self._frontier_monitor = FrontierExplorationMonitor(
            self._mapping_evidence,
            self._manager,
            mission_configuration.frontier,
            cancel_motion=self._cancel_automatic_motion,
            log_info=self.get_logger().info,
        )
        if isinstance(mission_configuration.automatic, UnknownWorldMissionSpec):
            self._automatic_mission_executor = UnknownWorldMissionExecutor(
                self,
                self._manager,
                self._mapping_evidence,
                mission_configuration.automatic,
            )
        else:
            self._automatic_mission_executor = AutomaticMissionExecutor(
                self,
                self._manager,
                self._mapping_evidence,
                mission_configuration.automatic,
            )
        self._dry_run = dry_run
        self._state_lock = threading.RLock()
        self._ready_condition = threading.Condition()
        self._ready_generation = 0
        self._latest_ready = False
        self._intent_lock = threading.Lock()
        self._last_intent: tuple[SessionCommand | None, float] = (None, 0.0)
        self._requests: queue.Queue[CommandRequest | None] = queue.Queue(maxsize=4)
        self._worker = threading.Thread(
            target=self._worker_loop,
            name="slam-session-worker",
            daemon=True,
        )
        self._stopping = threading.Event()
        self._operation_active = threading.Event()
        self._active_request: CommandRequest | None = None
        self._map_pose_condition = threading.Condition()
        self._latest_occupancy: OccupancySnapshot | None = None
        self._latest_map_pose_xy: tuple[float, float] | None = None
        self._navigation_input_generation = 0
        self._latest_occupancy_generation = -1
        self._latest_map_pose_generation = -1
        self._mission_sequence = 0
        self._mission_outcome = SlamSessionState.MISSION_IDLE
        self._mission_message = ""
        self._internal_command_sequence = 0
        self._navigation_goal_ledger = NavigationGoalLedger()

        state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        event_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        callback_group = ReentrantCallbackGroup()
        self._state_pub = self.create_publisher(
            SlamSessionState, "/slam/session_state", state_qos
        )
        self._agent_text_pub = self.create_publisher(
            String, "/agent/text_input", event_qos
        )
        self._internal_action_pub = self.create_publisher(
            RobotCommand, "/agent/action_candidate", event_qos
        )
        self._agent_action_gateway = AgentActionGateway(
            publish_text=lambda text: self._agent_text_pub.publish(
                String(data=text)
            ),
            subscriber_count=self._agent_text_pub.get_subscription_count,
            cancel_motion=self._cancel_automatic_motion,
            dry_run=dry_run,
            log_dry_run=lambda message: print(message, flush=True),
        )
        self.create_subscription(
            String,
            "/agent/asr_final",
            self._on_asr_final,
            event_qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            SystemReadiness,
            readiness_topic,
            self._on_readiness,
            state_qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            RobotCommand,
            "/agent/action_candidate",
            self._on_agent_candidate,
            event_qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            RobotCommandResult,
            "/robot/action_result",
            self._on_agent_result,
            event_qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            OccupancyGrid,
            "/map",
            self._on_map,
            state_qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self._on_odom,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self._on_amcl_pose,
            state_qos,
            callback_group=callback_group,
        )
        self.create_subscription(
            LaserScan,
            "/scan",
            self._on_scan,
            qos_profile_sensor_data,
            callback_group=callback_group,
        )
        self.create_subscription(
            NavPath,
            "/plan",
            self._on_navigation_plan,
            event_qos,
            callback_group=callback_group,
        )
        if ExploreStatus is not None:
            self.create_subscription(
                ExploreStatus,
                "/explore/status",
                self._on_explore_status,
                state_qos,
                callback_group=callback_group,
            )
        self._navigate_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            "/navigate_to_pose",
            callback_group=callback_group,
        )
        self._compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            "/compute_path_to_pose",
            callback_group=callback_group,
        )
        self._follow_waypoints_client = ActionClient(
            self,
            FollowWaypoints,
            "/follow_waypoints",
            callback_group=callback_group,
        )
        self._bt_navigator_state_client = self.create_client(
            GetState,
            "/bt_navigator/get_state",
            callback_group=callback_group,
        )
        self._waypoint_follower_state_client = self.create_client(
            GetState,
            "/waypoint_follower/get_state",
            callback_group=callback_group,
        )
        self._action_server = ActionServer(
            self,
            ManageSlamSession,
            "/slam/manage_session",
            execute_callback=self._execute_action,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=callback_group,
        )
        self.create_timer(1.0, self._check_child_process)
        self._publish_state()
        self._worker.start()

    def _state_message(self) -> SlamSessionState:
        snapshot = self._fsm.snapshot
        evidence = self._mapping_evidence.snapshot()
        message = SlamSessionState()
        message.stamp = self.get_clock().now().to_msg()
        message.phase = int(snapshot.phase)
        message.map_saved = snapshot.map_saved
        message.map_yaml_path = snapshot.map_yaml_path
        message.detail = snapshot.detail
        message.mission_profile = (
            SlamSessionState.PROFILE_UNKNOWN_WORLD
            if self._mission_profile == "unknown_world"
            else SlamSessionState.PROFILE_KNOWN_WORLD
        )
        message.mission_sequence = self._navigation_goal_ledger.mission_sequence
        message.mission_outcome = self._mission_outcome
        message.mission_message = self._mission_message
        message.frontier = _frontier_evidence_message(
            evidence.frontier_telemetry,
            mission_completion_reason=evidence.exploration_completion_reason,
        )
        message.sampled_navigation_goals = [
            _navigation_goal_evidence_message(item)
            for item in self._navigation_goal_ledger.snapshot()
        ]
        return message

    def _publish_state(self) -> None:
        with self._state_lock:
            message = self._state_message()
        self._state_pub.publish(message)
        self.get_logger().info(
            f"session phase={SessionPhase(message.phase).name.lower()} "
            f"map_saved={message.map_saved} detail={message.detail}"
        )

    def transition(self, phase: SessionPhase, **kwargs) -> None:
        with self._state_lock:
            if phase == SessionPhase.MISSION_COMPLETED:
                # phase 与 outcome 必须在同一把锁内成为一个快照。若先发布
                # MISSION_COMPLETED/RUNNING，再补发 SUCCEEDED，late join probe
                # 可能把第一帧当终态并产生确定性的假 FAIL。
                self._mission_outcome = SlamSessionState.MISSION_SUCCEEDED
                self._mission_message = "automatic mission completed"
            self._fsm.transition(phase, **kwargs)
        self._publish_state()

    def _on_readiness(self, message: SystemReadiness) -> None:
        with self._ready_condition:
            self._ready_generation += 1
            self._latest_ready = bool(message.ready)
            self._ready_condition.notify_all()

    def _on_agent_candidate(self, message: RobotCommand) -> None:
        self._agent_action_gateway.record_candidate(
            int(message.action_type), str(message.command_id)
        )

    def _on_agent_result(self, message: RobotCommandResult) -> None:
        self._agent_action_gateway.record_result(
            str(message.command_id),
            AgentActionOutcome(
                success=bool(message.success),
                status=int(message.status),
                message=str(message.message),
            ),
        )

    def _on_map(self, message: OccupancyGrid) -> None:
        self._mapping_evidence.record_map(message.data)
        try:
            snapshot = _occupancy_snapshot_from_message(message)
        except ValueError as exc:
            self.get_logger().error(f"cannot sample goals from /map: {exc}")
            return
        with self._map_pose_condition:
            self._latest_occupancy = snapshot
            self._latest_occupancy_generation = (
                self._navigation_input_generation
            )
            self._map_pose_condition.notify_all()

    def _on_scan(self, _message: LaserScan) -> None:
        # SystemReadiness 只能证明图中关键节点已经启动；Gazebo 传感器首帧可能
        # 稍晚到达。显式记录首帧，避免 bootstrap 动作被安全执行器按 scan_timeout 拒绝。
        self._mapping_evidence.mark_scan_ready()

    def _on_odom(self, message: Odometry) -> None:
        """累计本次自动建图的真实里程，防止覆盖阈值让探索过早结束。"""

        position = message.pose.pose.position
        self._mapping_evidence.record_odom(position.x, position.y)

    def _on_amcl_pose(self, message: PoseWithCovarianceStamped) -> None:
        position = message.pose.pose.position
        with self._map_pose_condition:
            self._latest_map_pose_xy = (
                float(position.x),
                float(position.y),
            )
            self._latest_map_pose_generation = (
                self._navigation_input_generation
            )
            self._map_pose_condition.notify_all()

    def _on_explore_status(self, message: Any) -> None:
        self._mapping_evidence.record_frontier_telemetry(
            _frontier_telemetry_from_message(message)
        )

    def _record_navigation_plan(self, sequence: int, message: NavPath) -> bool:
        goal = self._navigation_goal_ledger.get(sequence)
        points = _path_points_for_goal(message, goal.goal_xy)
        with self._map_pose_condition:
            occupancy = self._latest_occupancy
        if occupancy is None:
            raise RuntimeError("cannot audit navigation plan before /map")
        stats = inspect_path_occupancy(occupancy, points)
        self._navigation_goal_ledger.record_plan(
            sequence,
            unknown_cell_count=stats.unknown_cell_count,
            occupied_cell_count=stats.occupied_cell_count,
            outside_map_cell_count=stats.outside_map_cell_count,
        )
        return stats.known_free

    def _on_navigation_plan(self, message: NavPath) -> None:
        sequence = self._navigation_goal_ledger.active_sequence()
        if sequence is None:
            return
        try:
            known_free = self._record_navigation_plan(sequence, message)
        except (RuntimeError, ValueError) as exc:
            self.get_logger().error(f"cannot audit Nav2 plan: {exc}")
            return
        if not known_free:
            # 生产侧也做已知自由区检查，是运行安全保护；验收器仍会独立复算，
            # 因而不能靠伪造这个布尔值让任务通过。
            self.get_logger().error(
                f"sampled navigation plan {sequence} crosses unknown/occupied cells"
            )

    def _cancel_automatic_motion(self) -> None:
        self._agent_text_pub.publish(String(data="停下"))
        self._manager.stop_explorer()

    def run_agent_action(
        self,
        request: CommandRequest,
        *,
        text: str,
        expected_action: str,
        timeout_s: float,
    ) -> None:
        """复用 Agent 的 NLU 与 typed command 链路执行自动任务中的语义动作。"""

        self._agent_action_gateway.run(
            request,
            text=text,
            expected_action_type=_ACTION_TYPES[expected_action],
            expected_action_name=expected_action,
            timeout_s=timeout_s,
        )

    def stop_motion_and_wait(
        self,
        request: CommandRequest,
        *,
        timeout_s: float,
    ) -> None:
        """旁路文本意图层停车，并等待 typed Action 终态。"""

        self._internal_command_sequence += 1
        command = RobotCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.command_id = (
            f"slam-internal-stop-{self._mission_sequence}-"
            f"{self._internal_command_sequence}"
        )
        command.source = "slam_session_orchestrator"
        command.priority = True
        command.action_type = RobotCommand.STOP
        # 恢复动作属于任务编排控制面。直接进入 ActionGuard 仍保留限幅、
        # Action 取消和结果关联，又不会让“停下”回流成一次用户语音意图。
        self._agent_action_gateway.run_typed(
            request,
            command_id=command.command_id,
            publish_command=lambda: self._internal_action_pub.publish(command),
            expected_action_name="stop",
            timeout_s=timeout_s,
        )

    def wait_for_frontier(
        self,
        request: CommandRequest,
        *,
        recovery_attempts_remaining: int = 0,
        deadline_monotonic: float | None = None,
    ) -> str:
        return self._frontier_monitor.wait(
            request,
            recovery_attempts_remaining=recovery_attempts_remaining,
            deadline_monotonic=deadline_monotonic,
        )

    def select_mapped_navigation_goals(
        self,
        request: CommandRequest,
        *,
        count: int,
        seed: int,
        minimum_separation_m: float,
        clearance_m: float,
        timeout_s: float,
    ) -> tuple[tuple[float, float], ...]:
        """从保存图候选中选出已通过实时 Nav2 规划的完整目标批次。"""

        if timeout_s <= 0.0:
            raise ValueError("goal admission timeout must be positive")
        if self._dry_run:
            goals = tuple((float(index + 1), 0.0) for index in range(count))
            self._navigation_goal_ledger.plan(goals)
            self._publish_state()
            return goals
        deadline = time.monotonic() + timeout_s
        with self._map_pose_condition:
            while (
                self._latest_occupancy is None
                or self._latest_map_pose_xy is None
                or self._latest_occupancy_generation
                != self._navigation_input_generation
                or self._latest_map_pose_generation
                != self._navigation_input_generation
            ):
                if request.canceled:
                    raise AutomaticMissionCancelled(
                        "automatic mission canceled"
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        "fresh saved /map and AMCL pose unavailable for "
                        "goal admission"
                    )
                self._map_pose_condition.wait(timeout=min(0.5, remaining))
            occupancy = self._latest_occupancy
            start_xy = self._latest_map_pose_xy
        candidates = rank_mapped_goal_candidates(
            occupancy,
            start_xy=start_xy,
            seed=seed,
            # rejected candidate 不能成为 spacing anchor；先返回稠密、确定性的
            # 过采样序列，真正的目标间距只在 accepted 集合中计算。
            minimum_separation_m=0.0,
            clearance_m=clearance_m,
            candidate_limit=max(24, count * 8),
        )

        def admit(goal_xy: tuple[float, float]) -> None:
            if request.canceled:
                raise AutomaticMissionCancelled(
                    "automatic mission canceled"
                )
            if time.monotonic() >= deadline:
                raise TimeoutError("goal admission deadline exhausted")
            path = self._request_preflight_path(
                request,
                goal_xy=goal_xy,
                deadline=deadline,
            )
            stats = _path_occupancy_for_goal(occupancy, path, goal_xy)
            if not stats.known_free:
                # ComputePath 成功并不代表它遵守“只走本次已知自由区”；unsafe
                # 只淘汰当前候选，基础设施、TF、start 和 timeout 异常仍向上失败。
                raise _GoalCandidateRejected("unsafe_path")

        goals = _admit_goal_candidates(
            candidates,
            count=count,
            minimum_separation_m=minimum_separation_m,
            admit=admit,
        )
        # admission 期间不发布 PLANNED 状态，避免 ComputePath 的 `/plan` 被
        # tracker 误绑；只有完整批次凑齐后才原子登记恰好 count 个目标。
        self._navigation_goal_ledger.plan(goals)
        self._publish_state()
        self.get_logger().info(f"admitted unknown-world navigation goals: {goals}")
        return goals

    def run_navigation_goal(
        self,
        request: CommandRequest,
        *,
        sequence: int,
        goal_xy: tuple[float, float],
        timeout_s: float,
    ) -> None:
        """用 Nav2 typed Action 执行动态采样点，不回退到命名地点表。"""

        self._set_navigation_goal_status(
            sequence,
            NavigationGoalStatus.REQUESTED,
            detail="preflight path requested",
        )
        if self._dry_run:
            self._set_navigation_goal_status(
                sequence,
                NavigationGoalStatus.SUCCEEDED,
                nav2_status=GoalStatus.STATUS_SUCCEEDED,
                detail="dry-run navigation completed",
            )
            return
        deadline = time.monotonic() + timeout_s
        try:
            preflight = self._request_preflight_path(
                request,
                goal_xy=goal_xy,
                deadline=deadline,
            )
            if not self._record_navigation_plan(sequence, preflight):
                raise RuntimeError(
                    f"preflight path crosses unknown/occupied cells: {goal_xy}"
                )
        except AutomaticMissionCancelled:
            self._set_navigation_goal_status(
                sequence,
                NavigationGoalStatus.CANCELED,
                nav2_status=GoalStatus.STATUS_CANCELED,
                detail="mission canceled during path preflight",
            )
            raise
        except TimeoutError as exc:
            self._set_navigation_goal_status(
                sequence,
                NavigationGoalStatus.TIMED_OUT,
                detail=str(exc),
            )
            raise
        except Exception as exc:
            self._set_navigation_goal_status(
                sequence,
                NavigationGoalStatus.REJECTED,
                detail=str(exc),
            )
            raise
        try:
            wrapped = self._execute_sampled_nav2_goal(
                request,
                sequence=sequence,
                goal_xy=goal_xy,
                deadline=deadline,
            )
            nav2_result = getattr(wrapped, "result", None)
            error_code = int(
                getattr(
                    nav2_result,
                    "error_code",
                    NavigateToPose.Result.NONE,
                )
            )
            # ROS Action wrapper 的 SUCCEEDED 只说明协议正常结束；Nav2 仍可在
            # result.error_code 中报告业务失败。两者必须同时成功，typed 证据
            # 才能进入 SUCCEEDED，避免 evaluator 被协议层假阳性误导。
            succeeded = (
                wrapped.status == GoalStatus.STATUS_SUCCEEDED
                and nav2_result is not None
                and error_code == NavigateToPose.Result.NONE
            )
            terminal_status = (
                NavigationGoalStatus.SUCCEEDED
                if succeeded
                else NavigationGoalStatus.CANCELED
                if wrapped.status == GoalStatus.STATUS_CANCELED
                else NavigationGoalStatus.ABORTED
            )
            self._set_navigation_goal_status(
                sequence,
                terminal_status,
                nav2_status=int(wrapped.status),
                nav2_error_code=error_code,
                detail=(
                    "NavigateToPose succeeded"
                    if succeeded
                    else (
                        "NavigateToPose failed "
                        f"status={wrapped.status} error={error_code}"
                    )
                ),
            )
            if not succeeded:
                raise RuntimeError(
                    "navigation goal failed: "
                    f"goal={goal_xy} status={wrapped.status} error={error_code}"
                )
        except AutomaticMissionCancelled:
            self._set_navigation_goal_status_if_open(
                sequence,
                NavigationGoalStatus.CANCELED,
                nav2_status=GoalStatus.STATUS_CANCELED,
                detail="mission canceled during navigation",
            )
            raise
        except _NavigationGoalRejected as exc:
            self._set_navigation_goal_status_if_open(
                sequence,
                NavigationGoalStatus.REJECTED,
                detail=str(exc),
            )
            raise
        except TimeoutError as exc:
            self._set_navigation_goal_status_if_open(
                sequence,
                NavigationGoalStatus.TIMED_OUT,
                detail=str(exc),
            )
            raise
        except Exception as exc:
            self._set_navigation_goal_status_if_open(
                sequence,
                NavigationGoalStatus.ABORTED,
                nav2_status=GoalStatus.STATUS_ABORTED,
                detail=str(exc),
            )
            raise

    def _execute_sampled_nav2_goal(
        self,
        request: CommandRequest,
        *,
        sequence: int,
        goal_xy: tuple[float, float],
        deadline: float,
    ):
        """执行一个 Nav2 goal；所有异常由外层统一收口成 typed terminal。"""

        remaining = max(0.0, deadline - time.monotonic())
        if not self._navigate_to_pose_client.wait_for_server(
            timeout_sec=min(10.0, remaining)
        ):
            raise TimeoutError("NavigateToPose Action server unavailable")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        # map-frame 目标使用 zero/latest TF，避免 Gazebo 低实时率下目标 stamp
        # 短暂领先 map->odom 而被 Nav2 拒绝。
        goal.pose.pose.position.x = float(goal_xy[0])
        goal.pose.pose.position.y = float(goal_xy[1])
        goal.pose.pose.orientation.w = 1.0
        response = self._navigate_to_pose_client.send_goal_async(goal)
        while not response.done():
            if request.canceled:
                reason = "automatic mission canceled before goal response"
                # 不能只注册 late callback 后释放任务锁：回调与下一个任务会竞态。
                # 在独立安全预算内同步取得迟到 handle，并完成 cancel/STOP/终态。
                self._resolve_pending_nav2_goal_safely(
                    request,
                    response=response,
                    reason=reason,
                )
                raise AutomaticMissionCancelled("automatic mission canceled")
            if time.monotonic() >= deadline:
                reason = f"navigation goal response timeout: {goal_xy}"
                self._resolve_pending_nav2_goal_safely(
                    request,
                    response=response,
                    reason=reason,
                )
                raise TimeoutError(reason)
            time.sleep(0.05)
        try:
            handle = response.result()
        except Exception as exc:
            # response 标记 done 仍不代表能判断 goal 是否 accepted。此时没有
            # handle 可供定点取消，只能停止整个 stage 作为最终安全边界。
            _raise_navigation_cleanup_failure(
                self._manager,
                reason=f"cannot read navigation goal response for {goal_xy}",
                cleanup_errors=[f"Nav2 goal response failed: {exc}"],
            )
        if handle is None or not handle.accepted:
            raise _NavigationGoalRejected(f"navigation goal rejected: {goal_xy}")
        self._set_navigation_goal_status(
            sequence,
            NavigationGoalStatus.ACCEPTED,
            nav2_status=GoalStatus.STATUS_ACCEPTED,
            detail="NavigateToPose accepted",
        )
        self._set_navigation_goal_status(
            sequence,
            NavigationGoalStatus.EXECUTING,
            nav2_status=GoalStatus.STATUS_EXECUTING,
            detail="NavigateToPose executing",
        )
        try:
            result_future = handle.get_result_async()
        except Exception as exc:
            reason = f"cannot get Nav2 result future for goal {goal_xy}: {exc}"
            # handle 已 accepted，必须先 cancel + typed STOP；由于没有 future
            # 可证明终态，统一 helper 随后会停止 stage 并显式失败。
            self._cancel_nav2_goal_and_force_stop(
                request,
                handle=handle,
                result_future=None,
                reason=reason,
            )
            raise RuntimeError(reason)
        while not result_future.done():
            if request.canceled:
                reason = "automatic mission canceled during navigation"
                self._cancel_nav2_goal_and_force_stop(
                    request,
                    handle=handle,
                    result_future=result_future,
                    reason=reason,
                )
                raise AutomaticMissionCancelled("automatic mission canceled")
            record = self._navigation_goal_ledger.get(sequence)
            if record.plan_count > 0 and not record.all_plans_known_free:
                reason = (
                    f"unsafe runtime navigation plan for goal {sequence}"
                )
                self._cancel_nav2_goal_and_force_stop(
                    request,
                    handle=handle,
                    result_future=result_future,
                    reason=reason,
                )
                raise RuntimeError(reason)
            if time.monotonic() >= deadline:
                reason = f"navigation goal result timeout: {goal_xy}"
                self._cancel_nav2_goal_and_force_stop(
                    request,
                    handle=handle,
                    result_future=result_future,
                    reason=reason,
                )
                raise TimeoutError(reason)
            time.sleep(0.05)
        wrapped, terminal_error = _read_nav2_terminal_wrapper(result_future)
        if terminal_error is not None:
            reason = f"invalid NavigateToPose terminal result: {terminal_error}"
            self._cancel_nav2_goal_and_force_stop(
                request,
                handle=handle,
                result_future=result_future,
                reason=reason,
            )
            raise RuntimeError(reason)
        # `/plan` 与 Action result 属于不同 ROS topic，回调没有全局顺序；
        # 在 handle 仍可用于安全收口时排空迟到 replan。若路径证据变坏，仍要
        # 发 cancel + typed STOP，不能把异常抛给外层后直接释放任务互斥锁。
        time.sleep(0.3)
        record = self._navigation_goal_ledger.get(sequence)
        if record.plan_count == 0 or not record.all_plans_known_free:
            reason = f"unsafe or missing runtime plan for goal {sequence}"
            self._cancel_nav2_goal_and_force_stop(
                request,
                handle=handle,
                result_future=result_future,
                reason=reason,
            )
            raise RuntimeError(reason)
        return wrapped

    def _resolve_pending_nav2_goal_safely(
        self,
        request: CommandRequest,
        *,
        response,
        reason: str,
        timeout_s: float = _NAV2_SAFETY_STOP_TIMEOUT_S,
    ) -> None:
        """等待 pending response，并关闭可能迟到 accepted 的 Nav2 事务。"""

        if timeout_s <= 0.0:
            raise ValueError("navigation safety stop timeout must be positive")
        response_deadline = time.monotonic() + timeout_s
        while not response.done() and time.monotonic() < response_deadline:
            time.sleep(0.05)
        if not response.done():
            _raise_navigation_cleanup_failure(
                self._manager,
                reason=reason,
                cleanup_errors=[
                    "Nav2 goal response did not arrive within safety budget"
                ],
            )
        try:
            handle = response.result()
        except Exception as exc:
            _raise_navigation_cleanup_failure(
                self._manager,
                reason=reason,
                cleanup_errors=[f"Nav2 goal response failed: {exc}"],
            )
        if handle is None or not handle.accepted:
            return
        try:
            result_future = handle.get_result_async()
        except Exception as exc:
            # handle 已 accepted 却拿不到 result future，无法证明终态；仍先尝试
            # cancel + typed STOP，再由统一 helper 触发 stage fail-safe。
            result_future = None
            reason = f"{reason}; cannot get Nav2 result future: {exc}"
        self._cancel_nav2_goal_and_force_stop(
            request,
            handle=handle,
            result_future=result_future,
            reason=reason,
            timeout_s=timeout_s,
        )

    def _cancel_nav2_goal_and_force_stop(
        self,
        request: CommandRequest,
        *,
        handle,
        result_future,
        reason: str,
        timeout_s: float = _NAV2_SAFETY_STOP_TIMEOUT_S,
    ) -> None:
        """取消直接 Nav2 goal，并用 typed STOP 形成可等待的安全终态。"""

        # 仅保留参数用于统一所有调用分支；实际停车绝不复用可能 canceled 的
        # 用户 request，而在下方创建独立内部事务。
        del request
        if timeout_s <= 0.0:
            raise ValueError("navigation safety stop timeout must be positive")
        cleanup_errors: list[str] = []
        try:
            # 先发 cancel、随后立即发 priority STOP；不能先等 cancel timeout，
            # 否则控制器在故障路径上仍可能继续输出速度。
            handle.cancel_goal_async()
        except Exception as exc:
            cleanup_errors.append(f"cancel request failed: {exc}")
        try:
            # 使用独立安全预算，不能复用已经耗尽的导航 deadline。该调用会
            # 等待 typed stop Action result，保证任务锁在停车事实前不释放。
            # 即使调用方 request 已 canceled，也必须发出 priority STOP。
            self.stop_motion_and_wait(
                _navigation_safety_stop_request(),
                timeout_s=timeout_s,
            )
        except Exception as exc:
            cleanup_errors.append(f"typed stop failed: {exc}")

        if result_future is not None:
            terminal_deadline = time.monotonic() + timeout_s
            while time.monotonic() < terminal_deadline:
                try:
                    if result_future.done():
                        break
                except Exception:
                    # 具体错误由统一终态读取器生成，避免在安全分支重复协议判断。
                    break
                time.sleep(0.05)
        # future.done 只表示 Future 容器结束；还必须读取 wrapper，并验证 ROS
        # Action status 已进入 SUCCEEDED/ABORTED/CANCELED 且 payload 可读取。
        _, terminal_error = _read_nav2_terminal_wrapper(result_future)
        if terminal_error is not None:
            cleanup_errors.append(terminal_error)
        if cleanup_errors:
            _raise_navigation_cleanup_failure(
                self._manager,
                reason=reason,
                cleanup_errors=cleanup_errors,
            )

    def _set_navigation_goal_status_if_open(
        self,
        sequence: int,
        status: NavigationGoalStatus,
        *,
        nav2_status: int = 0,
        nav2_error_code: int = 0,
        detail: str,
    ) -> None:
        if self._navigation_goal_ledger.is_terminal(sequence):
            return
        self._set_navigation_goal_status(
            sequence,
            status,
            nav2_status=nav2_status,
            nav2_error_code=nav2_error_code,
            detail=detail,
        )

    def _set_navigation_goal_status(
        self,
        sequence: int,
        status: NavigationGoalStatus,
        *,
        nav2_status: int = 0,
        nav2_error_code: int = 0,
        detail: str,
    ) -> None:
        self._navigation_goal_ledger.transition(
            sequence,
            status,
            timestamp_ns=self.get_clock().now().nanoseconds,
            nav2_status=nav2_status,
            nav2_error_code=nav2_error_code,
            detail=detail,
        )
        self._publish_state()

    def _request_preflight_path(
        self,
        request: CommandRequest,
        *,
        goal_xy: tuple[float, float],
        deadline: float,
    ) -> NavPath:
        """先用 Nav2 planner 证明目标可达，再允许执行长任务 Action。"""

        remaining = max(0.0, deadline - time.monotonic())
        if not self._compute_path_client.wait_for_server(
            timeout_sec=min(10.0, remaining)
        ):
            raise TimeoutError("ComputePathToPose Action server unavailable")
        goal = ComputePathToPose.Goal()
        goal.goal.header.frame_id = "map"
        # zero/latest 与 Execute goal 保持一致，避免仿真低实时率下请求时间领先 TF。
        goal.goal.pose.position.x = float(goal_xy[0])
        goal.goal.pose.position.y = float(goal_xy[1])
        goal.goal.pose.orientation.w = 1.0
        response = self._compute_path_client.send_goal_async(goal)
        while not response.done():
            if request.canceled:
                raise AutomaticMissionCancelled("automatic mission canceled")
            if time.monotonic() >= deadline:
                raise TimeoutError(f"path preflight response timeout: {goal_xy}")
            time.sleep(0.05)
        handle = response.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f"path preflight rejected: {goal_xy}")
        result_future = handle.get_result_async()
        while not result_future.done():
            if request.canceled:
                handle.cancel_goal_async()
                raise AutomaticMissionCancelled("automatic mission canceled")
            if time.monotonic() >= deadline:
                handle.cancel_goal_async()
                raise TimeoutError(f"path preflight result timeout: {goal_xy}")
            time.sleep(0.05)
        wrapped = result_future.result()
        if wrapped is None or wrapped.result is None:
            raise RuntimeError(f"path preflight returned no result: {goal_xy}")
        error_code = int(getattr(wrapped.result, "error_code", 0))
        if (
            wrapped.status != GoalStatus.STATUS_SUCCEEDED
            or error_code != ComputePathToPose.Result.NONE
        ):
            if error_code in _SKIPPABLE_PLANNER_ERROR_CODES:
                # 204/206/208 只证明这个 endpoint 当前不可用；TF、start、
                # timeout 或 planner 配置失败会影响所有候选，必须原样向上失败。
                raise _GoalCandidateRejected(
                    f"planner_error_{error_code}"
                )
            raise RuntimeError(
                "path preflight failed: "
                f"goal={goal_xy} status={wrapped.status} "
                f"error={error_code}"
            )
        if not wrapped.result.path.poses:
            raise RuntimeError(f"path preflight returned an empty path: {goal_xy}")
        return wrapped.result.path

    def _wait_for_new_ready(self, generation: int) -> None:
        if self._dry_run:
            return
        deadline = time.monotonic() + self._startup_timeout_s
        with self._ready_condition:
            while time.monotonic() < deadline:
                if self._ready_generation > generation and self._latest_ready:
                    return
                self._ready_condition.wait(
                    timeout=min(0.5, max(0.0, deadline - time.monotonic()))
                )
        raise TimeoutError("stage did not publish ready SystemReadiness before timeout")

    def wait_navigation_ready(self, request: CommandRequest) -> None:
        """通用 readiness 之后再验证 Nav2 的两个长任务 Action 已真正激活。"""

        if self._dry_run:
            return
        deadline = time.monotonic() + self._startup_timeout_s
        clients = (
            ("navigate_to_pose", self._navigate_to_pose_client),
            ("follow_waypoints", self._follow_waypoints_client),
        )
        for name, client in clients:
            while not client.server_is_ready():
                if request.canceled:
                    self._cancel_automatic_motion()
                    raise AutomaticMissionCancelled("automatic mission canceled")
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(f"Nav2 Action server unavailable: {name}")
                client.wait_for_server(timeout_sec=min(0.5, remaining))
        lifecycle_clients = (
            ("bt_navigator", self._bt_navigator_state_client),
            ("waypoint_follower", self._waypoint_follower_state_client),
        )
        for name, client in lifecycle_clients:
            while True:
                if request.canceled:
                    self._cancel_automatic_motion()
                    raise AutomaticMissionCancelled("automatic mission canceled")
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(f"Nav2 lifecycle node not active: {name}")
                if not client.wait_for_service(timeout_sec=min(0.5, remaining)):
                    continue
                # 这里运行在专用 worker，而不是 ROS callback。同步 Client.call 会
                # 用事件等待 executor 处理响应，并在每次短超时后清理 pending request；
                # 旧的 call_async + busy polling 在 Gazebo 高负载下会把有效响应饿死。
                state = _get_lifecycle_state(client, min(0.5, remaining))
                if state == State.PRIMARY_STATE_ACTIVE:
                    break
                time.sleep(0.1)
        self.get_logger().info(
            "Nav2 navigation Action servers and lifecycle nodes are active"
        )

    def _readiness_generation(self) -> int:
        with self._ready_condition:
            return self._ready_generation

    def _goal_callback(self, goal_request) -> GoalResponse:
        try:
            command = SessionCommand(goal_request.command)
        except ValueError:
            return GoalResponse.REJECT
        with self._state_lock:
            accepted, _ = self._fsm.validate(command)
        if not accepted or self._operation_active.is_set():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _enqueue(self, request: CommandRequest) -> bool:
        if self._operation_active.is_set():
            return False
        with self._state_lock:
            accepted, reason = self._fsm.validate(request.command)
        if not accepted:
            request.message = reason
            request.completed.set()
            return False
        try:
            self._requests.put_nowait(request)
            return True
        except queue.Full:
            request.message = "session command queue is full"
            request.completed.set()
            return False

    def _on_asr_final(self, message: String) -> None:
        active_request = self._active_request
        if (
            active_request is not None
            and active_request.command == SessionCommand.RUN_AUTOMATIC_MISSION
            and is_automatic_mission_cancel_text(message.data)
        ):
            active_request.canceled = True
            self.get_logger().warning("canceling active automatic mission by voice")
            return
        if is_truncated_automatic_mission_text(message.data):
            # 该别名来自真实 ZipFormer 尾部漏字样本。只对完全相等的“开始自动”
            # 生效，既让现场演示可恢复，也不会把“开始自动播放音乐”误判为建图。
            self.get_logger().warning(
                "ASR final truncated to '开始自动'; recovering the explicit "
                "automatic mapping mission intent"
            )
        command = parse_session_command(message.data)
        if command is None:
            return
        now = time.monotonic()
        with self._intent_lock:
            previous, timestamp = self._last_intent
            if previous == command and now - timestamp < 3.0:
                self.get_logger().info("ignored duplicate SLAM session voice command")
                return
            self._last_intent = command, now
        request = CommandRequest(command=command, source="voice")
        if self._enqueue(request):
            self.get_logger().info(f"queued voice session command={command.name.lower()}")
        else:
            self.get_logger().warning(
                f"rejected voice session command={command.name.lower()}: "
                f"{request.message or 'busy'}"
            )

    def _execute_action(self, goal_handle):
        request = CommandRequest(
            command=SessionCommand(goal_handle.request.command),
            source="action",
            goal_handle=goal_handle,
        )
        if not self._enqueue(request):
            goal_handle.abort()
            return self._action_result(False, request.message or "session busy")
        while rclpy.ok() and not request.completed.wait(timeout=0.1):
            if goal_handle.is_cancel_requested:
                request.canceled = True
        if request.canceled:
            goal_handle.canceled()
            return self._action_result(False, request.message or "canceled")
        if request.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        return self._action_result(request.success, request.message)

    def _action_result(self, success: bool, message: str):
        result = ManageSlamSession.Result()
        result.success = success
        result.message = message
        result.state = self._state_message()
        return result

    def feedback(self, request: CommandRequest, progress: float) -> None:
        if request.goal_handle is None:
            return
        feedback = ManageSlamSession.Feedback()
        feedback.progress = progress
        feedback.state = self._state_message()
        request.goal_handle.publish_feedback(feedback)

    def _start_mapping(self) -> None:
        generation = self._readiness_generation()
        self.transition(SessionPhase.STARTING_MAPPING, detail="starting mapping stage")
        self._manager.start("mapping")
        self._wait_for_new_ready(generation)
        self.transition(SessionPhase.MAPPING, detail="mapping ready; explore by voice")

    def save_map(self, request: CommandRequest) -> None:
        if self._fsm.snapshot.map_saved:
            request.message = "map already saved"
            return
        self.transition(SessionPhase.SAVING_MAP, detail="saving /map")
        self.feedback(request, 0.35)
        try:
            yaml_path = self._manager.save_map()
        except Exception as exc:
            # 保存失败不杀死仍可用的 mapping stage，允许用户修正后重新说“保存地图”。
            self.transition(
                SessionPhase.MAPPING,
                detail=f"map save failed: {exc}",
                map_saved=False,
                map_yaml_path="",
            )
            raise
        self.transition(
            SessionPhase.MAP_SAVED,
            detail="map saved; ready to start navigation",
            map_saved=True,
            map_yaml_path=yaml_path,
        )
        self.feedback(request, 0.55)

    def start_navigation(self, request: CommandRequest) -> None:
        if self._fsm.snapshot.phase == SessionPhase.NAVIGATING:
            request.message = "navigation already active"
            return
        self.transition(
            SessionPhase.SWITCHING_TO_NAVIGATION,
            detail="stopping mapping stage",
        )
        self.feedback(request, 0.65)
        self._manager.stop()
        with self._map_pose_condition:
            # mapping 进程停止后再开启新代际，并在 navigation 启动前清缓存；否则
            # transient-local 或迟到的 SLAM /map 会与旧 AMCL pose 拼成“新保存图”，
            # 让候选采样读到跨阶段快照。回调和 admission 都检查同一代际。
            self._navigation_input_generation += 1
            self._latest_occupancy = None
            self._latest_map_pose_xy = None
            self._latest_occupancy_generation = -1
            self._latest_map_pose_generation = -1
            self._map_pose_condition.notify_all()
        generation = self._readiness_generation()
        self.transition(
            SessionPhase.STARTING_NAVIGATION,
            detail="starting saved-map AMCL/Nav2 stage",
        )
        # 进程切换很快时 transient-local state topic 只保证新订阅者拿到“最新状态”，
        # 不保证测试或 UI 一定调度到每个中间快照；Action feedback 因此同步承载阶段进度。
        self.feedback(request, 0.8)
        self._manager.start("navigation")
        self._wait_for_new_ready(generation)
        self.transition(
            SessionPhase.NAVIGATING,
            detail="navigation ready; semantic goals accepted",
        )
        self.feedback(request, 1.0)

    def _execute_request(self, request: CommandRequest) -> None:
        # 入队与真正执行之间可能已完成上一条阶段转换；执行前再次验证，防止
        # 两个几乎同时到达的 ASR final 都基于旧 MAPPING 状态被接受。
        with self._state_lock:
            accepted, reason = self._fsm.validate(request.command)
        if not accepted:
            request.message = reason
            request.completed.set()
            return
        self._operation_active.set()
        self._active_request = request
        try:
            if request.command == SessionCommand.RUN_AUTOMATIC_MISSION:
                # 同一 orchestrator 可运行多次任务；显式 mission_sequence 能阻止
                # 上一轮 transient-local goal 快照被验收器误归到新任务。
                self._mission_sequence += 1
                self._mission_outcome = SlamSessionState.MISSION_RUNNING
                self._mission_message = "automatic mission running"
                self._navigation_goal_ledger.reset(self._mission_sequence)
                self._publish_state()
                self._automatic_mission_executor.run(request)
            if request.command in {
                SessionCommand.SAVE_MAP,
                SessionCommand.SAVE_AND_START_NAVIGATION,
            }:
                self.save_map(request)
            if request.canceled:
                request.message = "canceled before stage switch"
                return
            if request.command in {
                SessionCommand.START_NAVIGATION,
                SessionCommand.SAVE_AND_START_NAVIGATION,
            }:
                self.start_navigation(request)
            if request.command == SessionCommand.STOP_SESSION:
                self.transition(SessionPhase.STOPPING, detail="stopping session")
                self._manager.stop()
                self.transition(SessionPhase.STOPPED, detail="session stopped")
            request.success = True
            if not request.message:
                request.message = "session command completed"
        except AutomaticMissionCancelled as exc:
            request.message = str(exc)
            if request.command == SessionCommand.RUN_AUTOMATIC_MISSION:
                self._mission_outcome = SlamSessionState.MISSION_CANCELED
                self._mission_message = request.message
            recovery_phase = (
                SessionPhase.NAVIGATING
                if self._manager.stage == "navigation"
                else SessionPhase.MAPPING
            )
            self.transition(recovery_phase, detail=request.message)
            self.get_logger().warning(request.message)
        except Exception as exc:
            request.message = str(exc)
            if request.command == SessionCommand.RUN_AUTOMATIC_MISSION:
                self._mission_outcome = SlamSessionState.MISSION_FAILED
                self._mission_message = request.message
                recovery_phase = (
                    SessionPhase.NAVIGATING
                    if self._manager.stage == "navigation"
                    else SessionPhase.MAPPING
                )
                self.transition(
                    recovery_phase,
                    detail=f"automatic mission failed: {exc}",
                )
            elif self._fsm.snapshot.phase != SessionPhase.MAPPING:
                self.transition(SessionPhase.FAILED, detail=f"session failed: {exc}")
            self.get_logger().error(request.message)
        finally:
            self._active_request = None
            self._operation_active.clear()
            request.completed.set()

    def _worker_loop(self) -> None:
        try:
            self._start_mapping()
        except Exception as exc:
            self.transition(SessionPhase.FAILED, detail=f"mapping startup failed: {exc}")
        while not self._stopping.is_set():
            try:
                request = self._requests.get(timeout=0.2)
            except queue.Empty:
                continue
            if request is None:
                break
            self._execute_request(request)

    def _check_child_process(self) -> None:
        if self._operation_active.is_set() or self._stopping.is_set():
            return
        exited, code = self._manager.exited_unexpectedly()
        if exited and self._fsm.snapshot.phase not in {
            SessionPhase.STOPPED,
            SessionPhase.FAILED,
        }:
            self.transition(
                SessionPhase.FAILED,
                detail=f"{self._manager.stage or 'stage'} process exited code={code}",
            )

    def close(self) -> None:
        self._stopping.set()
        try:
            self._requests.put_nowait(None)
        except queue.Full:
            pass
        self._manager.stop()
        self._worker.join(timeout=3.0)
        self._action_server.destroy()


def main() -> None:
    rclpy.init()
    node = SessionOrchestratorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        executor.shutdown()
        node.destroy_node()
        # SIGINT/SIGTERM 可能已由 rclpy 的全局 signal handler 关闭 context；
        # 再次 shutdown 会让正常 Ctrl+C 退出以 RCLError 堆栈结束。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
