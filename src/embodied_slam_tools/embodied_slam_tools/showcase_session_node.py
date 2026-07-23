"""单终端编排真实感语音建图、保存地图和导航阶段。"""

from __future__ import annotations

from dataclasses import replace
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
    ControlAuthorityState,
    FrontierExplorationEvidence,
    RobotCommand,
    RobotCommandResult,
    SlamMappingCompletionEvidence,
    SlamNavigationGoalEvidence,
    SlamSessionState,
    SystemReadiness,
    WakeEvent,
)
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import BackUp, ComputePathToPose, FollowWaypoints, NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from explore_lite_msgs.msg import ExploreStatus
except ImportError:  # 可选运行时由 setup_frontier_exploration.sh 安装。
    ExploreStatus = None

from .agent_action_gateway import AgentActionGateway, AgentActionOutcome
from .autonomy_quiescence import (
    AutonomyQuiescenceBarrier,
    QuiescenceError,
    QuiescenceIdentity,
    QuiescenceState,
)
from .control_authority_lease import (
    AUTONOMY,
    AuthoritySnapshot,
    ControlAuthorityLease,
)
from .frontier_monitor import FrontierExplorationMonitor
from .exploration_saturation import (
    SaturationAssessment,
    SaturationRuntimeEvidence,
    consecutive_low_yield_epoch_count,
)
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
from .mapping_return import (
    PlanarPose,
    ReturnActionKind,
    ReturnActionStatus,
    ReturnToStartEvidence,
    ReturnToStartSpec,
    build_return_to_start_report,
    evaluate_return_to_start,
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

try:
    from embodied_agent_interfaces.srv import AcknowledgeAutonomyQuiescence
except ImportError:  # source-only 单测可通过依赖注入绕过尚未生成的 ROS 接口。
    AcknowledgeAutonomyQuiescence = None


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


def _yaw_from_quaternion(quaternion) -> float:
    """将 geometry_msgs Quaternion 转为平面 yaw。"""

    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
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


def _begin_nav2_quiescence_transaction(
    owner: Any,
    kind: str,
) -> str | None:
    """登记可能产生速度的 Nav2 事务；source-only fake 无屏障时保持兼容。"""

    barrier = getattr(owner, "_autonomy_quiescence", None)
    if barrier is None:
        return None
    state_lock = getattr(owner, "_state_lock", None)
    if state_lock is None:
        sequence = int(
            getattr(owner, "_nav2_quiescence_sequence", 0)
        ) + 1
        owner._nav2_quiescence_sequence = sequence
    else:
        with state_lock:
            sequence = owner._nav2_quiescence_sequence + 1
            owner._nav2_quiescence_sequence = sequence
    token = f"{kind}:{sequence}"
    barrier.nav2_goal_started(token)
    return token


def _finish_nav2_quiescence_transaction(
    owner: Any,
    token: str | None,
) -> None:
    if token is None:
        return
    barrier = getattr(owner, "_autonomy_quiescence", None)
    if barrier is not None:
        barrier.nav2_goal_terminal(token)


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
        self._command_input_source = str(
            self.declare_parameter(
                "command_input_source", "raw_asr"
            ).value
        ).strip().lower()
        if self._command_input_source not in {"raw_asr", "wake_event"}:
            raise ValueError(
                "command_input_source must be raw_asr or wake_event"
            )
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
            self._return_to_start_spec = (
                mission_configuration.automatic.return_to_start_spec
            )
            self._saturation_policy = (
                mission_configuration.automatic.saturation_policy
            )
            self._automatic_mission_executor = UnknownWorldMissionExecutor(
                self,
                self._manager,
                self._mapping_evidence,
                mission_configuration.automatic,
            )
        else:
            self._return_to_start_spec = ReturnToStartSpec()
            self._saturation_policy = None
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
        self._authority_gate_enabled = bool(
            self.declare_parameter("authority_gate_enabled", False).value
        )
        authority_state_timeout_s = float(
            self.declare_parameter(
                "authority_state_timeout_s", 1.0
            ).value
        )
        self._authority_lease = ControlAuthorityLease(
            authority_state_timeout_s
        )
        self._authority_lease_failure_reported = False
        self._autonomy_quiescence_timeout_s = float(
            self.declare_parameter(
                "autonomy_quiescence_timeout_s", 15.0
            ).value
        )
        if (
            not math.isfinite(self._autonomy_quiescence_timeout_s)
            or self._autonomy_quiescence_timeout_s <= 0.0
        ):
            raise ValueError(
                "autonomy_quiescence_timeout_s must be greater than zero"
            )
        self._autonomy_quiescence = AutonomyQuiescenceBarrier()
        self._quiescence_worker_lock = threading.Lock()
        self._quiescence_worker: threading.Thread | None = None
        self._nav2_quiescence_sequence = 0
        self._map_pose_condition = threading.Condition()
        self._latest_occupancy: OccupancySnapshot | None = None
        self._latest_map_pose_xy: tuple[float, float] | None = None
        self._navigation_input_generation = 0
        self._latest_occupancy_generation = -1
        self._latest_map_pose_generation = -1
        self._mission_sequence = 0
        self._mission_outcome = SlamSessionState.MISSION_IDLE
        self._mission_message = ""
        self._navigation_startup_failure_pending = False
        self._internal_command_sequence = 0
        self._navigation_goal_ledger = NavigationGoalLedger()
        self._mapping_saturation_evidence: SaturationRuntimeEvidence | None = None
        self._mapping_saturation_assessment: SaturationAssessment | None = None
        self._return_to_start_evidence: ReturnToStartEvidence | None = None
        self._cmd_vel_condition = threading.Condition()
        self._cmd_vel_generation = 0
        self._last_cmd_vel = (0.0, 0.0)
        self._last_cmd_vel_observed_at_ns = 0

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
        self._tf_buffer = Buffer(cache_time=Duration(seconds=30.0))
        # 节点已经运行在 MultiThreadedExecutor；不另起 TF spin 线程，避免同一
        # subscription 被两个 executor 竞争并放大 WSL 内存占用。
        self._tf_listener = TransformListener(
            self._tf_buffer,
            self,
            spin_thread=False,
        )
        self._state_pub = self.create_publisher(
            SlamSessionState, "/slam/session_state", state_qos
        )
        self._agent_text_pub = self.create_publisher(
            String, "/agent/text_input", event_qos
        )
        self._internal_action_pub = self.create_publisher(
            RobotCommand, "/agent/action_candidate", event_qos
        )
        self._explore_control_pub = self.create_publisher(
            Bool, "/explore/resume", event_qos
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
        # Agent 会同时发布裸 ASR 与经过会话门控的 WakeEvent。节点只订阅选定
        # 的一种来源，防止同一句话形成两个任务；raw_asr 默认值保留旧验收兼容性。
        if self._command_input_source == "raw_asr":
            self.create_subscription(
                String,
                "/agent/asr_final",
                self._on_asr_final,
                event_qos,
                callback_group=callback_group,
            )
        else:
            self.create_subscription(
                WakeEvent,
                "/agent/wake_event",
                self._on_wake_event,
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
            ControlAuthorityState,
            "/control/authority/state",
            self._on_control_authority_state,
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
            Twist,
            "/cmd_vel",
            self._on_cmd_vel,
            event_qos,
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
        self._backup_client = ActionClient(
            self,
            BackUp,
            "/backup",
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
        self._quiescence_ack_client = None
        if self._authority_gate_enabled:
            if AcknowledgeAutonomyQuiescence is None:
                raise RuntimeError(
                    "authority gate requires generated "
                    "AcknowledgeAutonomyQuiescence service"
                )
            self._quiescence_ack_client = self.create_client(
                AcknowledgeAutonomyQuiescence,
                "/control/acknowledge_autonomy_quiescence",
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
        # manager 心跳失联不会产生新的 ROS 回调；独立 steady-clock 定时检查可在
        # 租约过期时主动取消正在运行的 Explore/Nav2 事务。
        self.create_timer(0.2, self._check_authority_lease)
        self._publish_state()
        self._worker.start()

    def _mapping_completion_message(self) -> SlamMappingCompletionEvidence:
        message = SlamMappingCompletionEvidence()
        message.stamp = self.get_clock().now().to_msg()
        saturation = self._mapping_saturation_evidence
        assessment = self._mapping_saturation_assessment
        return_evidence = self._return_to_start_evidence
        message.trigger_reason = self._mapping_evidence.completion_reason
        message.mode = (
            SlamMappingCompletionEvidence.MODE_BOUNDED_SATURATION
            if saturation is not None
            else SlamMappingCompletionEvidence.MODE_STRICT_FRONTIER
            if return_evidence is not None
            else SlamMappingCompletionEvidence.MODE_UNSPECIFIED
        )
        if saturation is not None:
            saturation_policy = self._saturation_policy
            if saturation_policy is None:
                raise RuntimeError(
                    "saturation evidence exists outside unknown-world profile"
                )
            message.low_yield_epoch_count = consecutive_low_yield_epoch_count(
                saturation.history,
                saturation_policy,
            )
            message.required_low_yield_epoch_count = (
                saturation_policy.minimum_low_yield_epochs
            )
            message.residual_available_frontiers = (
                saturation.residual_available_frontiers
            )
            message.ledger_drained = (
                saturation.active_goal_count == 0
                and saturation.pending_goal_count == 0
            )
            if saturation.history.final_probe is not None:
                message.final_probe_gain_cells = (
                    saturation.history.final_probe.map_gain_cells
                )
                message.final_probe_gain_ratio = (
                    saturation.history.final_probe.map_gain_ratio
                )
            message.typed_stop_succeeded = saturation.typed_stop_confirmed

        return_report = (
            build_return_to_start_report(
                return_evidence,
                self._return_to_start_spec,
            )
            if return_evidence is not None
            else None
        )
        checks = (
            dict(return_report.get("checks", {}))
            if return_report is not None
            else {}
        )
        message.return_home_valid = bool(
            return_report and return_report.get("passed") is True
        )
        message.return_typed_action_command = bool(
            checks.get("typed_action_command")
        )
        message.return_typed_action_succeeded = bool(
            checks.get("typed_action_succeeded")
        )
        message.return_before_map_save = bool(
            checks.get("return_before_map_save")
        )
        message.return_same_pose_frame = bool(checks.get("same_pose_frame"))
        message.return_final_pose_in_window = bool(
            checks.get("final_pose_in_return_window")
        )
        message.return_xy_within_tolerance = bool(
            checks.get("xy_within_tolerance")
        )
        message.return_yaw_within_tolerance = bool(
            checks.get("yaw_within_tolerance")
        )
        message.return_cmd_vel_after_return = bool(
            checks.get("cmd_vel_after_return")
        )
        message.return_final_cmd_vel_fresh = bool(
            checks.get("final_cmd_vel_fresh")
        )
        message.return_final_cmd_vel_zero = bool(
            checks.get("final_cmd_vel_zero")
        )
        if return_report is not None:
            errors = return_report.get("errors", {})
            thresholds = return_report.get("thresholds", {})
            message.return_xy_error_m = float(errors.get("xy_error_m") or 0.0)
            message.return_yaw_error_rad = float(
                errors.get("yaw_error_rad") or 0.0
            )
            message.return_max_xy_error_m = float(
                thresholds.get("max_xy_error_m", 0.0)
            )
            message.return_max_yaw_error_rad = float(
                thresholds.get("max_yaw_error_rad", 0.0)
            )
            message.return_zero_velocity_tolerance = float(
                thresholds.get("zero_velocity_tolerance", 0.0)
            )
            message.return_max_cmd_vel_age_s = float(
                thresholds.get("max_cmd_vel_age_s", 0.0)
            )
        message.valid = bool(
            message.return_home_valid
            and (
                saturation is None
                or assessment is not None
                and assessment.complete
            )
        )
        message.detail = (
            "mapping completion evidence valid"
            if message.valid
            else "mapping completion evidence pending or invalid"
        )
        return message

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
        message.mapping_completion = self._mapping_completion_message()
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
        command_id = str(message.command_id)
        self._agent_action_gateway.record_result(
            command_id,
            AgentActionOutcome(
                success=bool(message.success),
                status=int(message.status),
                message=str(message.message),
            ),
        )
        # 屏障只接受自己发布的 exact command_id；普通动作与旧 STOP 的结果
        # 会在深模块中被忽略，不能误完成本轮控制权撤销。
        self._autonomy_quiescence.observe_priority_stop_result(
            command_id,
            success=bool(message.success),
            detail=str(message.message),
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

    def _on_cmd_vel(self, message: Twist) -> None:
        """记录真实控制输出；初始默认零值不能充当停车证据。"""

        with self._cmd_vel_condition:
            self._cmd_vel_generation += 1
            generation = self._cmd_vel_generation
            self._last_cmd_vel = (
                float(message.linear.x),
                float(message.angular.z),
            )
            self._last_cmd_vel_observed_at_ns = (
                self.get_clock().now().nanoseconds
            )
            self._cmd_vel_condition.notify_all()
        self._autonomy_quiescence.observe_cmd_vel(
            generation=generation,
            linear_x=float(message.linear.x),
            angular_z=float(message.angular.z),
        )

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
        telemetry = _frontier_telemetry_from_message(message)
        self._mapping_evidence.record_frontier_telemetry(telemetry)
        self._autonomy_quiescence.observe_frontier(
            status=telemetry.status,
            active_goal_count=telemetry.active_goal_count,
            accepted_goal_count=telemetry.accepted_goal_count,
            terminal_goal_count=(
                telemetry.succeeded_goal_count
                + telemetry.aborted_goal_count
                + telemetry.canceled_goal_count
            ),
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
        if self._autonomy_quiescence.active:
            # 接管事务中只能先走 Explore 的 typed pause/ledger terminal；
            # 直接杀进程会丢掉 owner goal 的 result callback，无法形成 ACK 证据。
            self._explore_control_pub.publish(Bool(data=False))
            return
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
        """旁路文本意图层停车，并等待 typed Action 终态与新鲜零速度。"""

        deadline = time.monotonic() + timeout_s
        cmd_vel_condition = getattr(self, "_cmd_vel_condition", None)
        if cmd_vel_condition is not None:
            with cmd_vel_condition:
                cmd_vel_generation = self._cmd_vel_generation
        else:  # 只用于不构造完整 ROS Node 的纯单元 fake。
            cmd_vel_generation = -1

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
        if cmd_vel_condition is None:
            return
        with cmd_vel_condition:
            while True:
                linear_x, angular_z = self._last_cmd_vel
                if (
                    self._cmd_vel_generation > cmd_vel_generation
                    and abs(linear_x) <= 1.0e-3
                    and abs(angular_z) <= 1.0e-3
                ):
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        "typed STOP completed without a fresh zero /cmd_vel"
                    )
                cmd_vel_condition.wait(timeout=min(0.1, remaining))

    def run_recovery_backup(
        self,
        request: CommandRequest,
        *,
        distance_m: float,
        speed_mps: float,
        timeout_s: float,
    ) -> float:
        """执行碰撞检查后退，并返回独立 `/odom` 净位移证据。"""

        if self._dry_run:
            return float(distance_m)
        deadline = time.monotonic() + timeout_s
        start = self._wait_for_recovery_odom(
            request,
            after_generation=-1,
            deadline_monotonic=deadline,
        )
        self._execute_nav2_backup(
            request,
            distance_m=distance_m,
            speed_mps=speed_mps,
            time_allowance_s=timeout_s,
            deadline_monotonic=deadline,
        )
        end = self._wait_for_recovery_odom(
            request,
            after_generation=start.odom_generation,
            deadline_monotonic=deadline,
        )
        assert start.latest_odom_xy is not None
        assert end.latest_odom_xy is not None
        displacement_m = math.hypot(
            end.latest_odom_xy[0] - start.latest_odom_xy[0],
            end.latest_odom_xy[1] - start.latest_odom_xy[1],
        )
        self.get_logger().info(
            "Nav2 recovery BackUp completed: "
            f"requested={distance_m:.3f}m odom={displacement_m:.3f}m"
        )
        return displacement_m

    def _wait_for_recovery_odom(
        self,
        request: CommandRequest,
        *,
        after_generation: int,
        deadline_monotonic: float,
    ):
        """等待恢复动作前后的不同里程计样本，拒绝复用陈旧位姿。"""

        with self._mapping_evidence.condition:
            while True:
                if request.canceled:
                    raise AutomaticMissionCancelled(
                        "automatic mission canceled during recovery backup"
                    )
                snapshot = self._mapping_evidence.snapshot()
                if (
                    snapshot.latest_odom_xy is not None
                    and snapshot.odom_generation > after_generation
                ):
                    return snapshot
                remaining = deadline_monotonic - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        "fresh odometry unavailable for recovery backup"
                    )
                self._mapping_evidence.condition.wait(
                    timeout=min(0.1, remaining)
                )

    def _execute_nav2_backup(
        self,
        request: CommandRequest,
        *,
        distance_m: float,
        speed_mps: float,
        time_allowance_s: float,
        deadline_monotonic: float,
    ) -> None:
        """执行 Nav2 BackUp Action；失败分支必须形成停车终态。"""

        values = (distance_m, speed_mps, time_allowance_s)
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("Nav2 backup values must be finite and positive")
        remaining = deadline_monotonic - time.monotonic()
        if remaining <= 0.0 or not self._backup_client.wait_for_server(
            timeout_sec=min(5.0, max(0.0, remaining))
        ):
            raise TimeoutError("Nav2 BackUp Action server unavailable")

        goal = BackUp.Goal()
        # Jazzy BackUp 客户端传正的距离/速度幅值；Behavior Server 内部转换
        # 为机器人负 X 运动，并使用 local costmap 在每个周期预测碰撞。
        goal.target.x = float(distance_m)
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = float(speed_mps)
        whole_seconds = int(time_allowance_s)
        nanoseconds = int(round((time_allowance_s - whole_seconds) * 1e9))
        if nanoseconds >= 1_000_000_000:
            whole_seconds += 1
            nanoseconds -= 1_000_000_000
        goal.time_allowance.sec = whole_seconds
        goal.time_allowance.nanosec = nanoseconds

        quiescence_token = _begin_nav2_quiescence_transaction(
            self, "backup"
        )
        try:
            response = self._backup_client.send_goal_async(goal)
        except Exception:
            _finish_nav2_quiescence_transaction(
                self, quiescence_token
            )
            raise
        while not response.done():
            if request.canceled or time.monotonic() >= deadline_monotonic:
                reason = (
                    "automatic mission canceled during recovery backup"
                    if request.canceled
                    else "Nav2 BackUp goal response timeout"
                )
                self._resolve_pending_nav2_goal_safely(
                    request,
                    response=response,
                    reason=reason,
                    quiescence_token=quiescence_token,
                )
                if request.canceled:
                    raise AutomaticMissionCancelled(reason)
                raise TimeoutError(reason)
            time.sleep(0.05)
        handle = response.result()
        if handle is None or not handle.accepted:
            _finish_nav2_quiescence_transaction(
                self, quiescence_token
            )
            raise RuntimeError("Nav2 BackUp goal rejected")
        try:
            result_future = handle.get_result_async()
        except Exception as exc:
            reason = f"cannot get Nav2 BackUp result future: {exc}"
            self._cancel_nav2_goal_and_force_stop(
                request,
                handle=handle,
                result_future=None,
                reason=reason,
                quiescence_token=quiescence_token,
            )
            raise RuntimeError(reason)
        while not result_future.done():
            if request.canceled or time.monotonic() >= deadline_monotonic:
                reason = (
                    "automatic mission canceled during recovery backup"
                    if request.canceled
                    else "Nav2 BackUp result timeout"
                )
                self._cancel_nav2_goal_and_force_stop(
                    request,
                    handle=handle,
                    result_future=result_future,
                    reason=reason,
                    quiescence_token=quiescence_token,
                )
                if request.canceled:
                    raise AutomaticMissionCancelled(reason)
                raise TimeoutError(reason)
            time.sleep(0.05)
        wrapped, terminal_error = _read_nav2_terminal_wrapper(result_future)
        if terminal_error is not None:
            self._cancel_nav2_goal_and_force_stop(
                request,
                handle=handle,
                result_future=result_future,
                reason=(
                    "invalid Nav2 BackUp terminal result: "
                    + terminal_error
                ),
                quiescence_token=quiescence_token,
            )
            raise RuntimeError(f"invalid Nav2 BackUp terminal result: {terminal_error}")
        _finish_nav2_quiescence_transaction(
            self, quiescence_token
        )
        assert wrapped is not None
        error_code = int(getattr(wrapped.result, "error_code", BackUp.Result.UNKNOWN))
        if (
            int(wrapped.status) != GoalStatus.STATUS_SUCCEEDED
            or error_code != BackUp.Result.NONE
        ):
            # BackUp behavior 已终止，但仍补一个 typed priority STOP，避免插件
            # 异常终态后最后一帧速度残留；失败不会回退到裸负速度控制。
            self.stop_motion_and_wait(
                _navigation_safety_stop_request(),
                timeout_s=_NAV2_SAFETY_STOP_TIMEOUT_S,
            )
            error_message = str(
                getattr(wrapped.result, "error_msg", "")
            ).strip()
            raise RuntimeError(
                "Nav2 BackUp failed: "
                f"status={wrapped.status} error={error_code} "
                f"message={error_message or 'unknown'}"
            )

    def capture_mapping_start_pose(
        self,
        request: CommandRequest,
        *,
        timeout_s: float,
    ) -> PlanarPose:
        """在首次运动前动态捕获 map-frame 起点，不接受 YAML 固定坐标。"""

        return self._lookup_mapping_pose(request, timeout_s=timeout_s)

    def _lookup_mapping_pose(
        self,
        request: CommandRequest,
        *,
        timeout_s: float,
    ) -> PlanarPose:
        """读取一帧实时 map->base_link；起点与返航终点共用同一坐标口径。"""

        if self._dry_run:
            return PlanarPose(
                0.0,
                0.0,
                0.0,
                "map",
                self.get_clock().now().nanoseconds,
            )
        deadline = time.monotonic() + timeout_s
        last_error = "transform unavailable"
        while time.monotonic() < deadline:
            if request.canceled:
                raise AutomaticMissionCancelled("automatic mission canceled")
            try:
                transform = self._tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    Time(),
                    timeout=Duration(seconds=0.2),
                )
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                return PlanarPose(
                    x=float(translation.x),
                    y=float(translation.y),
                    yaw=_yaw_from_quaternion(rotation),
                    frame_id="map",
                    # 统一使用本节点 ROS 时钟，确保后续 Action/存图时间线可比较。
                    observed_at_ns=self.get_clock().now().nanoseconds,
                )
            except TransformException as exc:
                last_error = str(exc)
                time.sleep(0.05)
        raise TimeoutError(
            "cannot capture mapping start pose from map->base_link: "
            + last_error
        )

    def quiesce_frontier(
        self,
        request: CommandRequest,
        *,
        timeout_s: float,
    ) -> FrontierTelemetry:
        """优雅暂停 Explore Lite，并等待其 NavigateToPose 总账完全结算。"""

        del request  # 安全收口不能因上层取消位而跳过 active UUID 排空。
        if self._dry_run:
            return self._mapping_evidence.snapshot().frontier_telemetry
        deadline = time.monotonic() + timeout_s
        next_publish_at = 0.0
        with self._mapping_evidence.condition:
            while True:
                now = time.monotonic()
                if now >= next_publish_at:
                    # Explore Lite 已有 /explore/resume Bool 控制面；false 会停止
                    # 派发、取消 owner goal，并在 result callback 后发布 active=0。
                    self._explore_control_pub.publish(Bool(data=False))
                    next_publish_at = now + 0.5
                telemetry = (
                    self._mapping_evidence.snapshot().frontier_telemetry
                )
                terminal = (
                    telemetry.succeeded_goal_count
                    + telemetry.aborted_goal_count
                    + telemetry.canceled_goal_count
                )
                if (
                    telemetry.status == "exploration_paused"
                    and telemetry.active_goal_count == 0
                    and telemetry.accepted_goal_count == terminal
                ):
                    return telemetry
                remaining = deadline - now
                if remaining <= 0.0:
                    raise TimeoutError(
                        "frontier quiesce did not drain Action ledger: "
                        f"status={telemetry.status} "
                        f"active={telemetry.active_goal_count} "
                        f"accepted={telemetry.accepted_goal_count} "
                        f"terminal={terminal}"
                    )
                self._mapping_evidence.condition.wait(
                    timeout=min(0.1, remaining)
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

    def run_mapping_return_goal(
        self,
        request: CommandRequest,
        *,
        start_pose: PlanarPose,
        spec: ReturnToStartSpec,
        timeout_s: float,
    ) -> ReturnToStartEvidence:
        """在 SLAM stage 内返航；Action 成功后仍用 TF 与零速度二次验证。"""

        command_id = f"slam-return-home-{self._mission_sequence}"
        started_at_ns = self.get_clock().now().nanoseconds
        if self._dry_run:
            finished_at_ns = max(started_at_ns + 1, 2)
            return ReturnToStartEvidence(
                start_pose=start_pose,
                final_pose=replace(
                    start_pose,
                    observed_at_ns=finished_at_ns,
                ),
                action_kind=ReturnActionKind.NAVIGATE_TO_POSE,
                action_status=ReturnActionStatus.SUCCEEDED,
                action_command_id=command_id,
                action_started_at_ns=started_at_ns,
                action_finished_at_ns=finished_at_ns,
                map_saved_at_ns=0,
                cmd_vel_linear_x=0.0,
                cmd_vel_angular_z=0.0,
                cmd_vel_observed_at_ns=finished_at_ns,
                evaluated_at_ns=finished_at_ns,
            )

        deadline = time.monotonic() + timeout_s
        # ComputePathToPose 是执行前安全准入；它证明返航不穿越本次图中的未知区。
        self._request_preflight_path(
            request,
            goal_xy=(start_pose.x, start_pose.y),
            deadline=deadline,
        )
        wrapped = self._execute_untracked_nav2_goal(
            request,
            goal_pose=start_pose,
            deadline=deadline,
        )
        result = getattr(wrapped, "result", None)
        error_code = int(
            getattr(
                result,
                "error_code",
                getattr(NavigateToPose.Result, "UNKNOWN", -1),
            )
        )
        if (
            int(wrapped.status) != GoalStatus.STATUS_SUCCEEDED
            or result is None
            or error_code != NavigateToPose.Result.NONE
        ):
            raise RuntimeError(
                "mapping return NavigateToPose failed: "
                f"status={wrapped.status} error={error_code}"
            )
        finished_at_ns = self.get_clock().now().nanoseconds

        final_pose: PlanarPose | None = None
        consecutive_matches = 0
        while time.monotonic() < deadline:
            pose = self._lookup_mapping_pose(
                request,
                timeout_s=min(0.5, max(0.05, deadline - time.monotonic())),
            )
            xy_error = math.hypot(pose.x - start_pose.x, pose.y - start_pose.y)
            yaw_error = abs(
                math.atan2(
                    math.sin(pose.yaw - start_pose.yaw),
                    math.cos(pose.yaw - start_pose.yaw),
                )
            )
            if (
                xy_error <= spec.max_xy_error_m
                and yaw_error <= spec.max_yaw_error_rad
            ):
                consecutive_matches += 1
                final_pose = pose
                if consecutive_matches >= 3:
                    break
            else:
                consecutive_matches = 0
            time.sleep(0.05)
        if final_pose is None or consecutive_matches < 3:
            self.stop_motion_and_wait(
                _navigation_safety_stop_request(),
                timeout_s=_NAV2_SAFETY_STOP_TIMEOUT_S,
            )
            raise TimeoutError(
                "mapping return Action succeeded but TF did not enter "
                "return-to-start tolerance"
            )

        self.stop_motion_and_wait(
            _navigation_safety_stop_request(),
            timeout_s=min(
                _NAV2_SAFETY_STOP_TIMEOUT_S,
                max(0.1, deadline - time.monotonic()),
            ),
        )
        with self._cmd_vel_condition:
            linear_x, angular_z = self._last_cmd_vel
            cmd_vel_observed_at_ns = self._last_cmd_vel_observed_at_ns
        evaluated_at_ns = self.get_clock().now().nanoseconds
        return ReturnToStartEvidence(
            start_pose=start_pose,
            final_pose=final_pose,
            action_kind=ReturnActionKind.NAVIGATE_TO_POSE,
            action_status=ReturnActionStatus.SUCCEEDED,
            action_command_id=command_id,
            action_started_at_ns=started_at_ns,
            action_finished_at_ns=finished_at_ns,
            map_saved_at_ns=0,
            cmd_vel_linear_x=linear_x,
            cmd_vel_angular_z=angular_z,
            cmd_vel_observed_at_ns=cmd_vel_observed_at_ns,
            evaluated_at_ns=evaluated_at_ns,
        )

    def _execute_untracked_nav2_goal(
        self,
        request: CommandRequest,
        *,
        goal_pose: PlanarPose,
        deadline: float,
    ):
        """执行不进入 sampled-goal 账本的返航目标，复用统一故障收口。"""

        remaining = max(0.0, deadline - time.monotonic())
        if not self._navigate_to_pose_client.wait_for_server(
            timeout_sec=min(10.0, remaining)
        ):
            raise TimeoutError("NavigateToPose Action server unavailable for return")
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = goal_pose.frame_id
        goal.pose.pose.position.x = goal_pose.x
        goal.pose.pose.position.y = goal_pose.y
        goal.pose.pose.orientation.z = math.sin(goal_pose.yaw * 0.5)
        goal.pose.pose.orientation.w = math.cos(goal_pose.yaw * 0.5)
        quiescence_token = _begin_nav2_quiescence_transaction(
            self, "mapping-return"
        )
        try:
            response = self._navigate_to_pose_client.send_goal_async(goal)
        except Exception:
            _finish_nav2_quiescence_transaction(
                self, quiescence_token
            )
            raise
        while not response.done():
            if request.canceled or time.monotonic() >= deadline:
                reason = (
                    "automatic mission canceled before return goal response"
                    if request.canceled
                    else "mapping return goal response timeout"
                )
                self._resolve_pending_nav2_goal_safely(
                    request,
                    response=response,
                    reason=reason,
                    quiescence_token=quiescence_token,
                )
                if request.canceled:
                    raise AutomaticMissionCancelled(reason)
                raise TimeoutError(reason)
            time.sleep(0.05)
        handle = response.result()
        if handle is None or not handle.accepted:
            _finish_nav2_quiescence_transaction(
                self, quiescence_token
            )
            raise _NavigationGoalRejected("mapping return goal rejected")
        result_future = handle.get_result_async()
        while not result_future.done():
            if request.canceled or time.monotonic() >= deadline:
                reason = (
                    "automatic mission canceled during mapping return"
                    if request.canceled
                    else "mapping return goal result timeout"
                )
                self._cancel_nav2_goal_and_force_stop(
                    request,
                    handle=handle,
                    result_future=result_future,
                    reason=reason,
                    quiescence_token=quiescence_token,
                )
                if request.canceled:
                    raise AutomaticMissionCancelled(reason)
                raise TimeoutError(reason)
            time.sleep(0.05)
        wrapped, terminal_error = _read_nav2_terminal_wrapper(result_future)
        if terminal_error is not None:
            self._cancel_nav2_goal_and_force_stop(
                request,
                handle=handle,
                result_future=result_future,
                reason="invalid mapping return terminal: " + terminal_error,
                quiescence_token=quiescence_token,
            )
            raise RuntimeError(
                "invalid mapping return terminal: " + terminal_error
            )
        assert wrapped is not None
        _finish_nav2_quiescence_transaction(
            self, quiescence_token
        )
        return wrapped

    def record_mapping_completion(
        self,
        *,
        completion_reason: str,
        saturation_evidence: SaturationRuntimeEvidence | None,
        saturation_assessment: SaturationAssessment | None,
        return_to_start: ReturnToStartEvidence,
    ) -> None:
        """缓存 typed 建图收口证据；save_map 会补上真实保存时间。"""

        del completion_reason  # reason 已由 MappingEvidenceTracker 进入 frontier msg。
        self._mapping_saturation_evidence = saturation_evidence
        self._mapping_saturation_assessment = saturation_assessment
        self._return_to_start_evidence = return_to_start
        self._publish_state()

    def _reset_mapping_completion_evidence(self) -> None:
        """开始新 mission 时丢弃上一轮的瞬态收口证据。"""

        self._mapping_saturation_evidence = None
        self._mapping_saturation_assessment = None
        self._return_to_start_evidence = None

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
        quiescence_token = _begin_nav2_quiescence_transaction(
            self, f"sampled-{sequence}"
        )
        try:
            response = self._navigate_to_pose_client.send_goal_async(goal)
        except Exception:
            _finish_nav2_quiescence_transaction(
                self, quiescence_token
            )
            raise
        while not response.done():
            if request.canceled:
                reason = "automatic mission canceled before goal response"
                # 不能只注册 late callback 后释放任务锁：回调与下一个任务会竞态。
                # 在独立安全预算内同步取得迟到 handle，并完成 cancel/STOP/终态。
                self._resolve_pending_nav2_goal_safely(
                    request,
                    response=response,
                    reason=reason,
                    quiescence_token=quiescence_token,
                )
                raise AutomaticMissionCancelled("automatic mission canceled")
            if time.monotonic() >= deadline:
                reason = f"navigation goal response timeout: {goal_xy}"
                self._resolve_pending_nav2_goal_safely(
                    request,
                    response=response,
                    reason=reason,
                    quiescence_token=quiescence_token,
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
            _finish_nav2_quiescence_transaction(
                self, quiescence_token
            )
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
                quiescence_token=quiescence_token,
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
                    quiescence_token=quiescence_token,
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
                    quiescence_token=quiescence_token,
                )
                raise RuntimeError(reason)
            if time.monotonic() >= deadline:
                reason = f"navigation goal result timeout: {goal_xy}"
                self._cancel_nav2_goal_and_force_stop(
                    request,
                    handle=handle,
                    result_future=result_future,
                    reason=reason,
                    quiescence_token=quiescence_token,
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
                quiescence_token=quiescence_token,
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
                quiescence_token=quiescence_token,
            )
            raise RuntimeError(reason)
        _finish_nav2_quiescence_transaction(
            self, quiescence_token
        )
        return wrapped

    def _resolve_pending_nav2_goal_safely(
        self,
        request: CommandRequest,
        *,
        response,
        reason: str,
        timeout_s: float = _NAV2_SAFETY_STOP_TIMEOUT_S,
        quiescence_token: str | None = None,
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
            _finish_nav2_quiescence_transaction(
                self, quiescence_token
            )
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
            quiescence_token=quiescence_token,
        )

    def _cancel_nav2_goal_and_force_stop(
        self,
        request: CommandRequest,
        *,
        handle,
        result_future,
        reason: str,
        timeout_s: float = _NAV2_SAFETY_STOP_TIMEOUT_S,
        quiescence_token: str | None = None,
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
        else:
            _finish_nav2_quiescence_transaction(
                self, quiescence_token
            )
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
        try:
            self._wait_navigation_ready_impl(request)
        except AutomaticMissionCancelled:
            # 用户主动取消是正常控制流，外层会恢复到仍存活的 stage；不能把它
            # 误标为导航启动故障，也不能在这里重复停止进程。
            raise
        except Exception as exc:
            self._cleanup_failed_navigation_startup(exc)
            raise

    def _wait_navigation_ready_impl(self, request: CommandRequest) -> None:
        """等待 Nav2 运行依赖；失败关闭由公开入口统一处理。"""

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

    def _cleanup_failed_navigation_startup(self, primary_error: Exception) -> None:
        """回收半启动导航 stage，同时保留最先暴露的根因。"""

        # stage 启动失败或 readiness 超时后，进程是否完整存活已经不可证明。
        # 必须 fail-close；若 stop 也失败，只把它作为异常附注，不能覆盖首因。
        self._navigation_startup_failure_pending = True
        try:
            self._manager.stop()
        except Exception as cleanup_error:
            note = f"navigation startup cleanup failed: {cleanup_error}"
            add_note = getattr(primary_error, "add_note", None)
            if callable(add_note):
                add_note(note)
            self.get_logger().error(note)

    def _readiness_generation(self) -> int:
        with self._ready_condition:
            return self._ready_generation

    def _goal_callback(self, goal_request) -> GoalResponse:
        try:
            command = SessionCommand(goal_request.command)
        except ValueError:
            return GoalResponse.REJECT
        barrier = getattr(self, "_autonomy_quiescence", None)
        if barrier is not None and barrier.active:
            return GoalResponse.REJECT
        with self._state_lock:
            accepted, _ = self._fsm.validate(command)
            if (
                accepted
                and getattr(self, "_authority_gate_enabled", False)
                and command == SessionCommand.RUN_AUTOMATIC_MISSION
                and not self._authority_lease.fresh_autonomy(
                    time.monotonic()
                )
            ):
                accepted = False
        if not accepted or self._operation_active.is_set():
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    @staticmethod
    def _cancel_callback(_goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def _enqueue(self, request: CommandRequest) -> bool:
        barrier = getattr(self, "_autonomy_quiescence", None)
        if barrier is not None and barrier.active:
            # SAVE_MAP/START_NAV 同样会启停 stage，必须等本轮 Explore/Nav2
            # terminal ACK 后才能进入队列；只门控自动任务仍会破坏静默证据。
            request.message = "autonomy quiescence is still in progress"
            request.completed.set()
            return False
        if self._operation_active.is_set():
            return False
        with self._state_lock:
            if (
                getattr(self, "_authority_gate_enabled", False)
                and request.command == SessionCommand.RUN_AUTOMATIC_MISSION
                and not self._authority_lease.fresh_autonomy(
                    time.monotonic()
                )
            ):
                # 高层 SLAM 会话不会经过 ActionGuard；必须在任务入口再次检查
                # 控制权租约，否则 HOLD、manager 失联或迟到状态仍可能启动子进程。
                request.message = (
                    "automatic mission requires fresh AUTONOMY authority"
                )
                request.completed.set()
                return False
            if (
                getattr(self, "_authority_gate_enabled", False)
                and request.command == SessionCommand.RUN_AUTOMATIC_MISSION
            ):
                # admission 与 worker 启动之间仍可能切权；把本地安全代际绑定到
                # request，worker 必须再次匹配后才能创建 Explore/Nav2 goal。
                request.authority_generation = (
                    self._authority_lease.generation
                )
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
        # strict live-voice 验收必须证明“通过唤醒门控的命令”触发任务。
        # wake_event 模式若继续消费裸 ASR，会让 filler 或未唤醒语音绕过门控，
        # 从而产生语义因果上的假阳性；默认 raw_asr 仍兼容旧演示与合成探针。
        if getattr(self, "_command_input_source", "raw_asr") != "raw_asr":
            return
        SessionOrchestratorNode._handle_session_command_text(self, message.data)

    def _on_wake_event(self, message: WakeEvent) -> None:
        if getattr(self, "_command_input_source", "raw_asr") != "wake_event":
            return
        if (
            message.kind not in {WakeEvent.KIND_WAKE, WakeEvent.KIND_CONTINUE}
            or not message.command_known
            or not message.command.strip()
        ):
            return
        SessionOrchestratorNode._handle_session_command_text(
            self, message.command
        )

    def _handle_session_command_text(self, text: str) -> None:
        if is_automatic_mission_cancel_text(text) and (
            SessionOrchestratorNode._cancel_active_automatic_mission(
                self, "voice_cancel"
            )
        ):
            return
        if is_truncated_automatic_mission_text(text):
            # 该别名来自真实 ZipFormer 尾部漏字样本。只对完全相等的“开始自动”
            # 生效，既让现场演示可恢复，也不会把“开始自动播放音乐”误判为建图。
            self.get_logger().warning(
                "ASR final truncated to '开始自动'; recovering the explicit "
                "automatic mapping mission intent"
            )
        command = parse_session_command(text)
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

    def _cancel_active_automatic_mission(
        self,
        reason: str,
        revocation_identity: QuiescenceIdentity | None = None,
    ) -> bool:
        """把人工接管转换为现有 mission cancel，不在 ROS 回调中阻塞停进程。"""

        with self._state_lock:
            request = self._active_request
            if (
                request is None
                or request.command
                != SessionCommand.RUN_AUTOMATIC_MISSION
                or request.canceled
            ):
                return False
            request.canceled = True
            if revocation_identity is not None:
                request.authority_revocation_identity = (
                    revocation_identity.manager_epoch,
                    revocation_identity.revocation_sequence,
                )
        # Explore Lite 的取消和 Nav2 terminal 等待仍由 worker 中现有清理路径完成。
        # 回调只发布边沿，避免在 MultiThreadedExecutor 中并发调用非线程安全 manager。
        self._explore_control_pub.publish(Bool(data=False))
        self.get_logger().warning(
            f"canceling active automatic mission: {reason}"
        )
        return True

    def _on_control_authority_state(
        self, message: ControlAuthorityState
    ) -> None:
        if not self._authority_gate_enabled:
            return
        received_at = time.monotonic()
        snapshot = AuthoritySnapshot(
            authority=int(message.authority),
            estop_latched=bool(message.estop_latched),
            manager_epoch=int(message.manager_epoch),
            transition_sequence=int(message.transition_sequence),
            active_source=str(message.active_source),
            reason=str(message.reason),
            pending_autonomy_revocation_sequence=int(
                getattr(
                    message,
                    "pending_autonomy_revocation_sequence",
                    0,
                )
            ),
            autonomy_quiescence_acknowledged=bool(
                getattr(
                    message,
                    "autonomy_quiescence_acknowledged",
                    False,
                )
            ),
        )
        with self._state_lock:
            was_observed = self._authority_lease.observed
            was_fresh_autonomy = (
                self._authority_lease.fresh_autonomy(received_at)
            )
            update = self._authority_lease.update(
                snapshot, received_at
            )
            authority = self._authority_lease.authority
        if not update.accepted:
            self.get_logger().error(
                "ignored control authority state "
                f"epoch={message.manager_epoch} "
                f"sequence={message.transition_sequence} "
                f"reason={update.decision.value}"
            )
            return
        self._authority_lease_failure_reported = False
        pending_revocation = (
            snapshot.pending_autonomy_revocation_sequence
        )
        quiescence_identity = QuiescenceIdentity(
            manager_epoch=snapshot.manager_epoch,
            revocation_sequence=pending_revocation,
        )
        if self._autonomy_quiescence.active:
            self._autonomy_quiescence.observe_authority(
                quiescence_identity,
                autonomy_active=authority == AUTONOMY,
            )
            barrier_snapshot = self._autonomy_quiescence.snapshot()
            if barrier_snapshot.state is QuiescenceState.FAILED:
                self._fail_autonomy_quiescence(
                    QuiescenceError(barrier_snapshot.failure_reason)
                )
                return

        revocation_started = False
        if (
            authority != AUTONOMY
            and not snapshot.autonomy_quiescence_acknowledged
        ):
            # 先把撤销身份写入屏障，再启动后台收口。若反过来先 cancel，
            # Action result 或零速回调可能抢先到达，导致本轮证据被错误丢弃。
            self._autonomy_quiescence.observe_authority(
                quiescence_identity
            )
            try:
                manager = getattr(self, "_manager", None)
                explorer_process = getattr(
                    manager, "explorer_process", None
                )
                explorer_running = (
                    explorer_process is not None
                    and explorer_process.poll() is None
                )
                started = self._autonomy_quiescence.begin(
                    quiescence_identity,
                    # 进程刚启动而首帧 ExploreStatus 尚未到达时，也必须要求
                    # ledger terminal，不能把“没有缓存遥测”误当成“没有 goal”。
                    require_frontier=explorer_running,
                )
            except QuiescenceError as exc:
                self._fail_autonomy_quiescence(exc)
                return
            if started:
                revocation_started = True
                with self._state_lock:
                    active_request = self._active_request
                    active_automatic_mission = (
                        active_request is not None
                        and active_request.command
                        == SessionCommand.RUN_AUTOMATIC_MISSION
                        and not active_request.canceled
                    )
                transition = getattr(self, "transition", None)
                if active_automatic_mission and callable(transition):
                    # QUIESCING 描述的是“正在退出的自动任务”，而不是控制权
                    # manager 的空闲握手。若当前根本没有自动任务仍切入该 phase，
                    # ACK 完成后便没有 mission worker 负责恢复 MAPPING/NAVIGATING，
                    # UI 会永久停在 quiescing。空闲撤销仍由 barrier 阻塞新命令。
                    transition(
                        SessionPhase.QUIESCING,
                        detail=(
                            "control authority revoked; waiting for "
                            "Explore/Nav2 terminal, STOP result and fresh zero"
                        ),
                    )
                self._explore_control_pub.publish(Bool(data=False))
                self._start_autonomy_quiescence_worker(
                    quiescence_identity
                )

        # 屏障必须先绑定 revocation，随后才能设置用户 request 的 canceled 位；
        # 否则 Action result/零速回调可能抢在 begin() 前到达并被本轮错误丢弃。
        if authority != AUTONOMY or (
            was_observed and not was_fresh_autonomy
        ):
            reason = {
                ControlAuthorityState.HOLD: "authority_hold",
                ControlAuthorityState.KEYBOARD: "keyboard_takeover",
                ControlAuthorityState.ESTOP: "emergency_stop",
            }.get(authority, "authority_lease_discontinuity")
            SessionOrchestratorNode._cancel_active_automatic_mission(
                self,
                reason,
                (
                    quiescence_identity
                    if revocation_started
                    or self._autonomy_quiescence.identity
                    == quiescence_identity
                    else None
                ),
            )

    def _start_autonomy_quiescence_worker(
        self, identity: QuiescenceIdentity
    ) -> None:
        """确保至少一个持久 worker 会接棒当前及紧随其后的 revocation。"""

        with self._quiescence_worker_lock:
            current = self._quiescence_worker
            if current is not None and current.is_alive():
                return
            worker = threading.Thread(
                target=SessionOrchestratorNode._autonomy_quiescence_worker_loop,
                args=(self, identity),
                name=(
                    "autonomy-quiescence-"
                    f"{identity.manager_epoch}-"
                    f"{identity.revocation_sequence}"
                ),
                daemon=True,
            )
            self._quiescence_worker = worker
            worker.start()

    def _autonomy_quiescence_worker_loop(
        self, identity: QuiescenceIdentity
    ) -> None:
        """串行收口 revocation；退出边沿再次检查，避免丢掉第二代任务。"""

        current_identity = identity
        try:
            while True:
                if not self._autonomy_quiescence.acknowledged(
                    current_identity
                ):
                    self._complete_autonomy_quiescence(current_identity)
                snapshot = self._autonomy_quiescence.snapshot()
                if snapshot.state is QuiescenceState.FAILED:
                    return
                if (
                    snapshot.identity is not None
                    and snapshot.identity != current_identity
                    and self._autonomy_quiescence.active
                ):
                    current_identity = snapshot.identity
                    continue
                return
        finally:
            restart_identity: QuiescenceIdentity | None = None
            with self._quiescence_worker_lock:
                if self._quiescence_worker is threading.current_thread():
                    self._quiescence_worker = None
                snapshot = self._autonomy_quiescence.snapshot()
                if (
                    self._autonomy_quiescence.active
                    and snapshot.identity is not None
                ):
                    restart_identity = snapshot.identity
            if restart_identity is not None:
                # 新 revocation 可能落在上一次 loop 的最终快照之后；finally
                # 重新走同一个原子启动门，保证回调无需依赖固定 sleep 重试。
                SessionOrchestratorNode._start_autonomy_quiescence_worker(
                    self, restart_identity
                )

    def _complete_autonomy_quiescence(
        self, identity: QuiescenceIdentity
    ) -> None:
        """排空 Explore/Nav2、验证 STOP/零速，再向 manager 发送 typed ACK。"""

        deadline = (
            time.monotonic() + self._autonomy_quiescence_timeout_s
        )

        def remaining() -> float:
            value = deadline - time.monotonic()
            if value <= 0.0:
                raise TimeoutError(
                    "autonomy quiescence safety budget exhausted"
                )
            return value

        try:
            if self._autonomy_quiescence.acknowledged(identity):
                return
            snapshot = self._autonomy_quiescence.snapshot()
            if snapshot.identity != identity:
                raise QuiescenceError(
                    "quiescence worker identity no longer current"
                )
            if snapshot.frontier_required:
                telemetry = self.quiesce_frontier(
                    _navigation_safety_stop_request(),
                    timeout_s=remaining(),
                )
                self._autonomy_quiescence.observe_frontier(
                    status=telemetry.status,
                    active_goal_count=telemetry.active_goal_count,
                    accepted_goal_count=telemetry.accepted_goal_count,
                    terminal_goal_count=(
                        telemetry.succeeded_goal_count
                        + telemetry.aborted_goal_count
                        + telemetry.canceled_goal_count
                    ),
                )
                # 账本结算后才允许回收进程；这里的 stop 只是资源清理，
                # 绝不会被屏障当成 Explore terminal 的替代证据。
                self._manager.stop_explorer()
            self._request_quiescence_priority_stop(
                identity,
                timeout_s=remaining(),
            )
            self._autonomy_quiescence.wait_ready(
                timeout_s=remaining()
            )
            # READY 与 service ACK 之间显式进入 ACK_IN_FLIGHT。此后任一新
            # Explore/Nav2/non-zero 证据都会把屏障置为 FAILED，而不是静默退回
            # WAITING 后让远端 manager 消费一张已经失效的许可。
            self._autonomy_quiescence.begin_acknowledgement(identity)
            self._acknowledge_autonomy_quiescence(
                identity,
                timeout_s=remaining(),
            )
            self._autonomy_quiescence.complete_acknowledgement(identity)
            self.get_logger().info(
                "autonomy quiescence acknowledged: "
                f"epoch={identity.manager_epoch} "
                f"revocation={identity.revocation_sequence}"
            )
        except Exception as exc:
            # service response 与 authority topic 没有跨通道顺序。若 manager 的
            # AUTONOMY/下一撤销回调已经权威完成本 identity，迟到 future 异常不应
            # 反向污染下一代屏障；其余情况一律 fail-closed。
            if self._autonomy_quiescence.acknowledged(identity):
                return
            self._fail_autonomy_quiescence(exc)

    def _request_quiescence_priority_stop(
        self,
        identity: QuiescenceIdentity,
        *,
        timeout_s: float,
    ) -> None:
        """发布与本轮 revocation 绑定的 priority STOP，并等待 exact result。"""

        command_id = (
            f"quiescence-stop-{identity.manager_epoch}-"
            f"{identity.revocation_sequence}"
        )
        with self._cmd_vel_condition:
            cmd_vel_generation = self._cmd_vel_generation
        self._autonomy_quiescence.mark_priority_stop_requested(
            command_id,
            cmd_vel_generation=cmd_vel_generation,
        )

        command = RobotCommand()
        command.header.stamp = self.get_clock().now().to_msg()
        command.command_id = command_id
        command.source = "slam_session_quiescence"
        command.priority = True
        command.action_type = RobotCommand.STOP
        self._agent_action_gateway.run_typed(
            _navigation_safety_stop_request(),
            command_id=command_id,
            publish_command=lambda: self._internal_action_pub.publish(
                command
            ),
            expected_action_name="stop",
            timeout_s=timeout_s,
        )

    def _acknowledge_autonomy_quiescence(
        self,
        identity: QuiescenceIdentity,
        *,
        timeout_s: float,
    ) -> None:
        """调用 exact epoch/revocation ACK；服务不可用或拒绝都保持 fail-closed。"""

        client = self._quiescence_ack_client
        service_type = AcknowledgeAutonomyQuiescence
        if client is None or service_type is None:
            raise RuntimeError(
                "autonomy quiescence ACK service is unavailable"
            )
        deadline = time.monotonic() + timeout_s
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(
                "autonomy quiescence ACK service unavailable"
            )
        request = service_type.Request()
        request.manager_epoch = identity.manager_epoch
        request.autonomy_revocation_sequence = (
            identity.revocation_sequence
        )
        request.requester = "slam_session_orchestrator"
        request.detail = (
            "Explore/Nav2 terminal, priority STOP result and fresh zero "
            "cmd_vel verified"
        )
        response_future = client.call_async(request)
        completed = threading.Event()
        response_future.add_done_callback(lambda _future: completed.set())
        response_timeout_s = max(0.0, deadline - time.monotonic())
        if not completed.wait(timeout=response_timeout_s):
            raise TimeoutError(
                "autonomy quiescence ACK response timeout"
            )
        response = response_future.result()
        if response is None or not bool(response.accepted):
            message = (
                "empty response"
                if response is None
                else str(response.message)
            )
            raise RuntimeError(
                f"autonomy quiescence ACK rejected: {message}"
            )
        state = response.state
        if (
            int(state.manager_epoch) != identity.manager_epoch
            or int(state.pending_autonomy_revocation_sequence)
            != identity.revocation_sequence
            or not bool(state.autonomy_quiescence_acknowledged)
        ):
            raise RuntimeError(
                "autonomy quiescence ACK returned stale state"
            )

    def _fail_autonomy_quiescence(self, error: BaseException) -> None:
        reason = f"autonomy quiescence failed: {error}"
        self._autonomy_quiescence.fail(reason)
        with self._state_lock:
            self._mission_outcome = SlamSessionState.MISSION_FAILED
            self._mission_message = reason
        # timeout、manager 重启或任一终态缺失都不能回到 MAPPING/NAVIGATING；
        # FAILED 会阻止“按 R 恢复后旧任务继续跑”的假恢复。
        self.transition(SessionPhase.FAILED, detail=reason)
        self.get_logger().error(reason)

    def _check_authority_lease(self) -> None:
        if not self._authority_gate_enabled:
            return
        with self._state_lock:
            authority = self._authority_lease.authority
            fresh = self._authority_lease.fresh(time.monotonic())
            already_reported = getattr(
                self, "_authority_lease_failure_reported", False
            )
            if authority == AUTONOMY and not fresh and not already_reported:
                self._authority_lease_failure_reported = True
                should_fail = True
            else:
                should_fail = False
        if should_fail:
            SessionOrchestratorNode._cancel_active_automatic_mission(
                self, "authority_lease_expired"
            )
            self._fail_autonomy_quiescence(
                TimeoutError(
                    "authority manager lease expired before typed revocation"
                )
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
        if self._return_to_start_evidence is not None:
            draft = self._return_to_start_evidence
            # map_saver 是同步调用，现场一次保存耗时超过 5 秒。返航时拿到的
            # 零速到这里可能已经超过 freshness 门槛，因此不能通过放宽阈值
            # 或改写时间戳“续期”。先记录真实存图边界，再经 typed STOP 链路
            # 获取一帧保存后的真实零速，作为切换 Nav2 前的第二道安全确认。
            saved_at_ns = max(
                self.get_clock().now().nanoseconds,
                draft.action_finished_at_ns + 1,
                draft.evaluated_at_ns + 1,
            )
            self.stop_motion_and_wait(
                _navigation_safety_stop_request(),
                timeout_s=_NAV2_SAFETY_STOP_TIMEOUT_S,
            )
            with self._cmd_vel_condition:
                linear_x, angular_z = self._last_cmd_vel
                cmd_vel_observed_at_ns = self._last_cmd_vel_observed_at_ns
            evaluated_at_ns = max(
                self.get_clock().now().nanoseconds,
                saved_at_ns,
                cmd_vel_observed_at_ns,
            )
            finalized = replace(
                draft,
                map_saved_at_ns=saved_at_ns,
                cmd_vel_linear_x=linear_x,
                cmd_vel_angular_z=angular_z,
                cmd_vel_observed_at_ns=cmd_vel_observed_at_ns,
                evaluated_at_ns=evaluated_at_ns,
            )
            decision = evaluate_return_to_start(
                finalized,
                self._return_to_start_spec,
            )
            if not decision.passed:
                raise RuntimeError(
                    "return-to-start evidence invalid after map save "
                    "and before navigation: "
                    + ",".join(decision.failed_checks)
                )
            self._return_to_start_evidence = finalized
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
        try:
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
            # 不保证测试或 UI 一定调度到每个中间快照；Action feedback因此同步承载阶段进度。
            self.feedback(request, 0.8)
            self._manager.start("navigation")
            self._wait_for_new_ready(generation)
        except Exception as exc:
            self._cleanup_failed_navigation_startup(exc)
            raise
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
            if (
                accepted
                and getattr(self, "_authority_gate_enabled", False)
                and request.command
                == SessionCommand.RUN_AUTOMATIC_MISSION
                and (
                    not self._authority_lease.fresh_autonomy(
                        time.monotonic()
                    )
                    or request.authority_generation
                    != self._authority_lease.generation
                )
            ):
                accepted = False
                reason = (
                    "automatic mission authority generation expired "
                    "before execution"
                )
            if not accepted:
                request.message = reason
                request.completed.set()
                return
            # 安装 active request 与 authority 二次检查必须在同一把锁内：
            # 切权回调要么先改变 generation 令本请求失败，要么随后看见本请求并取消。
            self._operation_active.set()
            self._active_request = request
        # 每条 request 独立记录阶段启动故障，避免上一轮失败污染下一轮状态。
        self._navigation_startup_failure_pending = False
        try:
            if request.command == SessionCommand.RUN_AUTOMATIC_MISSION:
                # 同一 orchestrator 可运行多次任务；显式 mission_sequence 能阻止
                # 上一轮 transient-local goal 快照被验收器误归到新任务。
                self._mission_sequence += 1
                self._mission_outcome = SlamSessionState.MISSION_RUNNING
                self._mission_message = "automatic mission running"
                self._navigation_goal_ledger.reset(self._mission_sequence)
                # 收口证据只属于单次 mission。若不先清空，第二轮刚启动时发布的
                # transient-local state 会携带上一轮 return/saturation PASS，验收器
                # 可能在机器人尚未运动前误判本轮已经返航并完成建图。
                # 使用类方法显式调用，也让不构造完整 rclpy Node 的故障注入
                # fake 走到同一重置逻辑，而不要求测试对象伪造一个绑定方法。
                SessionOrchestratorNode._reset_mapping_completion_evidence(
                    self
                )
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
            revocation = request.authority_revocation_identity
            if (
                request.command == SessionCommand.RUN_AUTOMATIC_MISSION
                and revocation is not None
            ):
                identity = QuiescenceIdentity(*revocation)
                # 用户任务 worker 保持 `_operation_active`，直到后台安全 worker
                # 完成 exact ACK/FAILED。这样保存地图或切换 Nav2 stage 不会在
                # Explore/Nav2 result 尚未结算时抢跑。
                self.transition(
                    SessionPhase.QUIESCING,
                    detail=(
                        "automatic mission canceled; waiting for control "
                        "authority quiescence ACK"
                    ),
                )
                try:
                    self._autonomy_quiescence.wait_acknowledged(
                        identity,
                        timeout_s=self._autonomy_quiescence_timeout_s,
                    )
                except Exception as quiescence_error:
                    if (
                        self._autonomy_quiescence.snapshot().state
                        is not QuiescenceState.FAILED
                    ):
                        self._fail_autonomy_quiescence(quiescence_error)
            quiescence_failed = (
                self._autonomy_quiescence.snapshot().state
                is QuiescenceState.FAILED
            )
            if (
                request.command == SessionCommand.RUN_AUTOMATIC_MISSION
                and quiescence_failed
            ):
                request.message = (
                    self._autonomy_quiescence.snapshot().failure_reason
                    or request.message
                )
                self._mission_outcome = SlamSessionState.MISSION_FAILED
                self._mission_message = request.message
                self.transition(
                    SessionPhase.FAILED,
                    detail=request.message,
                )
                self.get_logger().error(request.message)
                return
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
                if self._navigation_startup_failure_pending:
                    # manager.stage 只能说明“start 被调用过”，不能证明 Nav2 ready。
                    # 启动/就绪失败后若回写 MAPPING 或 NAVIGATING，会向 UI 和验收器
                    # 发布一个并不存在的可用系统；因此这里必须显式进入 FAILED。
                    self.transition(
                        SessionPhase.FAILED,
                        detail=f"automatic mission failed: {exc}",
                    )
                else:
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
            self._navigation_startup_failure_pending = False
            with self._state_lock:
                # 只清除本 worker 安装的 request；并发控制权回调只能标 canceled，
                # 不能把下一轮请求或别的测试 fake 覆盖掉。
                if self._active_request is request:
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
