"""单终端编排真实感语音建图、保存地图和导航阶段。"""

from __future__ import annotations

from dataclasses import dataclass, replace
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
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
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
from .nav2_motion_ros import RosNav2MotionAdapter
from .nav2_motion_transaction import (
    MappingReturn,
    Nav2MotionTransaction,
    RecoveryBackup,
    SampledNavigate,
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

try:
    from embodied_agent_interfaces.srv import SetControlAuthority
except ImportError:  # 与 ACK 接口分别降级，避免一个缺失掩盖另一个诊断。
    SetControlAuthority = None


_ACTION_TYPES = {
    "stop": RobotCommand.STOP,
    "move": RobotCommand.MOVE,
    "turn": RobotCommand.TURN,
    "navigate_to": RobotCommand.NAVIGATE_TO,
    "follow_waypoints": RobotCommand.FOLLOW_WAYPOINTS,
}
_PLAN_GOAL_TOLERANCE_M = 0.75
_NAV2_SAFETY_STOP_TIMEOUT_S = 10.0
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


class _PersistentStageSwitchInterrupted(RuntimeError):
    """mapping 已销毁后被接管；禁止外层伪恢复到 MAPPING。"""


class _GoalCandidateRejected(RuntimeError):
    """单个候选不可达或路径不安全；允许 admission 继续尝试下一点。"""

    def __init__(self, reason: str) -> None:
        super().__init__(reason)
        self.reason = str(reason)


@dataclass(slots=True)
class _InternalStageSwitch:
    """绑定一次内部 HOLD 的防重放身份与唯一 reason token。"""

    reason: str
    manager_epoch: int
    transition_sequence_floor: int
    identity: QuiescenceIdentity | None = None


def _navigation_safety_stop_request() -> CommandRequest:
    """创建不继承用户取消位的内部停车事务。"""

    # 用户 request 在 cancel 分支中必然 canceled=True；复用它会让 gateway 在
    # 发布 priority STOP 前直接退出。因此安全停车始终使用独立、未取消 request。
    return CommandRequest(
        command=SessionCommand.STOP_SESSION,
        source="navigation_safety_stop",
    )


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


def _request_lifecycle_transition(
    client,
    transition_id: int,
    timeout_s: float,
) -> bool | None:
    """提交 lifecycle 转换；``None`` 表示响应未知，不能等同于明确拒绝。"""

    request = ChangeState.Request()
    request.transition.id = int(transition_id)
    response = client.call(request, timeout_sec=timeout_s)
    if response is None:
        return None
    return bool(response.success)


def _converge_lifecycle_active(
    state_client,
    change_client,
    *,
    deadline: float,
    label: str,
    runtime_guard: Callable[[], None],
) -> None:
    """通过 state/service 闭环把 lifecycle 节点推进到 ACTIVE。"""

    while True:
        runtime_guard()
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            raise TimeoutError(f"{label} did not become active before timeout")
        if not state_client.wait_for_service(
            timeout_sec=min(0.5, remaining)
        ):
            continue
        state = _get_lifecycle_state(
            state_client,
            min(0.5, remaining),
        )
        if state == State.PRIMARY_STATE_ACTIVE:
            # ACTIVE 响应和提交下一阶段之间仍存在调度窗口；返回前复核
            # owner 进程，禁止用已退出进程的迟到响应放行。
            runtime_guard()
            return
        if state == State.PRIMARY_STATE_FINALIZED:
            raise RuntimeError(f"{label} reached finalized during startup")

        transition_id = None
        if state == State.PRIMARY_STATE_UNCONFIGURED:
            transition_id = Transition.TRANSITION_CONFIGURE
        elif state == State.PRIMARY_STATE_INACTIVE:
            transition_id = Transition.TRANSITION_ACTIVATE
        if transition_id is None:
            # configuring/activating 等过渡态只能等待，禁止重复发送转换。
            time.sleep(min(0.1, max(0.0, remaining)))
            continue
        if not change_client.wait_for_service(
            timeout_sec=min(0.5, remaining)
        ):
            continue
        transition_result = _request_lifecycle_transition(
            change_client,
            transition_id,
            min(0.5, remaining),
        )
        if transition_result is False:
            raise RuntimeError(
                f"{label} rejected lifecycle transition "
                f"{transition_id} from state={state}"
            )
        if transition_result is None:
            # 响应超时不等于转换未执行；重新 GetState 判断真实结果。
            time.sleep(min(0.05, max(0.0, remaining)))


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
        # 在新的持久演示完成 Gazebo E2E 之前保持默认关闭，避免改变已发布的
        # unknown-world 门禁语义。新 launch 必须显式 opt-in 才复用同一 base。
        self._persistent_runtime_enabled = bool(
            self.declare_parameter(
                "persistent_runtime_enabled", False
            ).value
        )
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
            persistent_runtime_enabled=self._persistent_runtime_enabled,
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
        self._authority_condition = threading.Condition(self._state_lock)
        self._authority_snapshot: AuthoritySnapshot | None = None
        self._internal_stage_switch: _InternalStageSwitch | None = None
        self._internal_stage_switch_sequence = 0
        self._ready_condition = threading.Condition()
        self._ready_generation = 0
        self._latest_ready = False
        # persistent mapping/navigation 各自拥有 readiness aggregator。切换前
        # 先武装 exact profile，避免常驻 base 或旧 stage 心跳提前解锁 RESUME。
        self._expected_readiness_profile = ""
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
        if (
            self._persistent_runtime_enabled
            and not self._authority_gate_enabled
        ):
            raise ValueError(
                "persistent runtime requires authority_gate_enabled=true"
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
        self._map_pose_condition = threading.Condition()
        self._latest_occupancy: OccupancySnapshot | None = None
        self._latest_map_pose_xy: tuple[float, float] | None = None
        self._navigation_input_generation = 0
        self._navigation_inputs_enabled = False
        self._latest_occupancy_generation = -1
        self._latest_map_pose_generation = -1
        self._mission_sequence = 0
        self._mission_outcome = SlamSessionState.MISSION_IDLE
        self._mission_message = ""
        self._child_runtime_failure = ""
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
        initial_pose_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            # 编排器跨 mapping/navigation stage 常驻。禁止 transient-local，
            # 否则下一代 AMCL 会自动收到上一代缓存位姿，绕过 generation 屏障。
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
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "/initialpose",
            initial_pose_qos,
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
        nav2_runtime = RosNav2MotionAdapter(
            navigate_to_pose_client=self._navigate_to_pose_client,
            backup_client=self._backup_client,
            force_priority_stop=self._force_navigation_priority_stop,
            stop_navigation_stage=self._manager.stop_stage,
            nav2_goal_started=self._autonomy_quiescence.nav2_goal_started,
            nav2_goal_terminal=self._autonomy_quiescence.nav2_goal_terminal,
            evidence_timestamp_ns=(
                lambda: self.get_clock().now().nanoseconds
            ),
            navigation_evidence_changed=self._publish_state,
        )
        # 三类 Nav2 运动共享同一个事务实例和互斥锁；这样 recovery、返航与
        # sampled navigation 不会在竞态中同时取得底盘控制权。
        self._nav2_motion = Nav2MotionTransaction(
            nav2_runtime,
            self._navigation_goal_ledger,
            safety_timeout_s=_NAV2_SAFETY_STOP_TIMEOUT_S,
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
        self._collision_monitor_state_client = self.create_client(
            GetState,
            "/collision_monitor/get_state",
            callback_group=callback_group,
        )
        self._typed_action_bridge_state_client = self.create_client(
            GetState,
            "/typed_action_bridge/get_state",
            callback_group=callback_group,
        )
        self._typed_action_bridge_change_client = self.create_client(
            ChangeState,
            "/typed_action_bridge/change_state",
            callback_group=callback_group,
        )
        self._simulation_control_state_client = self.create_client(
            GetState,
            "/simulation_control/get_state",
            callback_group=callback_group,
        )
        self._simulation_control_change_client = self.create_client(
            ChangeState,
            "/simulation_control/change_state",
            callback_group=callback_group,
        )
        self._map_server_state_client = self.create_client(
            GetState,
            "/map_server/get_state",
            callback_group=callback_group,
        )
        self._amcl_state_client = self.create_client(
            GetState,
            "/amcl/get_state",
            callback_group=callback_group,
        )
        self._quiescence_ack_client = None
        self._authority_control_client = None
        if self._authority_gate_enabled:
            if (
                AcknowledgeAutonomyQuiescence is None
                or SetControlAuthority is None
            ):
                raise RuntimeError(
                    "authority gate requires generated control authority "
                    "services"
                )
            self._quiescence_ack_client = self.create_client(
                AcknowledgeAutonomyQuiescence,
                "/control/acknowledge_autonomy_quiescence",
                callback_group=callback_group,
            )
            self._authority_control_client = self.create_client(
                SetControlAuthority,
                "/control/set_authority",
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
            expected_profile = getattr(
                self, "_expected_readiness_profile", ""
            )
            if (
                expected_profile
                and str(getattr(message, "profile", ""))
                != expected_profile
            ):
                return
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
            if not getattr(self, "_navigation_inputs_enabled", True):
                return
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
            if not getattr(self, "_navigation_inputs_enabled", True):
                return
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

    def _force_navigation_priority_stop(self, timeout_s: float) -> None:
        """供 Nav2 事务使用独立 request 发布 priority typed STOP。"""

        # 原任务进入 canceled/timeout 后不可再复用它，否则 Gateway 会在发出
        # STOP 前提前退出。即使 timeout=0，也必须先发布命令再做即时终态检查。
        self.stop_motion_and_wait(
            _navigation_safety_stop_request(),
            timeout_s=max(0.0, float(timeout_s)),
        )

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
        self._nav2_motion.execute(
            RecoveryBackup(
                distance_m=distance_m,
                speed_mps=speed_mps,
                time_allowance_s=timeout_s,
            ),
            is_cancelled=lambda: bool(request.canceled),
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
                if getattr(request, "canceled", False):
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
            if getattr(request, "canceled", False):
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
                if getattr(request, "canceled", False):
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
            if getattr(request, "canceled", False):
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
        # 从 accepted 到 terminal 的状态、取消、STOP 与 ledger 收口全部由
        # 深 Module 负责；Node 不再维护第二套并行状态机。
        self._nav2_motion.execute(
            SampledNavigate(sequence=sequence, goal_xy=goal_xy),
            is_cancelled=lambda: bool(request.canceled),
            deadline_monotonic=deadline,
        )

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
        self._nav2_motion.execute(
            MappingReturn(goal_pose=start_pose),
            is_cancelled=lambda: bool(request.canceled),
            deadline_monotonic=deadline,
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

    def _wait_persistent_base_control_ready(self) -> None:
        """由唯一编排器把常驻 typed bridge 推进到 ACTIVE，再启动 stage."""

        if self._dry_run:
            return
        deadline = time.monotonic() + self._startup_timeout_s

        def require_live_base() -> None:
            role, code = self._manager.unexpected_exit()
            if role == "base":
                raise RuntimeError(
                    "persistent base process exited "
                    f"code={code} before typed Action bridge became active"
                )

        _converge_lifecycle_active(
            self._typed_action_bridge_state_client,
            self._typed_action_bridge_change_client,
            deadline=deadline,
            label="typed Action bridge",
            runtime_guard=require_live_base,
        )

    def _wait_persistent_stage_executor_ready(self) -> None:
        """由会话 owner 激活当前代 simulation_control executor。"""

        if self._dry_run:
            return
        deadline = time.monotonic() + self._startup_timeout_s

        def require_live_runtime() -> None:
            role, code = self._manager.unexpected_exit()
            if role is not None:
                raise RuntimeError(
                    f"{role} process exited code={code} before "
                    "simulation_control became active"
                )

        _converge_lifecycle_active(
            self._simulation_control_state_client,
            self._simulation_control_change_client,
            deadline=deadline,
            label="simulation_control",
            runtime_guard=require_live_runtime,
        )

    def _wait_persistent_velocity_pipeline_ready(self) -> None:
        """在开放建图动作前验证最终速度安全边界已真正可用."""

        if self._dry_run:
            return
        deadline = time.monotonic() + self._startup_timeout_s
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(
                    "Collision Monitor did not become active before timeout"
                )
            if not self._collision_monitor_state_client.wait_for_service(
                timeout_sec=min(0.5, remaining)
            ):
                continue
            state = _get_lifecycle_state(
                self._collision_monitor_state_client,
                min(0.5, remaining),
            )
            if state == State.PRIMARY_STATE_ACTIVE:
                break
            time.sleep(0.1)

        with self._cmd_vel_condition:
            generation = self._cmd_vel_generation
            while True:
                publisher_count = self.count_publishers("/cmd_vel")
                linear_x, angular_z = self._last_cmd_vel
                if (
                    publisher_count == 1
                    and self._cmd_vel_generation > generation
                    and abs(linear_x) <= 1.0e-3
                    and abs(angular_z) <= 1.0e-3
                ):
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        "final velocity pipeline is not ready: "
                        f"publishers={publisher_count} "
                        f"generation={self._cmd_vel_generation} "
                        f"velocity=({linear_x:.4f},{angular_z:.4f})"
                    )
                # 必须观察 ACTIVE 之后的新鲜最终零速；启动前缓存和仅有
                # /control/selected/cmd_vel 的零速都不能证明 Collision Monitor
                # 已接管最后一道障碍安全裁决。
                self._cmd_vel_condition.wait(timeout=min(0.1, remaining))

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
        if getattr(self, "_persistent_runtime_enabled", False):
            # bt_navigator/waypoint_follower 属于常驻或可提前激活的公共组件，
            # 不能证明本轮保存地图对应的 map_server/AMCL stage 已经 ready。
            lifecycle_clients += (
                ("map_server", self._map_server_state_client),
                ("amcl", self._amcl_state_client),
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
        if getattr(self, "_persistent_runtime_enabled", False):
            # /initialpose 必须属于同一 readiness 事务：只有 AMCL ACTIVE 且
            # DDS 订阅已匹配后才发布，并等待本代 /amcl_pose 回执。
            self._seed_persistent_amcl_pose(request, deadline)
            SessionOrchestratorNode._wait_for_persistent_navigation_inputs(
                self, request, deadline
            )
        self.get_logger().info(
            "Nav2 navigation Action servers and lifecycle nodes are active"
        )

    def _persistent_navigation_seed_pose(self) -> PlanarPose:
        """选择切换前最后一帧可信 SLAM 位姿作为 AMCL 初值."""

        evidence = getattr(self, "_return_to_start_evidence", None)
        if evidence is None:
            # 兼容手工 save→navigation：旧入口的明确契约就是保存地图起点
            # (0,0,0)。自动任务必须有返航证据，因此不会走到这个兼容分支。
            return PlanarPose(
                x=0.0,
                y=0.0,
                yaw=0.0,
                frame_id="map",
                observed_at_ns=0,
            )
        final_pose = evidence.final_pose
        if final_pose is None or final_pose.frame_id != "map":
            raise RuntimeError(
                "persistent navigation requires a finalized return pose "
                "in the map frame"
            )
        if int(getattr(evidence, "map_saved_at_ns", 0)) <= 0:
            raise RuntimeError(
                "persistent navigation return evidence is not finalized "
                "after map save"
            )
        return final_pose

    def _build_persistent_initial_pose(
        self, pose: PlanarPose
    ) -> PoseWithCovarianceStamped:
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = pose.x
        message.pose.pose.position.y = pose.y
        message.pose.pose.orientation.z = math.sin(pose.yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(pose.yaw / 2.0)
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[35] = 0.0685
        return message

    def _seed_persistent_amcl_pose(
        self,
        request: CommandRequest,
        deadline: float,
    ) -> None:
        """匹配 ACTIVE AMCL，并重发初值直到收到当前代定位回执."""

        pose = SessionOrchestratorNode._persistent_navigation_seed_pose(self)
        while self._initial_pose_pub.get_subscription_count() <= 0:
            if getattr(request, "canceled", False):
                self._cancel_automatic_motion()
                raise AutomaticMissionCancelled(
                    "automatic mission canceled"
                )
            role, code = self._manager.unexpected_exit()
            if role is not None:
                raise RuntimeError(
                    f"{role} process exited code={code} while waiting for "
                    "AMCL initial-pose subscriber"
                )
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError(
                    "/initialpose subscriber did not match before "
                    "navigation startup deadline"
                )
            time.sleep(min(0.05, remaining))

        generation = self._navigation_input_generation
        next_publish_at = 0.0
        publish_count = 0
        while True:
            with self._map_pose_condition:
                if (
                    self._latest_map_pose_xy is not None
                    and self._latest_map_pose_generation == generation
                ):
                    self.get_logger().info(
                        "AMCL initial pose acknowledged for "
                        f"generation={generation} after "
                        f"{publish_count} publish attempt(s)"
                    )
                    return
            if getattr(request, "canceled", False):
                self._cancel_automatic_motion()
                raise AutomaticMissionCancelled(
                    "automatic mission canceled"
                )
            role, code = self._manager.unexpected_exit()
            if role is not None:
                raise RuntimeError(
                    f"{role} process exited code={code} before AMCL "
                    "initial pose acknowledgement"
                )
            now = time.monotonic()
            remaining = deadline - now
            if remaining <= 0.0:
                raise TimeoutError(
                    "AMCL did not publish a current-generation pose after "
                    f"{publish_count} /initialpose attempt(s)"
                )
            if now >= next_publish_at:
                self._initial_pose_pub.publish(
                    SessionOrchestratorNode._build_persistent_initial_pose(
                        self, pose
                    )
                )
                publish_count += 1
                next_publish_at = now + 0.25
            with self._map_pose_condition:
                self._map_pose_condition.wait(
                    timeout=min(
                        0.05,
                        remaining,
                        max(0.0, next_publish_at - time.monotonic()),
                    )
                )

    def _wait_for_persistent_navigation_inputs(
        self,
        request: CommandRequest,
        deadline: float,
    ) -> None:
        """等待当前 navigation 代际的 map 与 AMCL 首帧，拒绝旧缓存。"""

        with self._map_pose_condition:
            generation = self._navigation_input_generation
            while True:
                fresh_map = (
                    self._latest_occupancy is not None
                    and self._latest_occupancy_generation == generation
                )
                fresh_pose = (
                    self._latest_map_pose_xy is not None
                    and self._latest_map_pose_generation == generation
                )
                if fresh_map and fresh_pose:
                    return
                if getattr(request, "canceled", False):
                    self._cancel_automatic_motion()
                    raise AutomaticMissionCancelled(
                        "automatic mission canceled"
                    )
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        "navigation stage did not publish fresh /map and "
                        "/amcl_pose for the current generation"
                    )
                self._map_pose_condition.wait(
                    timeout=min(0.1, remaining)
                )

    def _wait_persistent_navigation_ready(
        self, request: CommandRequest
    ) -> None:
        """stage 切换事务内的完整 Nav2 就绪门，成功后才允许 RESUME。"""

        if self._dry_run:
            return
        self._wait_navigation_ready_impl(request)

    def _cleanup_failed_navigation_startup(self, primary_error: Exception) -> None:
        """回收半启动导航 stage，同时保留最先暴露的根因。"""

        # stage 启动失败或 readiness 超时后，进程是否完整存活已经不可证明。
        # 必须 fail-close；若 stop 也失败，只把它作为异常附注，不能覆盖首因。
        self._navigation_startup_failure_pending = True
        try:
            if getattr(self, "_persistent_runtime_enabled", False):
                # 导航 stage 半启动只污染可替换层；base 仍是本次会话唯一的
                # Gazebo/机器人所有者，故障清理不能把它一并销毁。
                self._manager.stop_stage()
            else:
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

    def _arm_readiness_profile(self, profile: str) -> int:
        """原子切换期望 profile，并返回本轮等待使用的代际基线。"""

        with self._ready_condition:
            self._expected_readiness_profile = profile
            # 旧 profile 的 true 不能泄漏进新阶段；新 aggregator 至少发布一帧
            # exact profile 后 generation 才会推进。
            self._latest_ready = False
            return self._ready_generation

    def _goal_callback(self, goal_request) -> GoalResponse:
        stopping = getattr(self, "_stopping", None)
        if stopping is not None and stopping.is_set():
            return GoalResponse.REJECT
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
        stopping = getattr(self, "_stopping", None)
        if stopping is not None and stopping.is_set():
            # startup fail-close 与 Action goal/execute callback 可能并发；先释放
            # 等待者，再拒绝入队，避免调用方一直等待一个已经没有 worker 的请求。
            request.message = "session is stopping"
            request.completed.set()
            return False
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

    def _call_control_authority(
        self,
        command: int,
        *,
        reason: str,
        timeout_s: float,
    ) -> Any:
        """在 worker 线程同步等待 typed service，ROS executor 仍可处理回调。"""

        service_type = SetControlAuthority
        client = getattr(self, "_authority_control_client", None)
        if service_type is None or client is None:
            raise RuntimeError(
                "control authority service is unavailable"
            )
        deadline = time.monotonic() + timeout_s
        if not client.wait_for_service(timeout_sec=timeout_s):
            raise TimeoutError(
                "control authority service unavailable"
            )
        service_request = service_type.Request()
        service_request.command = command
        service_request.requester = "slam_session_stage_switch"
        service_request.reason = reason
        future = client.call_async(service_request)
        completed = threading.Event()
        future.add_done_callback(lambda _future: completed.set())
        if not completed.wait(
            timeout=max(0.0, deadline - time.monotonic())
        ):
            raise TimeoutError(
                "control authority service response timeout"
            )
        response = future.result()
        if response is None:
            raise RuntimeError("control authority service returned no response")
        return response

    def _enter_internal_stage_switch_hold(
        self, request: CommandRequest
    ) -> QuiescenceIdentity:
        """进入 exact HOLD，并等现有 quiescence 屏障完成 typed ACK。"""

        deadline = (
            time.monotonic() + self._autonomy_quiescence_timeout_s
        )

        def remaining() -> float:
            value = deadline - time.monotonic()
            if value <= 0.0:
                raise TimeoutError(
                    "internal stage switch HOLD budget exhausted"
                )
            return value

        with self._state_lock:
            snapshot = getattr(self, "_authority_snapshot", None)
            if (
                snapshot is None
                or snapshot.authority != AUTONOMY
                or not self._authority_lease.fresh_autonomy(
                    time.monotonic()
                )
            ):
                raise RuntimeError(
                    "internal stage switch requires fresh AUTONOMY authority"
                )
            self._internal_stage_switch_sequence += 1
            reason = (
                "internal_stage_switch:"
                f"{snapshot.manager_epoch}:"
                f"{snapshot.transition_sequence}:"
                f"{self._internal_stage_switch_sequence}"
            )
            intent = _InternalStageSwitch(
                reason=reason,
                manager_epoch=snapshot.manager_epoch,
                transition_sequence_floor=snapshot.transition_sequence,
            )
            self._internal_stage_switch = intent

        response = SessionOrchestratorNode._call_control_authority(
            self,
            SetControlAuthority.Request.ENTER_HOLD,
            reason=reason,
            timeout_s=remaining(),
        )
        state = response.state
        identity = QuiescenceIdentity(
            int(state.manager_epoch),
            int(state.pending_autonomy_revocation_sequence),
        )
        if (
            not bool(response.accepted)
            or not bool(response.changed)
            or not bool(response.stop_requested)
            or int(state.authority) != ControlAuthorityState.HOLD
            or bool(state.estop_latched)
            or int(state.manager_epoch) != intent.manager_epoch
            or int(state.transition_sequence)
            <= intent.transition_sequence_floor
            or int(state.pending_autonomy_revocation_sequence)
            != int(state.transition_sequence)
            or bool(state.autonomy_quiescence_acknowledged)
            or str(state.active_source)
            or str(state.reason) != intent.reason
        ):
            detail = str(getattr(response, "message", "invalid state"))
            raise RuntimeError(
                "internal stage switch HOLD rejected or stale: "
                f"{detail}"
            )
        with self._state_lock:
            if self._internal_stage_switch is not intent:
                raise RuntimeError(
                    "internal stage switch intent was replaced"
                )
            if intent.identity not in {None, identity}:
                raise RuntimeError(
                    "internal stage switch revocation identity conflict"
                )
            intent.identity = identity

        # 回调侧复用既有安全深模块完成 Explore/Nav2 terminal、priority
        # STOP、新鲜零速与 exact ACK；这里不复制第二套“看起来相似”的停车逻辑。
        self._autonomy_quiescence.wait_acknowledged(
            identity,
            timeout_s=remaining(),
        )
        if request.canceled:
            raise AutomaticMissionCancelled(
                "external control authority changed during stage quiescence"
            )
        return identity

    def _resume_internal_stage_switch(
        self, identity: QuiescenceIdentity
    ) -> None:
        """只从本轮已 ACK 的 HOLD 显式恢复；并发人工接管保持 fail-closed。"""

        deadline = (
            time.monotonic() + self._autonomy_quiescence_timeout_s
        )

        def remaining() -> float:
            value = deadline - time.monotonic()
            if value <= 0.0:
                raise TimeoutError(
                    "internal stage switch RESUME budget exhausted"
                )
            return value

        condition = self._authority_condition
        with condition:
            while True:
                intent = self._internal_stage_switch
                snapshot = self._authority_snapshot
                intent_matches = (
                    intent is not None
                    and intent.identity == identity
                    and intent.manager_epoch == identity.manager_epoch
                )
                hold_is_acknowledged = (
                    snapshot is not None
                    and snapshot.authority
                    == ControlAuthorityState.HOLD
                    and snapshot.manager_epoch == identity.manager_epoch
                    and snapshot.pending_autonomy_revocation_sequence
                    == identity.revocation_sequence
                    and snapshot.autonomy_quiescence_acknowledged
                    and not snapshot.estop_latched
                    and not snapshot.active_source
                )
                if intent_matches and hold_is_acknowledged:
                    hold_sequence = snapshot.transition_sequence
                    resume_reason = f"{intent.reason}:resume"
                    break
                if (
                    not intent_matches
                    or snapshot is None
                    or snapshot.manager_epoch != identity.manager_epoch
                    or snapshot.authority
                    in {
                        ControlAuthorityState.KEYBOARD,
                        ControlAuthorityState.ESTOP,
                    }
                ):
                    raise RuntimeError(
                        "control authority changed during internal stage switch"
                    )
                condition.wait(timeout=remaining())

        response = SessionOrchestratorNode._call_control_authority(
            self,
            SetControlAuthority.Request.RESUME_AUTONOMY,
            reason=resume_reason,
            timeout_s=remaining(),
        )
        state = response.state
        resumed_sequence = int(state.transition_sequence)
        if (
            not bool(response.accepted)
            or not bool(response.changed)
            or bool(response.stop_requested)
            or int(state.authority) != ControlAuthorityState.AUTONOMY
            or bool(state.estop_latched)
            or int(state.manager_epoch) != identity.manager_epoch
            or resumed_sequence <= hold_sequence
            or int(state.pending_autonomy_revocation_sequence) != 0
            or bool(state.autonomy_quiescence_acknowledged)
            or str(state.active_source) != "autonomy"
            or str(state.reason) != resume_reason
        ):
            detail = str(getattr(response, "message", "invalid state"))
            raise RuntimeError(
                "internal stage switch RESUME rejected or stale: "
                f"{detail}"
            )

        # service response 与 transient-local topic 是两条 DDS 通道。只有本地
        # lease 也看见同代 AUTONOMY 后才允许新 Nav2 goal 离开 orchestrator。
        with condition:
            while True:
                snapshot = self._authority_snapshot
                if (
                    snapshot is not None
                    and snapshot.manager_epoch == identity.manager_epoch
                    and snapshot.transition_sequence >= resumed_sequence
                    and snapshot.authority
                    == ControlAuthorityState.AUTONOMY
                    and snapshot.active_source == "autonomy"
                ):
                    return
                if (
                    snapshot is None
                    or snapshot.manager_epoch != identity.manager_epoch
                    or (
                        snapshot.transition_sequence >= resumed_sequence
                        and snapshot.authority
                        != ControlAuthorityState.AUTONOMY
                    )
                ):
                    raise RuntimeError(
                        "control authority was superseded before RESUME "
                        "became visible"
                    )
                condition.wait(timeout=remaining())

    def _clear_internal_stage_switch(self) -> None:
        with self._state_lock:
            self._internal_stage_switch = None

    def _matches_internal_stage_switch_revocation(
        self,
        snapshot: AuthoritySnapshot,
    ) -> bool:
        """只豁免由本节点发起且身份完全匹配的内部 HOLD。

        reason token 只用于捕获首个 HOLD 边沿；拿到 exact epoch/revocation
        后，ACK 会改变 reason 和 transition_sequence，因此后续状态改用
        pending revocation 身份匹配。KEYBOARD/ESTOP 永远不会走此豁免。
        """

        intent = getattr(self, "_internal_stage_switch", None)
        if intent is None or snapshot.authority != ControlAuthorityState.HOLD:
            return False
        identity = QuiescenceIdentity(
            snapshot.manager_epoch,
            snapshot.pending_autonomy_revocation_sequence,
        )
        if intent.identity is not None:
            return identity == intent.identity
        exact_hold = (
            snapshot.manager_epoch == intent.manager_epoch
            and snapshot.transition_sequence
            > intent.transition_sequence_floor
            and snapshot.pending_autonomy_revocation_sequence
            == snapshot.transition_sequence
            and not snapshot.autonomy_quiescence_acknowledged
            and snapshot.reason == intent.reason
        )
        if exact_hold:
            # callback 可能早于 service future；在看到唯一 reason token 时立即
            # 固化身份，避免紧随其后的 ACK heartbeat 被误判成用户接管。
            intent.identity = identity
        return exact_hold

    def _matches_internal_stage_switch_resume(
        self,
        snapshot: AuthoritySnapshot,
    ) -> bool:
        """识别本节点唯一的 RESUME 边沿，不放宽普通 AUTONOMY heartbeat。"""

        intent = getattr(self, "_internal_stage_switch", None)
        identity = None if intent is None else intent.identity
        return (
            intent is not None
            and identity is not None
            and snapshot.authority == ControlAuthorityState.AUTONOMY
            and snapshot.manager_epoch == identity.manager_epoch
            and snapshot.transition_sequence
            > identity.revocation_sequence
            and snapshot.pending_autonomy_revocation_sequence == 0
            and not snapshot.autonomy_quiescence_acknowledged
            and not snapshot.estop_latched
            and snapshot.active_source == "autonomy"
            and snapshot.reason == f"{intent.reason}:resume"
        )

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
            if update.lease_discontinuity:
                self._authority_lease_failure_reported = True
                SessionOrchestratorNode._cancel_active_automatic_mission(
                    self, "authority_lease_discontinuity"
                )
                self._fail_autonomy_quiescence(
                    TimeoutError(
                        "authority manager lease_discontinuity before "
                        "typed revocation"
                    )
                )
            return
        self._authority_lease_failure_reported = False
        with self._state_lock:
            self._authority_snapshot = snapshot
            authority_condition = getattr(
                self, "_authority_condition", None
            )
            if authority_condition is not None:
                authority_condition.notify_all()
            internal_stage_switch_revocation = (
                SessionOrchestratorNode
                ._matches_internal_stage_switch_revocation(
                    self, snapshot
                )
            )
            internal_stage_switch_resume = (
                SessionOrchestratorNode
                ._matches_internal_stage_switch_resume(
                    self, snapshot
                )
            )
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
        if (
            authority != AUTONOMY
            or (was_observed and not was_fresh_autonomy)
        ) and not (
            internal_stage_switch_revocation
            or internal_stage_switch_resume
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
        persistent_runtime = getattr(
            self, "_persistent_runtime_enabled", False
        )
        generation = (
            self._arm_readiness_profile("persistent_mapping_stage")
            if persistent_runtime
            else self._readiness_generation()
        )
        self.transition(SessionPhase.STARTING_MAPPING, detail="starting mapping stage")
        if persistent_runtime:
            # base 拥有 Gazebo/robot/RViz/Agent，只在会话启动一次；mapping 是
            # 可替换 stage，之后切 Nav2 时不能把机器人世界一并销毁。
            self._manager.start_base()
            # Bridge ACTIVE 是 base 与 stage 的明确提交边界。mapping stage 会
            # 继续制造大量 DDS endpoint，不能与首轮 lifecycle 转换竞争。
            self._wait_persistent_base_control_ready()
            self._manager.start_mapping()
            self._wait_persistent_stage_executor_ready()
        else:
            self._manager.start("mapping")
        self._wait_for_new_ready(generation)
        if persistent_runtime:
            self._wait_persistent_velocity_pipeline_ready()
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
        persistent_runtime = getattr(
            self, "_persistent_runtime_enabled", False
        )
        map_yaml_path: Path | None = None
        if persistent_runtime:
            saved_map_path = self._fsm.snapshot.map_yaml_path
            if not saved_map_path:
                raise RuntimeError(
                    "persistent navigation requires a saved map path"
                )
            map_yaml_path = Path(saved_map_path)
        authority_transaction = (
            persistent_runtime
            and getattr(self, "_authority_gate_enabled", False)
        )
        switch_identity: QuiescenceIdentity | None = None
        stage_replaced = False
        try:
            if authority_transaction:
                # 进程替换不能在 AUTONOMY 下进行：先进入 exact HOLD，并复用
                # Explore/Nav2 terminal、priority STOP、fresh zero、typed ACK 屏障。
                switch_identity = self._enter_internal_stage_switch_hold(
                    request
                )
                if getattr(request, "canceled", False):
                    raise AutomaticMissionCancelled(
                        "control authority changed during stage switch"
                    )
            if persistent_runtime:
                # 先彻底停止 mapping owner，再创建 navigation 代际。该顺序
                # 防止新 AMCL/map 首帧在缓存清理前到达又被擦除。
                stage_replaced = True
                self._manager.stop_stage()
            else:
                self._manager.stop()
            with self._map_pose_condition:
                # stop_stage 已等待旧进程退出。先关闭输入闸门并开启新代际，
                # 之后才允许 map_server/AMCL 启动，杜绝跨阶段快照。
                self._navigation_inputs_enabled = False
                self._navigation_input_generation += 1
                self._latest_occupancy = None
                self._latest_map_pose_xy = None
                self._latest_occupancy_generation = -1
                self._latest_map_pose_generation = -1
                self._map_pose_condition.notify_all()
            if getattr(request, "canceled", False):
                raise AutomaticMissionCancelled(
                    "control authority changed after mapping stopped"
                )
            if persistent_runtime:
                assert map_yaml_path is not None
                self._manager.prepare_navigation(map_yaml_path)
            generation = (
                self._arm_readiness_profile(
                    "persistent_navigation_stage"
                )
                if persistent_runtime
                else self._readiness_generation()
            )
            self.transition(
                SessionPhase.STARTING_NAVIGATION,
                detail="starting saved-map AMCL/Nav2 stage",
            )
            # 进程切换很快时 transient-local state topic 只保证新订阅者拿到“最新状态”，
            # 不保证测试或 UI 一定调度到每个中间快照；Action feedback因此同步承载阶段进度。
            self.feedback(request, 0.8)
            with self._map_pose_condition:
                self._navigation_inputs_enabled = True
                self._map_pose_condition.notify_all()
            self._manager.start("navigation")
            if persistent_runtime:
                self._wait_persistent_stage_executor_ready()
            self._wait_for_new_ready(generation)
            if getattr(request, "canceled", False):
                raise AutomaticMissionCancelled(
                    "control authority changed before navigation readiness"
                )
            if persistent_runtime:
                # readiness profile 只证明 stage 聚合器完成；还必须看到
                # map_server/AMCL ACTIVE 与本代 fresh map/pose，才能恢复自治。
                self._wait_persistent_navigation_ready(request)
            if switch_identity is not None:
                if getattr(request, "canceled", False):
                    raise AutomaticMissionCancelled(
                        "control authority changed before navigation resume"
                    )
                # 只有新 navigation stage 已 ready 才消费一次性 ACK 许可。
                # KEYBOARD/ESTOP 若并发到达，service 会拒绝且保持非自治状态。
                self._resume_internal_stage_switch(switch_identity)
        except AutomaticMissionCancelled as exc:
            with self._map_pose_condition:
                self._navigation_inputs_enabled = False
                self._map_pose_condition.notify_all()
            if persistent_runtime and stage_replaced:
                self._manager.stop_stage()
                self._navigation_startup_failure_pending = True
                raise _PersistentStageSwitchInterrupted(
                    "control authority changed after mapping stage was "
                    "replaced; persistent session remains safely stopped"
                ) from exc
            raise
        except Exception as exc:
            with self._map_pose_condition:
                self._navigation_inputs_enabled = False
                self._map_pose_condition.notify_all()
            if (
                authority_transaction
                and self._autonomy_quiescence.snapshot().state
                is QuiescenceState.FAILED
            ):
                # 屏障失败意味着 terminal/STOP/zero/ACK 至少一项不可证明；
                # 外层 mission 的通用恢复分支不得把 FAILED 覆盖成 MAPPING。
                self._navigation_startup_failure_pending = True
            # HOLD/ACK 在 stage 替换前失败时仍保留可诊断的 mapping stage；
            # 只有已开始替换，或旧兼容模式，才需要回收半启动进程。
            if not persistent_runtime or stage_replaced:
                self._cleanup_failed_navigation_startup(exc)
            raise
        finally:
            if authority_transaction:
                self._clear_internal_stage_switch()
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
                if getattr(self, "_persistent_runtime_enabled", False):
                    # STOP_SESSION 是整场演示的终点，必须释放 base 所有权；
                    # 普通 mapping→navigation 切换只允许 stop_stage。
                    self._manager.shutdown()
                else:
                    self._manager.stop()
                self.transition(SessionPhase.STOPPED, detail="session stopped")
            request.success = True
            if not request.message:
                request.message = "session command completed"
        except AutomaticMissionCancelled as exc:
            request.message = str(exc)
            runtime_failure = getattr(self, "_child_runtime_failure", "")
            if runtime_failure:
                # base 崩溃属于不可恢复的会话故障，不是普通用户取消。watchdog
                # 已进入 FAILED 后，worker 绝不能再按 manager.stage 虚报 MAPPING。
                request.message = runtime_failure
                if request.command == SessionCommand.RUN_AUTOMATIC_MISSION:
                    self._mission_outcome = SlamSessionState.MISSION_FAILED
                    self._mission_message = runtime_failure
                if self._fsm.snapshot.phase != SessionPhase.FAILED:
                    self.transition(
                        SessionPhase.FAILED,
                        detail=runtime_failure,
                    )
                self.get_logger().error(runtime_failure)
                return
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

    def _request_process_shutdown(self) -> None:
        """让 fatal startup failure 结束 ROS 进程，而非留下 FAILED 空壳."""

        if rclpy.ok():
            rclpy.shutdown()

    def _fail_mapping_startup(self, error: BaseException) -> None:
        """启动失败时原子拒绝新工作、释放等待者并回收整场运行时."""

        detail = f"mapping startup failed: {error}"
        self._stopping.set()
        with self._state_lock:
            self._mission_outcome = SlamSessionState.MISSION_FAILED
            self._mission_message = detail
        self.transition(SessionPhase.FAILED, detail=detail)

        # Action server 在 base 启动期间已经可见；失败边沿前进入队列的请求必须
        # 得到明确终态，不能因 worker 直接返回而永久阻塞 execute callback。
        while True:
            try:
                pending = self._requests.get_nowait()
            except queue.Empty:
                break
            if pending is None:
                continue
            pending.success = False
            pending.message = detail
            pending.completed.set()

        try:
            if getattr(self, "_persistent_runtime_enabled", False):
                self._manager.shutdown()
            else:
                self._manager.stop()
        except Exception as cleanup_error:
            note = f"mapping startup cleanup failed: {cleanup_error}"
            add_note = getattr(error, "add_note", None)
            if callable(add_note):
                add_note(note)
            self.get_logger().error(note)
        finally:
            # 仅发布 FAILED 仍会让 authority manager 和外层 launch 误以为会话活着；
            # 结束 rclpy context 后，脚本 trap 才能回收 session 级 manager。
            self._request_process_shutdown()

    def _worker_loop(self) -> None:
        try:
            self._start_mapping()
        except Exception as exc:
            SessionOrchestratorNode._fail_mapping_startup(self, exc)
            return
        while not self._stopping.is_set():
            try:
                request = self._requests.get(timeout=0.2)
            except queue.Empty:
                continue
            if request is None:
                break
            self._execute_request(request)

    def _check_child_process(self) -> None:
        if self._stopping.is_set():
            return
        persistent_runtime = getattr(
            self, "_persistent_runtime_enabled", False
        )
        operation_active = self._operation_active.is_set()
        if persistent_runtime:
            role, code = self._manager.unexpected_exit()
            # stage/explorer 在任务内部会被主动替换或回收；但常驻 base 无论
            # 是否正在执行任务都不应退出，不能让 operation_active 掩盖其崩溃。
            if operation_active and role not in {None, "base"}:
                return
            exited = role is not None
            owner = role or "runtime"
        else:
            if operation_active:
                return
            exited, code = self._manager.exited_unexpectedly()
            owner = self._manager.stage or "stage"
        if exited and self._fsm.snapshot.phase not in {
            SessionPhase.STOPPED,
            SessionPhase.FAILED,
        }:
            failure_detail = f"{owner} process exited code={code}"
            if persistent_runtime and owner == "base":
                with self._state_lock:
                    self._child_runtime_failure = failure_detail
                    active_request = self._active_request
                    if active_request is not None:
                        active_request.canceled = True
                        if not active_request.message:
                            active_request.message = (
                                "persistent base process exited"
                            )
            self.transition(
                SessionPhase.FAILED,
                detail=failure_detail,
            )

    def close(self) -> None:
        self._stopping.set()
        # 先在同一状态锁下取消 worker 当前请求，再关闭 manager。配合 manager
        # 的 terminal lifecycle lock，可保证 shutdown 后不会迟到 spawn Nav2。
        with self._state_lock:
            active_request = self._active_request
            if active_request is not None:
                active_request.canceled = True
                if not active_request.message:
                    active_request.message = "session is closing"
        try:
            self._requests.put_nowait(None)
        except queue.Full:
            pass
        if getattr(self, "_persistent_runtime_enabled", False):
            self._manager.shutdown()
        else:
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
