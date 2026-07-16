"""单终端编排真实感语音建图、保存地图和导航阶段。"""

from __future__ import annotations

import os
from pathlib import Path
import queue
import threading
import time
from typing import Any

from embodied_agent_interfaces.action import ManageSlamSession
from embodied_agent_interfaces.msg import (
    RobotCommand,
    RobotCommandResult,
    SlamSessionState,
    SystemReadiness,
)
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import FollowWaypoints, NavigateToPose
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
import yaml

try:
    from explore_lite_msgs.msg import ExploreStatus
except ImportError:  # 可选运行时由 setup_frontier_exploration.sh 安装。
    ExploreStatus = None

from .frontier_monitor import FrontierExplorationMonitor, FrontierMonitorConfig
from .mapping_evidence import MappingEvidenceTracker
from .mission_executor import (
    AutomaticMissionCancelled,
    AutomaticMissionExecutor,
    AutomaticMissionSpec,
    CommandRequest,
)
from .stage_process_manager import StageProcessManager
from .showcase_session import (
    SessionCommand,
    SessionPhase,
    ShowcaseSessionStateMachine,
    is_automatic_mission_cancel_text,
    is_truncated_automatic_mission_text,
    parse_mapping_bootstrap_route,
    parse_session_command,
)


_ACTION_TYPES = {
    "stop": RobotCommand.STOP,
    "move": RobotCommand.MOVE,
    "turn": RobotCommand.TURN,
    "navigate_to": RobotCommand.NAVIGATE_TO,
    "follow_waypoints": RobotCommand.FOLLOW_WAYPOINTS,
}


def _get_lifecycle_state(client, timeout_s: float) -> int | None:
    """在非 executor 工作线程中同步查询 lifecycle，超时返回 ``None``。"""

    response = client.call(GetState.Request(), timeout_sec=timeout_s)
    if response is None:
        return None
    return int(response.current_state.id)


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
        self._mission_plan = yaml.safe_load(
            mission_plan_path.read_text(encoding="utf-8")
        )
        if not isinstance(self._mission_plan, dict):
            raise ValueError("mission_plan must contain a YAML mapping")
        automatic_config = self._mission_plan.get("automatic_exploration", {})
        if not isinstance(automatic_config, dict):
            raise ValueError("automatic_exploration must be a YAML mapping")
        explorer_config_name = str(
            automatic_config.get("config", "frontier_exploration.yaml")
        )
        explorer_config_path = (
            workspace / "src/embodied_simulation/config" / explorer_config_name
        )
        exploration_timeout_s = float(
            automatic_config.get("timeout_s", 300.0)
        )
        exploration_min_runtime_s = float(
            automatic_config.get("min_runtime_s", 15.0)
        )
        exploration_stable_map_s = float(
            automatic_config.get("stable_map_s", 20.0)
        )
        exploration_min_growth_cells = int(
            automatic_config.get("min_growth_cells", 40)
        )
        mission_navigation_timeout_s = float(
            automatic_config.get("navigation_timeout_s", 330.0)
        )
        exploration_completion_status = str(
            automatic_config.get("completion_status", "exploration_complete")
        )
        mapping_bootstrap_route = parse_mapping_bootstrap_route(
            automatic_config
        )
        mapping_bootstrap_action_timeout_s = float(
            automatic_config.get("bootstrap_action_timeout_s", 45.0)
        )
        acceptance = self._mission_plan.get("acceptance", {})
        min_known_map_cells = int(acceptance.get("min_known_map_cells", 0))
        min_occupied_map_cells = int(
            acceptance.get("min_occupied_map_cells", 0)
        )
        min_mapping_path_m = float(
            acceptance.get("min_mapping_path_m", 0.0)
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
        if scan_startup_timeout_s <= 0.0:
            raise ValueError("scan_startup_timeout_s must be positive")
        stop_timeout_s = float(
            self.declare_parameter("stop_timeout_s", 15.0).value
        )
        dry_run = bool(self.declare_parameter("dry_run", False).value)
        dry_run_exploration_delay_s = float(
            self.declare_parameter("dry_run_exploration_delay_s", 0.0).value
        )
        self._fsm = ShowcaseSessionStateMachine()
        self._manager = StageProcessManager(
            workspace,
            mode,
            map_prefix,
            stop_timeout_s=stop_timeout_s,
            dry_run=dry_run,
        )
        self._mapping_evidence = MappingEvidenceTracker(
            exploration_min_growth_cells
        )
        self._frontier_monitor = FrontierExplorationMonitor(
            self._mapping_evidence,
            self._manager,
            FrontierMonitorConfig(
                timeout_s=exploration_timeout_s,
                min_runtime_s=exploration_min_runtime_s,
                stable_map_s=exploration_stable_map_s,
                completion_status=exploration_completion_status,
                min_known_cells=min_known_map_cells,
                min_occupied_cells=min_occupied_map_cells,
                min_mapping_path_m=min_mapping_path_m,
                status_available=ExploreStatus is not None,
                dry_run=dry_run,
                dry_run_delay_s=dry_run_exploration_delay_s,
            ),
            cancel_motion=self._cancel_automatic_motion,
            log_info=self.get_logger().info,
        )
        navigation = self._mission_plan.get("navigation_mission", {})
        if not isinstance(navigation, dict):
            raise ValueError("navigation_mission must be a YAML mapping")
        self._automatic_mission_executor = AutomaticMissionExecutor(
            self,
            self._manager,
            self._mapping_evidence,
            AutomaticMissionSpec(
                explorer_config_path=explorer_config_path,
                bootstrap_route=tuple(mapping_bootstrap_route),
                scan_startup_timeout_s=scan_startup_timeout_s,
                bootstrap_action_timeout_s=(
                    mapping_bootstrap_action_timeout_s
                ),
                navigation_timeout_s=mission_navigation_timeout_s,
                navigate_text=str(navigation["navigate_text"]),
                patrol_text=str(navigation["patrol_text"]),
            ),
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
        self._agent_condition = threading.Condition()
        self._agent_candidates: list[tuple[int, str]] = []
        self._agent_results: dict[str, RobotCommandResult] = {}

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
            LaserScan,
            "/scan",
            self._on_scan,
            qos_profile_sensor_data,
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
        message = SlamSessionState()
        message.stamp = self.get_clock().now().to_msg()
        message.phase = int(snapshot.phase)
        message.map_saved = snapshot.map_saved
        message.map_yaml_path = snapshot.map_yaml_path
        message.detail = snapshot.detail
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
            self._fsm.transition(phase, **kwargs)
        self._publish_state()

    def _on_readiness(self, message: SystemReadiness) -> None:
        with self._ready_condition:
            self._ready_generation += 1
            self._latest_ready = bool(message.ready)
            self._ready_condition.notify_all()

    def _on_agent_candidate(self, message: RobotCommand) -> None:
        if not message.command_id:
            return
        with self._agent_condition:
            self._agent_candidates.append(
                (int(message.action_type), str(message.command_id))
            )
            self._agent_condition.notify_all()

    def _on_agent_result(self, message: RobotCommandResult) -> None:
        if not message.command_id:
            return
        with self._agent_condition:
            self._agent_results[str(message.command_id)] = message
            self._agent_condition.notify_all()

    def _on_map(self, message: OccupancyGrid) -> None:
        self._mapping_evidence.record_map(message.data)

    def _on_scan(self, _message: LaserScan) -> None:
        # SystemReadiness 只能证明图中关键节点已经启动；Gazebo 传感器首帧可能
        # 稍晚到达。显式记录首帧，避免 bootstrap 动作被安全执行器按 scan_timeout 拒绝。
        self._mapping_evidence.mark_scan_ready()

    def _on_odom(self, message: Odometry) -> None:
        """累计本次自动建图的真实里程，防止覆盖阈值让探索过早结束。"""

        position = message.pose.pose.position
        self._mapping_evidence.record_odom(position.x, position.y)

    def _on_explore_status(self, message: Any) -> None:
        self._mapping_evidence.record_explore_status(str(message.status))

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

        if self._dry_run:
            print(
                f"DRY RUN agent action: {text} -> {expected_action}",
                flush=True,
            )
            return
        expected_type = _ACTION_TYPES[expected_action]
        deadline = time.monotonic() + timeout_s
        while self._agent_text_pub.get_subscription_count() == 0:
            if request.canceled:
                self._cancel_automatic_motion()
                raise AutomaticMissionCancelled("automatic mission canceled")
            if time.monotonic() >= deadline:
                raise TimeoutError("Agent text input subscriber unavailable")
            time.sleep(0.1)
        with self._agent_condition:
            candidate_start = len(self._agent_candidates)
        self._agent_text_pub.publish(String(data=text))

        command_id = ""
        with self._agent_condition:
            while not command_id:
                if request.canceled:
                    self._cancel_automatic_motion()
                    raise AutomaticMissionCancelled("automatic mission canceled")
                for action_type, candidate_id in self._agent_candidates[candidate_start:]:
                    if action_type == expected_type:
                        command_id = candidate_id
                        break
                remaining = deadline - time.monotonic()
                if command_id or remaining <= 0.0:
                    break
                self._agent_condition.wait(timeout=min(0.2, remaining))
        if not command_id:
            raise TimeoutError(
                f"Agent did not publish {expected_action} for {text!r}"
            )

        with self._agent_condition:
            while command_id not in self._agent_results:
                if request.canceled:
                    self._cancel_automatic_motion()
                    raise AutomaticMissionCancelled("automatic mission canceled")
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError(
                        f"action result timeout for {expected_action}:{command_id}"
                    )
                self._agent_condition.wait(timeout=min(0.2, remaining))
            result = self._agent_results.pop(command_id)
        if not result.success:
            raise RuntimeError(
                f"{expected_action} failed status={result.status}: {result.message}"
            )

    def wait_for_frontier(self, request: CommandRequest) -> None:
        self._frontier_monitor.wait(request)

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
