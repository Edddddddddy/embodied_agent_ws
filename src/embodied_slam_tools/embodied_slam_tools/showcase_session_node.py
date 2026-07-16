"""单终端编排真实感语音建图、保存地图和导航阶段。"""

from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
import queue
import signal
import subprocess
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
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import FollowWaypoints, NavigateToPose
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
import yaml

try:
    from explore_lite_msgs.msg import ExploreStatus
except ImportError:  # 可选运行时由 setup_frontier_exploration.sh 安装。
    ExploreStatus = None

from .showcase_session import (
    SessionCommand,
    SessionPhase,
    ShowcaseSessionStateMachine,
    exploration_completion_reason,
    is_automatic_mission_cancel_text,
    parse_session_command,
)


_ACTION_TYPES = {
    "stop": RobotCommand.STOP,
    "move": RobotCommand.MOVE,
    "turn": RobotCommand.TURN,
    "navigate_to": RobotCommand.NAVIGATE_TO,
    "follow_waypoints": RobotCommand.FOLLOW_WAYPOINTS,
}


@dataclass
class CommandRequest:
    command: SessionCommand
    source: str
    goal_handle: object | None = None
    completed: threading.Event = field(default_factory=threading.Event)
    success: bool = False
    message: str = ""
    canceled: bool = False


class AutomaticMissionCancelled(RuntimeError):
    """用户急停或取消高层任务，不应被误报成系统故障。"""


class StageProcessManager:
    """只负责子进程生命周期；状态策略留在 SessionOrchestratorNode。"""

    def __init__(
        self,
        workspace: Path,
        mode: str,
        map_prefix: Path,
        *,
        stop_timeout_s: float,
        dry_run: bool,
    ) -> None:
        self.workspace = workspace
        self.mode = mode
        self.map_prefix = map_prefix
        self.stop_timeout_s = stop_timeout_s
        self.dry_run = dry_run
        self.process: subprocess.Popen | None = None
        self.explorer_process: subprocess.Popen | None = None
        self.stage = ""

    def _environment(self) -> dict[str, str]:
        environment = os.environ.copy()
        environment["WORKSPACE"] = str(self.workspace)
        environment["SHOWCASE_SESSION_DIR"] = str(self.map_prefix.parent)
        environment["SHOWCASE_MAP_PREFIX"] = str(self.map_prefix)
        return environment

    def command(self, stage: str) -> list[str]:
        return [
            "bash",
            str(self.workspace / "scripts/voice_slam_nav_showcase.sh"),
            stage,
            self.mode,
        ]

    def start(self, stage: str) -> None:
        if self.process is not None and self.process.poll() is None:
            raise RuntimeError(f"stage process already running: {self.stage}")
        command = self.command(stage)
        if self.dry_run:
            print("DRY RUN stage:", " ".join(command), flush=True)
            self.stage = stage
            self.process = None
            return
        self.process = subprocess.Popen(
            command,
            cwd=self.workspace,
            env=self._environment(),
            start_new_session=True,
        )
        self.stage = stage

    def save_map(self) -> str:
        command = self.command("save")
        if self.dry_run:
            print("DRY RUN save:", " ".join(command), flush=True)
            return str(self.map_prefix.with_suffix(".yaml"))
        completed = subprocess.run(
            command,
            cwd=self.workspace,
            env=self._environment(),
            text=True,
            capture_output=True,
            timeout=35.0,
            check=False,
        )
        if completed.stdout:
            print(completed.stdout, end="", flush=True)
        if completed.returncode != 0:
            raise RuntimeError(
                completed.stderr.strip()
                or f"map_saver exited with code {completed.returncode}"
            )
        yaml_path = self.map_prefix.with_suffix(".yaml")
        image_path = self.map_prefix.with_suffix(".pgm")
        if not yaml_path.is_file() or not image_path.is_file():
            raise RuntimeError("map_saver returned success without YAML/PGM artifacts")
        return str(yaml_path)

    def start_explorer(self, config_path: Path) -> None:
        if self.dry_run:
            print(
                f"DRY RUN frontier explorer: config={config_path}",
                flush=True,
            )
            return
        check = subprocess.run(
            ["ros2", "pkg", "prefix", "explore_lite"],
            cwd=self.workspace,
            env=self._environment(),
            capture_output=True,
            text=True,
            check=False,
        )
        if check.returncode != 0:
            raise RuntimeError(
                "Explore Lite is not installed; run "
                "bash scripts/setup_frontier_exploration.sh"
            )
        if self.explorer_process is not None and self.explorer_process.poll() is None:
            raise RuntimeError("frontier explorer is already running")
        self.explorer_process = subprocess.Popen(
            [
                "ros2",
                "run",
                "explore_lite",
                "explore",
                "--ros-args",
                "--params-file",
                str(config_path),
            ],
            cwd=self.workspace,
            env=self._environment(),
            start_new_session=True,
        )

    @staticmethod
    def _stop_process(process: subprocess.Popen | None, timeout_s: float) -> None:
        if process is None or process.poll() is not None:
            return
        # `ros2 run` 是 Python 包装进程，真正的 C++ explore 是其子进程；只 terminate
        # 包装器会留下孤儿 explore，并在定位阶段继续发送 frontier goal。
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=timeout_s)
            return
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def stop_explorer(self) -> None:
        process = self.explorer_process
        self.explorer_process = None
        if self.dry_run:
            return
        self._stop_process(process, 5.0)

    def explorer_exited_unexpectedly(self) -> tuple[bool, int | None]:
        if self.dry_run or self.explorer_process is None:
            return False, None
        code = self.explorer_process.poll()
        return code is not None, code

    def stop(self) -> None:
        self.stop_explorer()
        process = self.process
        self.process = None
        self.stage = ""
        if self.dry_run or process is None or process.poll() is not None:
            return

        # 先只通知父脚本，让 continuous_nav2_voice_control.sh 的 trap 按顺序关闭
        # launch/monitor；超时后才升级为进程组信号，避免 Gazebo/LLM 收到双重中断。
        process.terminate()
        try:
            process.wait(timeout=self.stop_timeout_s)
            return
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                # SIGKILL 后仍未回收通常表示僵尸进程由上层 init 接管；关闭路径
                # 不应因此掩盖真正的会话结果或阻塞 ROS 节点退出。
                pass

    def exited_unexpectedly(self) -> tuple[bool, int | None]:
        if self.dry_run or self.process is None:
            return False, None
        code = self.process.poll()
        return code is not None, code


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
        self._explorer_config_path = (
            workspace / "src/embodied_simulation/config" / explorer_config_name
        )
        self._exploration_timeout_s = float(
            automatic_config.get("timeout_s", 300.0)
        )
        self._exploration_min_runtime_s = float(
            automatic_config.get("min_runtime_s", 15.0)
        )
        self._exploration_stable_map_s = float(
            automatic_config.get("stable_map_s", 20.0)
        )
        self._exploration_min_growth_cells = int(
            automatic_config.get("min_growth_cells", 40)
        )
        self._mission_navigation_timeout_s = float(
            automatic_config.get("navigation_timeout_s", 330.0)
        )
        self._exploration_completion_status = str(
            automatic_config.get("completion_status", "exploration_complete")
        )
        acceptance = self._mission_plan.get("acceptance", {})
        self._min_known_map_cells = int(acceptance.get("min_known_map_cells", 0))
        self._min_occupied_map_cells = int(
            acceptance.get("min_occupied_map_cells", 0)
        )
        readiness_topic = str(
            self.declare_parameter("readiness_topic", "/system/readiness").value
        )
        self._startup_timeout_s = float(
            self.declare_parameter("startup_timeout_s", 120.0).value
        )
        stop_timeout_s = float(
            self.declare_parameter("stop_timeout_s", 15.0).value
        )
        dry_run = bool(self.declare_parameter("dry_run", False).value)
        self._dry_run_exploration_delay_s = float(
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
        self._map_stats: dict[str, int] | None = None
        self._best_known_map_cells = 0
        self._last_map_growth_at = time.monotonic()
        self._exploration_completion_reason = ""
        self._explore_condition = threading.Condition()
        self._explore_status = ""

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

    def _transition(self, phase: SessionPhase, **kwargs) -> None:
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
        stats = {
            "known_cells": sum(1 for value in message.data if value >= 0),
            "occupied_cells": sum(1 for value in message.data if value >= 65),
        }
        with self._explore_condition:
            self._map_stats = stats
            if (
                stats["known_cells"]
                >= self._best_known_map_cells + self._exploration_min_growth_cells
            ):
                self._best_known_map_cells = stats["known_cells"]
                self._last_map_growth_at = time.monotonic()
            self._explore_condition.notify_all()

    def _on_explore_status(self, message: Any) -> None:
        with self._explore_condition:
            self._explore_status = str(message.status)
            self._explore_condition.notify_all()

    def _cancel_automatic_motion(self) -> None:
        self._agent_text_pub.publish(String(data="停下"))
        self._manager.stop_explorer()

    def _run_agent_text_action(
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

    def _wait_for_frontier_completion(self, request: CommandRequest) -> None:
        if self._dry_run:
            deadline = time.monotonic() + self._dry_run_exploration_delay_s
            while time.monotonic() < deadline:
                if request.canceled:
                    self._cancel_automatic_motion()
                    raise AutomaticMissionCancelled("automatic mission canceled")
                time.sleep(0.05)
            return
        if ExploreStatus is None:
            raise RuntimeError(
                "explore_lite_msgs is unavailable; run "
                "bash scripts/setup_frontier_exploration.sh"
            )
        started_at = time.monotonic()
        deadline = started_at + self._exploration_timeout_s
        with self._explore_condition:
            while time.monotonic() < deadline:
                if request.canceled:
                    self._cancel_automatic_motion()
                    raise AutomaticMissionCancelled("automatic mission canceled")
                elapsed = time.monotonic() - started_at
                stats = self._map_stats or {}
                if (
                    self._explore_status == self._exploration_completion_status
                    and elapsed >= self._exploration_min_runtime_s
                ):
                    if stats.get("known_cells", 0) < self._min_known_map_cells:
                        raise RuntimeError(
                            "frontier exploration ended before known-cell threshold"
                        )
                    if stats.get("occupied_cells", 0) < self._min_occupied_map_cells:
                        raise RuntimeError(
                            "frontier exploration ended before occupied-cell threshold"
                        )
                completion_reason = exploration_completion_reason(
                    status=self._explore_status,
                    completion_status=self._exploration_completion_status,
                    elapsed_s=elapsed,
                    min_runtime_s=self._exploration_min_runtime_s,
                    known_cells=stats.get("known_cells", 0),
                    occupied_cells=stats.get("occupied_cells", 0),
                    min_known_cells=self._min_known_map_cells,
                    min_occupied_cells=self._min_occupied_map_cells,
                    seconds_since_map_growth=(
                        time.monotonic() - self._last_map_growth_at
                    ),
                    stable_map_s=self._exploration_stable_map_s,
                )
                if completion_reason is not None:
                    # 真实室内图常残留家具背后或墙外的不可达 frontier。覆盖达标且地图
                    # 长时间不再增长时继续恢复只会空转，因此在可审计阈值处结束任务。
                    self._exploration_completion_reason = completion_reason
                    self.get_logger().info(
                        f"frontier exploration complete reason={completion_reason}: "
                        f"known={stats.get('known_cells', 0)} "
                        f"occupied={stats.get('occupied_cells', 0)}"
                    )
                    return
                exited, code = self._manager.explorer_exited_unexpectedly()
                if exited:
                    raise RuntimeError(
                        f"frontier explorer exited unexpectedly code={code}"
                    )
                self._explore_condition.wait(
                    timeout=min(0.5, max(0.0, deadline - time.monotonic()))
                )
        raise TimeoutError("frontier exploration did not complete before timeout")

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

    def _wait_for_navigation_action_servers(self, request: CommandRequest) -> None:
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
                future = client.call_async(GetState.Request())
                while not future.done() and time.monotonic() < deadline:
                    if request.canceled:
                        future.cancel()
                        self._cancel_automatic_motion()
                        raise AutomaticMissionCancelled("automatic mission canceled")
                    time.sleep(0.05)
                if future.done() and future.result() is not None:
                    if (
                        future.result().current_state.id
                        == State.PRIMARY_STATE_ACTIVE
                    ):
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

    def _feedback(self, request: CommandRequest, progress: float) -> None:
        if request.goal_handle is None:
            return
        feedback = ManageSlamSession.Feedback()
        feedback.progress = progress
        feedback.state = self._state_message()
        request.goal_handle.publish_feedback(feedback)

    def _start_mapping(self) -> None:
        generation = self._readiness_generation()
        self._transition(SessionPhase.STARTING_MAPPING, detail="starting mapping stage")
        self._manager.start("mapping")
        self._wait_for_new_ready(generation)
        self._transition(SessionPhase.MAPPING, detail="mapping ready; explore by voice")

    def _save_map(self, request: CommandRequest) -> None:
        if self._fsm.snapshot.map_saved:
            request.message = "map already saved"
            return
        self._transition(SessionPhase.SAVING_MAP, detail="saving /map")
        self._feedback(request, 0.35)
        try:
            yaml_path = self._manager.save_map()
        except Exception as exc:
            # 保存失败不杀死仍可用的 mapping stage，允许用户修正后重新说“保存地图”。
            self._transition(
                SessionPhase.MAPPING,
                detail=f"map save failed: {exc}",
                map_saved=False,
                map_yaml_path="",
            )
            raise
        self._transition(
            SessionPhase.MAP_SAVED,
            detail="map saved; ready to start navigation",
            map_saved=True,
            map_yaml_path=yaml_path,
        )
        self._feedback(request, 0.55)

    def _start_navigation(self, request: CommandRequest) -> None:
        if self._fsm.snapshot.phase == SessionPhase.NAVIGATING:
            request.message = "navigation already active"
            return
        self._transition(
            SessionPhase.SWITCHING_TO_NAVIGATION,
            detail="stopping mapping stage",
        )
        self._feedback(request, 0.65)
        self._manager.stop()
        generation = self._readiness_generation()
        self._transition(
            SessionPhase.STARTING_NAVIGATION,
            detail="starting saved-map AMCL/Nav2 stage",
        )
        # 进程切换很快时 transient-local state topic 只保证新订阅者拿到“最新状态”，
        # 不保证测试或 UI 一定调度到每个中间快照；Action feedback 因此同步承载阶段进度。
        self._feedback(request, 0.8)
        self._manager.start("navigation")
        self._wait_for_new_ready(generation)
        self._transition(
            SessionPhase.NAVIGATING,
            detail="navigation ready; semantic goals accepted",
        )
        self._feedback(request, 1.0)

    def _run_automatic_mission(self, request: CommandRequest) -> None:
        """一次高层命令完成 frontier 探索、存图、重定位和语义巡航。"""

        self._transition(
            SessionPhase.AUTOMATIC_MAPPING,
            detail="frontier exploration running",
        )
        self._feedback(request, 0.05)
        with self._explore_condition:
            self._explore_status = ""
            self._best_known_map_cells = 0
            self._last_map_growth_at = time.monotonic()
            self._exploration_completion_reason = ""
        self._manager.start_explorer(self._explorer_config_path)
        try:
            self._wait_for_frontier_completion(request)
        finally:
            self._manager.stop_explorer()
        stats = self._map_stats or {}
        self._transition(
            SessionPhase.MAPPING,
            detail=(
                "frontier exploration complete "
                f"reason={self._exploration_completion_reason or 'unknown'} "
                f"known={stats.get('known_cells', 0)} "
                f"occupied={stats.get('occupied_cells', 0)}"
            ),
        )
        self._feedback(request, 0.5)

        self._save_map(request)
        if request.canceled:
            raise AutomaticMissionCancelled("automatic mission canceled")
        self._start_navigation(request)
        self._wait_for_navigation_action_servers(request)
        self._transition(
            SessionPhase.AUTOMATIC_NAVIGATING,
            detail="automatic semantic navigation running",
        )
        self._feedback(request, 0.82)

        navigation = self._mission_plan.get("navigation_mission", {})
        self._run_agent_text_action(
            request,
            text=str(navigation["navigate_text"]),
            expected_action="navigate_to",
            timeout_s=self._mission_navigation_timeout_s,
        )
        self._feedback(request, 0.9)
        self._run_agent_text_action(
            request,
            text=str(navigation["patrol_text"]),
            expected_action="follow_waypoints",
            timeout_s=self._mission_navigation_timeout_s,
        )
        self._transition(
            SessionPhase.MISSION_COMPLETED,
            detail="automatic mapping and navigation mission completed",
        )
        self._feedback(request, 1.0)

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
                self._run_automatic_mission(request)
            if request.command in {
                SessionCommand.SAVE_MAP,
                SessionCommand.SAVE_AND_START_NAVIGATION,
            }:
                self._save_map(request)
            if request.canceled:
                request.message = "canceled before stage switch"
                return
            if request.command in {
                SessionCommand.START_NAVIGATION,
                SessionCommand.SAVE_AND_START_NAVIGATION,
            }:
                self._start_navigation(request)
            if request.command == SessionCommand.STOP_SESSION:
                self._transition(SessionPhase.STOPPING, detail="stopping session")
                self._manager.stop()
                self._transition(SessionPhase.STOPPED, detail="session stopped")
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
            self._transition(recovery_phase, detail=request.message)
            self.get_logger().warning(request.message)
        except Exception as exc:
            request.message = str(exc)
            if request.command == SessionCommand.RUN_AUTOMATIC_MISSION:
                recovery_phase = (
                    SessionPhase.NAVIGATING
                    if self._manager.stage == "navigation"
                    else SessionPhase.MAPPING
                )
                self._transition(
                    recovery_phase,
                    detail=f"automatic mission failed: {exc}",
                )
            elif self._fsm.snapshot.phase != SessionPhase.MAPPING:
                self._transition(SessionPhase.FAILED, detail=f"session failed: {exc}")
            self.get_logger().error(request.message)
        finally:
            self._active_request = None
            self._operation_active.clear()
            request.completed.set()

    def _worker_loop(self) -> None:
        try:
            self._start_mapping()
        except Exception as exc:
            self._transition(SessionPhase.FAILED, detail=f"mapping startup failed: {exc}")
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
            self._transition(
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
