"""自动建图导航任务事务；不依赖 rclpy 或 ROS 消息类型。"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
import threading
import time
from typing import Protocol

from .mapping_evidence import MappingEvidenceTracker
from .showcase_session import SessionCommand, SessionPhase


@dataclass
class CommandRequest:
    """跨 Action 回调与工作线程传递的一次会话命令。"""

    command: SessionCommand
    source: str
    goal_handle: object | None = None
    completed: threading.Event = field(default_factory=threading.Event)
    success: bool = False
    message: str = ""
    canceled: bool = False


class AutomaticMissionCancelled(RuntimeError):
    """用户急停或取消高层任务，不应被误报成系统故障。"""


def wait_for_required_event(
    event: threading.Event,
    timeout_s: float,
    is_canceled,
    *,
    poll_s: float = 0.05,
) -> bool:
    """可取消地等待运行时依赖；成功返回 True，超时返回 False。"""

    deadline = time.monotonic() + max(0.0, timeout_s)
    while True:
        if is_canceled():
            raise AutomaticMissionCancelled("automatic mission canceled")
        remaining = deadline - time.monotonic()
        if remaining <= 0.0:
            return event.is_set()
        if event.wait(timeout=min(poll_s, remaining)):
            return True


@dataclass(frozen=True, slots=True)
class AutomaticMissionSpec:
    """任务顺序和超时配置；ROS 参数与 YAML 在 Node Adapter 中完成解析。"""

    explorer_config_path: Path
    bootstrap_route: tuple[tuple[str, str, str], ...]
    scan_startup_timeout_s: float
    bootstrap_action_timeout_s: float
    navigation_timeout_s: float
    navigate_text: str
    patrol_text: str


class ExplorerProcessPort(Protocol):
    """任务层仅依赖 explorer 生命周期，不感知 subprocess 实现。"""

    dry_run: bool

    def start_explorer(self, config_path: Path) -> None: ...

    def stop_explorer(self) -> None: ...


class MissionRuntimePort(Protocol):
    """任务事务需要的最小 ROS 运行时端口。"""

    def transition(self, phase: SessionPhase, **kwargs) -> None: ...

    def feedback(self, request: CommandRequest, progress: float) -> None: ...

    def run_agent_action(
        self,
        request: CommandRequest,
        *,
        text: str,
        expected_action: str,
        timeout_s: float,
    ) -> None: ...

    def wait_for_frontier(self, request: CommandRequest) -> None: ...

    def save_map(self, request: CommandRequest) -> None: ...

    def start_navigation(self, request: CommandRequest) -> None: ...

    def wait_navigation_ready(self, request: CommandRequest) -> None: ...


class AutomaticMissionExecutor:
    """在一个事务中执行自动探索、存图、定位切换和语义巡航。"""

    def __init__(
        self,
        runtime: MissionRuntimePort,
        manager: ExplorerProcessPort,
        evidence: MappingEvidenceTracker,
        spec: AutomaticMissionSpec,
    ) -> None:
        self._runtime = runtime
        self._manager = manager
        self._evidence = evidence
        self._spec = spec

    def run(self, request: CommandRequest) -> None:
        # bootstrap 与 frontier 都属于本次建图里程；证据模块会过滤仿真重置跳变。
        self._evidence.begin_mapping_path()
        try:
            self._run_transaction(request)
        finally:
            # 即使首帧等待或 bootstrap 在 explorer 启动前失败，也不能让后续 odom
            # 继续污染本次事务；清理由深模块自己保证，不依赖 Node 调用者补救。
            self._evidence.finish_mapping_path()

    def _run_transaction(self, request: CommandRequest) -> None:
        self._runtime.transition(
            SessionPhase.AUTOMATIC_MAPPING,
            detail="automatic mapping bootstrap running",
        )
        self._runtime.feedback(request, 0.03)

        if not self._manager.dry_run and not wait_for_required_event(
            self._evidence.scan_ready,
            self._spec.scan_startup_timeout_s,
            lambda: request.canceled,
        ):
            raise TimeoutError(
                "mapping scan did not become ready before bootstrap"
            )

        route_size = max(1, len(self._spec.bootstrap_route))
        for index, (label, text, expected_action) in enumerate(
            self._spec.bootstrap_route
        ):
            if request.canceled:
                raise AutomaticMissionCancelled("automatic mission canceled")
            self._runtime.transition(
                SessionPhase.AUTOMATIC_MAPPING,
                detail=f"bootstrap {index + 1}/{route_size}: {label}",
            )
            self._runtime.run_agent_action(
                request,
                text=text,
                expected_action=expected_action,
                timeout_s=self._spec.bootstrap_action_timeout_s,
            )
            self._runtime.feedback(
                request,
                0.05 + 0.25 * (index + 1) / route_size,
            )

        self._runtime.transition(
            SessionPhase.AUTOMATIC_MAPPING,
            detail="frontier exploration running",
        )
        self._evidence.reset_exploration()
        self._manager.start_explorer(self._spec.explorer_config_path)
        try:
            self._runtime.wait_for_frontier(request)
        finally:
            # 任何失败或取消都必须停 explorer，否则定位阶段仍会收到旧目标。
            self._manager.stop_explorer()

        snapshot = self._evidence.snapshot()
        stats = snapshot.map_stats or {}
        self._runtime.transition(
            SessionPhase.MAPPING,
            detail=(
                "frontier exploration complete "
                f"reason={snapshot.exploration_completion_reason or 'unknown'} "
                f"known={stats.get('known_cells', 0)} "
                f"occupied={stats.get('occupied_cells', 0)} "
                f"path={snapshot.mapping_path_m:.2f}m"
            ),
        )
        self._runtime.feedback(request, 0.5)

        self._runtime.save_map(request)
        if request.canceled:
            raise AutomaticMissionCancelled("automatic mission canceled")
        self._runtime.start_navigation(request)
        self._runtime.wait_navigation_ready(request)
        self._runtime.transition(
            SessionPhase.AUTOMATIC_NAVIGATING,
            detail="automatic semantic navigation running",
        )
        self._runtime.feedback(request, 0.82)

        self._runtime.run_agent_action(
            request,
            text=self._spec.navigate_text,
            expected_action="navigate_to",
            timeout_s=self._spec.navigation_timeout_s,
        )
        self._runtime.feedback(request, 0.9)
        self._runtime.run_agent_action(
            request,
            text=self._spec.patrol_text,
            expected_action="follow_waypoints",
            timeout_s=self._spec.navigation_timeout_s,
        )
        self._runtime.transition(
            SessionPhase.MISSION_COMPLETED,
            # MAPPING 状态很短，最终状态必须重复结束原因，避免监控端因调度错过。
            detail=(
                "automatic mapping and navigation mission completed "
                f"exploration_reason="
                f"{snapshot.exploration_completion_reason or 'unknown'}"
            ),
        )
        self._runtime.feedback(request, 1.0)
