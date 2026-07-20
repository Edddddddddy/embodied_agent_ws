"""自动建图导航任务事务；不依赖 rclpy 或 ROS 消息类型。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
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


class EpochRecoveryDecision(str, Enum):
    """恢复扫描后，任务层对下一个 frontier epoch 的唯一决策。"""

    ADVANCE_EPOCH = "advance_epoch"
    COMPLETE_NO_GAIN = "complete_no_gain"
    FAIL_NO_GAIN = "fail_no_gain"
    FAIL_BUDGET_EXHAUSTED = "fail_budget_exhausted"


def decide_epoch_recovery(
    *,
    recovery_reason: str,
    map_gain_cells: int,
    minimum_gain_cells: int,
    recovery_attempts_remaining: int,
    available_frontier_count: int,
    active_goal_count: int,
    blacklisted_frontier_count: int,
) -> EpochRecoveryDecision:
    """只用实时地图与 typed telemetry 判断是否允许开启新 epoch。"""

    counters = (
        map_gain_cells,
        minimum_gain_cells,
        recovery_attempts_remaining,
        available_frontier_count,
        active_goal_count,
        blacklisted_frontier_count,
    )
    if any(value < 0 for value in counters):
        raise ValueError("epoch recovery counters must be non-negative")

    if map_gain_cells >= minimum_gain_cells:
        return (
            EpochRecoveryDecision.ADVANCE_EPOCH
            if recovery_attempts_remaining > 0
            else EpochRecoveryDecision.FAIL_BUDGET_EXHAUSTED
        )

    counters_are_clear = (
        available_frontier_count == 0
        and active_goal_count == 0
        and blacklisted_frontier_count == 0
    )
    if (
        recovery_reason
        == "recovery_required:frontier_attempts_exhausted"
        and counters_are_clear
    ):
        # 只有 provider 已证明本 epoch 尝试耗尽，且没有被隐藏的活动、可达或
        # 黑名单目标时，“扫描无增益”才是完成证据；其他 recovery 必须失败。
        return EpochRecoveryDecision.COMPLETE_NO_GAIN
    return EpochRecoveryDecision.FAIL_NO_GAIN


@dataclass(frozen=True, slots=True)
class UnknownWorldMissionSpec:
    """未知环境任务只包含传感器策略和通用采样参数，不包含场景坐标。"""

    explorer_config_path: Path
    scan_startup_timeout_s: float
    exploration_timeout_s: float
    action_timeout_s: float
    navigation_timeout_s: float
    initial_scan_text: str
    recovery_scan_text: str
    max_recovery_attempts: int
    minimum_epoch_map_gain_cells: int
    map_settle_s: float
    navigation_goal_count: int
    navigation_goal_seed: int
    navigation_goal_minimum_separation_m: float
    navigation_goal_clearance_m: float

    def __post_init__(self) -> None:
        if self.scan_startup_timeout_s <= 0.0:
            raise ValueError("scan startup timeout must be positive")
        if (
            self.exploration_timeout_s <= 0.0
            or self.action_timeout_s <= 0.0
            or self.navigation_timeout_s <= 0.0
        ):
            raise ValueError("unknown-world mission timeouts must be positive")
        if self.max_recovery_attempts < 0:
            raise ValueError("max recovery attempts must be non-negative")
        if self.minimum_epoch_map_gain_cells < 0:
            raise ValueError("minimum epoch map gain cells must be non-negative")
        if not math.isfinite(self.map_settle_s) or self.map_settle_s < 0.0:
            raise ValueError("map settle time must be finite and non-negative")
        if self.navigation_goal_count <= 0:
            raise ValueError("navigation goal count must be positive")
        if self.navigation_goal_minimum_separation_m < 0.0:
            raise ValueError("goal separation must be non-negative")
        if self.navigation_goal_clearance_m < 0.0:
            raise ValueError("goal clearance must be non-negative")
        if not self.initial_scan_text.strip() or not self.recovery_scan_text.strip():
            raise ValueError("unknown-world scan actions must be non-empty")


class UnknownWorldMissionRuntimePort(MissionRuntimePort, Protocol):
    def stop_motion_and_wait(
        self,
        request: CommandRequest,
        *,
        timeout_s: float,
    ) -> None: ...

    def wait_for_frontier(
        self,
        request: CommandRequest,
        *,
        recovery_attempts_remaining: int,
        deadline_monotonic: float,
    ) -> str: ...

    def select_mapped_navigation_goals(
        self,
        request: CommandRequest,
        *,
        count: int,
        seed: int,
        minimum_separation_m: float,
        clearance_m: float,
        timeout_s: float,
    ) -> tuple[tuple[float, float], ...]: ...

    def run_navigation_goal(
        self,
        request: CommandRequest,
        *,
        sequence: int,
        goal_xy: tuple[float, float],
        timeout_s: float,
    ) -> None: ...


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


class UnknownWorldMissionExecutor:
    """无场景路线的探索、恢复、存图和动态目标导航事务。"""

    def __init__(
        self,
        runtime: UnknownWorldMissionRuntimePort,
        manager: ExplorerProcessPort,
        evidence: MappingEvidenceTracker,
        spec: UnknownWorldMissionSpec,
    ) -> None:
        self._runtime = runtime
        self._manager = manager
        self._evidence = evidence
        self._spec = spec

    def run(self, request: CommandRequest) -> None:
        try:
            self._run_transaction(request)
        finally:
            self._evidence.finish_mapping_path()

    def _run_transaction(self, request: CommandRequest) -> None:
        self._runtime.transition(
            SessionPhase.AUTOMATIC_MAPPING,
            detail="unknown-world sensor initialization",
        )
        self._runtime.feedback(request, 0.03)
        if not self._manager.dry_run and not wait_for_required_event(
            self._evidence.scan_ready,
            self._spec.scan_startup_timeout_s,
            lambda: request.canceled,
        ):
            raise TimeoutError("mapping scan did not become ready")

        # 原地扫描只依赖实时激光，不含场景坐标；它在探索计时和里程开始前执行，
        # 防止把固定脱困动作包装成自主 frontier 探索证据。
        self._runtime.run_agent_action(
            request,
            text=self._spec.initial_scan_text,
            expected_action="turn",
            timeout_s=self._spec.action_timeout_s,
        )
        self._evidence.reset_exploration()
        self._evidence.begin_mapping_path()
        outcome = self._explore_with_bounded_recovery(request)
        snapshot = self._evidence.snapshot()
        stats = snapshot.map_stats or {}
        self._runtime.transition(
            SessionPhase.MAPPING,
            detail=(
                "unknown-world exploration complete "
                f"reason={outcome} known={stats.get('known_cells', 0)} "
                f"occupied={stats.get('occupied_cells', 0)} "
                f"path={snapshot.mapping_path_m:.2f}m"
            ),
        )
        self._runtime.feedback(request, 0.55)
        self._evidence.finish_mapping_path()

        self._runtime.save_map(request)
        if request.canceled:
            raise AutomaticMissionCancelled("automatic mission canceled")
        self._runtime.start_navigation(request)
        self._runtime.wait_navigation_ready(request)
        goals = self._runtime.select_mapped_navigation_goals(
            request,
            count=self._spec.navigation_goal_count,
            seed=self._spec.navigation_goal_seed,
            minimum_separation_m=(
                self._spec.navigation_goal_minimum_separation_m
            ),
            clearance_m=self._spec.navigation_goal_clearance_m,
            # 候选 admission 与恢复扫描共享通用短 Action 预算；内部所有
            # ComputePath 请求复用一个绝对 deadline，不能按候选重置。
            timeout_s=self._spec.action_timeout_s,
        )
        if len(goals) != self._spec.navigation_goal_count:
            raise RuntimeError(
                "mapped goal sampler returned an unexpected goal count"
            )

        self._runtime.transition(
            SessionPhase.AUTOMATIC_NAVIGATING,
            detail=f"navigating {len(goals)} dynamically sampled goals",
        )
        for index, goal in enumerate(goals, start=1):
            if request.canceled:
                raise AutomaticMissionCancelled("automatic mission canceled")
            self._runtime.run_navigation_goal(
                request,
                sequence=index,
                goal_xy=goal,
                timeout_s=self._spec.navigation_timeout_s,
            )
            self._runtime.feedback(
                request,
                0.65 + 0.3 * index / len(goals),
            )
        self._runtime.transition(
            SessionPhase.MISSION_COMPLETED,
            detail=(
                "unknown-world mapping and navigation completed "
                f"exploration_reason={outcome} sampled_goals={len(goals)}"
            ),
        )
        self._runtime.feedback(request, 1.0)

    def _explore_with_bounded_recovery(
        self, request: CommandRequest
    ) -> str:
        remaining = self._spec.max_recovery_attempts
        # timeout_s 是整轮自主探索预算，不是“每次重启 explorer 都重新赠送一份”。
        # 复用绝对 deadline 可保证内层最坏时间与 acceptance 的预算公式一致。
        exploration_deadline = time.monotonic() + self._spec.exploration_timeout_s
        while True:
            self._runtime.transition(
                SessionPhase.AUTOMATIC_MAPPING,
                detail=(
                    "frontier exploration running "
                    f"recovery_budget={remaining}"
                ),
            )
            self._manager.start_explorer(self._spec.explorer_config_path)
            try:
                outcome = self._runtime.wait_for_frontier(
                    request,
                    recovery_attempts_remaining=remaining,
                    deadline_monotonic=exploration_deadline,
                )
            finally:
                self._manager.stop_explorer()
            if not outcome.startswith("recovery_required:"):
                self._evidence.completion_reason = outcome
                return outcome

            self._evidence.finish_mapping_path()
            self._runtime.transition(
                SessionPhase.AUTOMATIC_MAPPING,
                detail=(
                    "sensor-driven exploration recovery "
                    f"reason={outcome} remaining={remaining}"
                ),
            )
            # 恢复停车是编排层内部控制，不应重新经过 text -> ASR final；否则
            # “停下”会被会话层误判成用户急停，并把自己的自动任务取消。
            self._runtime.stop_motion_and_wait(
                request,
                timeout_s=self._spec.action_timeout_s,
            )
            if request.canceled:
                # 停车等待期间到达的急停必须阻断恢复扫描；虽然 Explorer 已回收，
                # 继续转圈仍会与用户的停止意图冲突，更不能随后开启新 epoch。
                raise AutomaticMissionCancelled("automatic mission canceled")
            recovery_snapshot = self._evidence.snapshot()
            recovery_start_stats = recovery_snapshot.map_stats or {}
            recovery_start_known_cells = int(
                recovery_start_stats.get("known_cells", 0)
            )
            self._runtime.run_agent_action(
                request,
                text=self._spec.recovery_scan_text,
                expected_action="turn",
                timeout_s=self._spec.action_timeout_s,
            )

            # Explorer 的绝对 deadline 只约束自主搜索；恢复转圈由 action_timeout
            # 独立限时。所有 recovery 都等待 SLAM 尾帧稳定，避免 blacklisted 分支
            # 未经信息增益证明就重建进程、清空 FrontierAttemptMemory。
            # quiet window 是“最后一次增长后还要静默多久”的软完成条件；
            # action_timeout 是确认阶段的硬预算，防止地图持续抖动时无限续期。
            confirmation_settle_deadline = time.monotonic() + max(
                self._spec.map_settle_s,
                self._spec.action_timeout_s,
            )
            settled = self._wait_for_map_quiet(
                request,
                deadline_monotonic=confirmation_settle_deadline,
            )
            settled_stats = settled.map_stats or {}
            recovery_end_known_cells = int(
                settled_stats.get("known_cells", 0)
            )
            map_gain_cells = max(
                0,
                recovery_end_known_cells - recovery_start_known_cells,
            )
            telemetry = recovery_snapshot.frontier_telemetry
            decision = decide_epoch_recovery(
                recovery_reason=outcome,
                map_gain_cells=map_gain_cells,
                minimum_gain_cells=(
                    self._spec.minimum_epoch_map_gain_cells
                ),
                recovery_attempts_remaining=remaining,
                available_frontier_count=(
                    telemetry.available_frontier_count
                ),
                active_goal_count=telemetry.active_goal_count,
                blacklisted_frontier_count=(
                    telemetry.blacklisted_frontier_count
                ),
            )
            decision_detail = (
                "frontier epoch recovery evaluated "
                f"reason={outcome} "
                f"known_before={recovery_start_known_cells} "
                f"known_after={recovery_end_known_cells} "
                f"gain={map_gain_cells} "
                f"threshold={self._spec.minimum_epoch_map_gain_cells} "
                f"decision={decision.value}"
            )
            self._runtime.transition(
                SessionPhase.AUTOMATIC_MAPPING,
                detail=decision_detail,
            )

            if decision is EpochRecoveryDecision.COMPLETE_NO_GAIN:
                terminal_reason = "frontier_attempts_exhausted_no_map_gain"
                self._evidence.completion_reason = terminal_reason
                return terminal_reason
            if decision is EpochRecoveryDecision.FAIL_NO_GAIN:
                raise RuntimeError(
                    "frontier recovery produced insufficient map gain: "
                    + decision_detail
                    + " "
                    + (
                        "available="
                        f"{telemetry.available_frontier_count} "
                        f"active={telemetry.active_goal_count} "
                        "blacklisted="
                        f"{telemetry.blacklisted_frontier_count}"
                    )
                )
            if decision is EpochRecoveryDecision.FAIL_BUDGET_EXHAUSTED:
                raise RuntimeError(
                    "frontier recovery gained map but recovery budget is "
                    f"exhausted: {decision_detail}"
                )

            # 预算只在 ADVANCE_EPOCH 时消费；低增益扫描不允许通过重建 Explorer
            # 让成功/失败 approach 复活，也不会重置 typed Action 累计账本。
            remaining -= 1
            self._evidence.reset_exploration(preserve_action_totals=True)
            self._evidence.resume_mapping_path()

    def _wait_for_map_quiet(
        self,
        request: CommandRequest,
        *,
        deadline_monotonic: float,
    ):
        """在不可延长的硬预算内，等待最后增长后的完整安静窗口。"""

        with self._evidence.condition:
            while True:
                if request.canceled:
                    raise AutomaticMissionCancelled("automatic mission canceled")
                snapshot = self._evidence.snapshot()
                now = time.monotonic()
                quiet_deadline = (
                    snapshot.last_map_growth_at
                    + self._spec.map_settle_s
                )
                # 新 SLAM 尾帧会后移 soft quiet deadline，保证确实等到稳定；但
                # 绝不能改写 caller 给出的 hard deadline，否则持续增长会让任务
                # 永远停在确认阶段，破坏整个任务的有界执行保证。
                quiet_remaining_s = quiet_deadline - now
                if quiet_remaining_s <= 0.0:
                    return snapshot
                budget_remaining_s = deadline_monotonic - now
                if budget_remaining_s <= 0.0:
                    raise TimeoutError(
                        "map did not settle before confirmation deadline"
                    )
                # Condition 会被 map callback 主动唤醒；短轮询只用于及时响应取消，
                # 而不是用 sleep 猜测 SLAM 更新何时结束。
                self._evidence.condition.wait(
                    timeout=min(
                        0.1,
                        quiet_remaining_s,
                        budget_remaining_s,
                    )
                )
