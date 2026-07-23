"""自动建图导航任务事务；不依赖 rclpy 或 ROS 消息类型。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
import math
from pathlib import Path
import threading
import time
from typing import Protocol

from .exploration_saturation import (
    SaturationAssessment,
    SaturationEvidenceTracker,
    SaturationPolicy,
    SaturationRuntimeEvidence,
    SaturationTrigger,
    assess_bounded_frontier_saturation,
)
from .mapping_evidence import FrontierTelemetry, MappingEvidenceTracker
from .mapping_return import (
    PlanarPose,
    ReturnToStartEvidence,
    ReturnToStartSpec,
)
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
    RUN_STALL_CONFIRMATION_EPOCH = "run_stall_confirmation_epoch"
    ASSESS_STALL_SATURATION = "assess_stall_saturation"
    RUN_FINAL_CONFIRMATION_EPOCH = "run_final_confirmation_epoch"
    COMPLETE_BELOW_MATERIAL_GAIN = "complete_below_material_gain"
    FAIL_BELOW_MATERIAL_GAIN = "fail_below_material_gain"
    FAIL_BUDGET_EXHAUSTED = "fail_budget_exhausted"


_RELOCATION_RECOVERY_REASONS = frozenset(
    {
        "recovery_required:no_clearance_safe_frontier_approach",
        "recovery_required:frontier_attempts_exhausted",
    }
)


def decide_epoch_recovery(
    *,
    recovery_reason: str,
    map_gain_cells: int,
    minimum_gain_cells: int,
    recovery_attempts_remaining: int,
    available_frontier_count: int,
    active_goal_count: int,
    blacklisted_frontier_count: int,
    detected_frontier_count: int,
    map_gain_ratio: float,
    minimum_gain_ratio: float,
    completed_recovery_epochs: int = 0,
    final_confirmation_used: bool = False,
    recovery_displacement_m: float | None = None,
    minimum_recovery_displacement_m: float | None = None,
) -> EpochRecoveryDecision:
    """只用实时地图与 typed telemetry 判断是否允许开启新 epoch。"""

    counters = (
        map_gain_cells,
        minimum_gain_cells,
        recovery_attempts_remaining,
        available_frontier_count,
        active_goal_count,
        blacklisted_frontier_count,
        detected_frontier_count,
        completed_recovery_epochs,
    )
    if any(value < 0 for value in counters):
        raise ValueError("epoch recovery counters must be non-negative")
    if blacklisted_frontier_count > detected_frontier_count:
        raise ValueError("blacklisted frontier count exceeds detected count")

    if (
        not math.isfinite(map_gain_ratio)
        or map_gain_ratio < 0.0
        or not math.isfinite(minimum_gain_ratio)
        or not 0.0 < minimum_gain_ratio <= 1.0
    ):
        raise ValueError("map gain ratio values are invalid")

    if (recovery_displacement_m is None) != (
        minimum_recovery_displacement_m is None
    ):
        raise ValueError(
            "recovery displacement and its minimum must be provided together"
        )
    relocation_verified = False
    if recovery_displacement_m is not None:
        assert minimum_recovery_displacement_m is not None
        if (
            not math.isfinite(recovery_displacement_m)
            or recovery_displacement_m < 0.0
            or not math.isfinite(minimum_recovery_displacement_m)
            or minimum_recovery_displacement_m <= 0.0
        ):
            raise ValueError("recovery displacement values are invalid")
        relocation_verified = (
            recovery_reason in _RELOCATION_RECOVERY_REASONS
            and recovery_displacement_m >= minimum_recovery_displacement_m
        )

    # 大地图边缘的少量栅格抖动可能刚好越过绝对 cell 门槛；只有绝对值和
    # 相对比例同时显著，才把它解释为仍有新区域，而非传感器细化。
    map_growth_is_significant = (
        map_gain_cells >= minimum_gain_cells
        and map_gain_ratio >= minimum_gain_ratio
    )
    counters_are_terminal = (
        available_frontier_count == 0
        and active_goal_count == 0
        and blacklisted_frontier_count <= detected_frontier_count
    )
    if map_growth_is_significant:
        if recovery_attempts_remaining > 0:
            return EpochRecoveryDecision.ADVANCE_EPOCH
        if (
            recovery_reason
            == "recovery_required:frontier_attempts_exhausted"
            and counters_are_terminal
            and completed_recovery_epochs > 0
            and not final_confirmation_used
        ):
            # 最后一次恢复扫描发生在 Explorer 停止之后。若它仍显著扩图，
            # 既不能直接失败，也不能未经 provider 消费新地图就宣布完成；只
            # 授权一次不含 BackUp/扫描的确认 epoch，并继续复用原绝对 deadline。
            return EpochRecoveryDecision.RUN_FINAL_CONFIRMATION_EPOCH
        return EpochRecoveryDecision.FAIL_BUDGET_EXHAUSTED
    if relocation_verified:
        return (
            EpochRecoveryDecision.ADVANCE_EPOCH
            if recovery_attempts_remaining > 0
            else EpochRecoveryDecision.FAIL_BUDGET_EXHAUSTED
        )

    if (
        recovery_reason
        == "recovery_required:reachable_frontiers_stalled"
        and counters_are_terminal
    ):
        if recovery_attempts_remaining > 0:
            # 一次 plateau 可能只是局部最小值。低增益时不直接完成，也不立即
            # 失败，而是消费有界预算，让 fresh Explorer 形成独立 epoch 证据。
            return EpochRecoveryDecision.RUN_STALL_CONFIRMATION_EPOCH
        if completed_recovery_epochs > 0:
            # 是否真的连续低收益、里程/账本/停车是否充分，统一交给 saturation
            # 深模块评估；这里仅判断已到达允许评估的状态机边界。
            return EpochRecoveryDecision.ASSESS_STALL_SATURATION

    if (
        recovery_reason
        == "recovery_required:frontier_attempts_exhausted"
        and counters_are_terminal
        and recovery_attempts_remaining == 0
        and completed_recovery_epochs > 0
    ):
        # 低增益完成必须有“至少一个已验证恢复 epoch”的历史证据。
        # 否则 max_recovery_attempts=0 会把首轮尝试耗尽误当成建图完成。
        return EpochRecoveryDecision.COMPLETE_BELOW_MATERIAL_GAIN
    return EpochRecoveryDecision.FAIL_BELOW_MATERIAL_GAIN


@dataclass(frozen=True, slots=True)
class RecoveryBackUpSpec:
    """局部脱困只声明动作幅值，不包含地图坐标或场景先验。"""

    distance_m: float
    speed_mps: float
    timeout_s: float
    minimum_displacement_m: float

    def __post_init__(self) -> None:
        values = (
            self.distance_m,
            self.speed_mps,
            self.timeout_s,
            self.minimum_displacement_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("recovery backup values must be finite")
        if not 0.0 < self.distance_m <= 0.50:
            raise ValueError("recovery backup distance must be in (0, 0.50]")
        if not 0.0 < self.speed_mps <= 0.20:
            raise ValueError("recovery backup speed must be in (0, 0.20]")
        if not 0.0 < self.minimum_displacement_m <= self.distance_m:
            raise ValueError(
                "minimum recovery displacement must be positive and no "
                "greater than backup distance"
            )
        minimum_timeout_s = self.distance_m / self.speed_mps + 1.0
        if self.timeout_s < minimum_timeout_s:
            raise ValueError(
                "recovery backup timeout must include travel time and margin"
            )


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
    recovery_backup: RecoveryBackUpSpec
    max_recovery_attempts: int
    minimum_epoch_map_gain_cells: int
    minimum_epoch_map_gain_ratio: float
    map_settle_s: float
    navigation_goal_count: int
    navigation_goal_seed: int
    navigation_goal_minimum_separation_m: float
    navigation_goal_clearance_m: float
    saturation_policy: SaturationPolicy = field(default_factory=SaturationPolicy)
    return_to_start_spec: ReturnToStartSpec = field(
        default_factory=ReturnToStartSpec
    )
    return_to_start_timeout_s: float = 180.0
    return_map_settle_s: float = 15.0
    final_confirmation_timeout_s: float = 240.0

    def __post_init__(self) -> None:
        if (
            not math.isfinite(self.scan_startup_timeout_s)
            or self.scan_startup_timeout_s <= 0.0
        ):
            raise ValueError("scan startup timeout must be positive")
        bounded_timeouts = (
            self.exploration_timeout_s,
            self.action_timeout_s,
            self.navigation_timeout_s,
            self.return_to_start_timeout_s,
            self.final_confirmation_timeout_s,
        )
        if not all(
            math.isfinite(value) and value > 0.0 for value in bounded_timeouts
        ):
            raise ValueError("unknown-world mission timeouts must be positive")
        if self.max_recovery_attempts < 0:
            raise ValueError("max recovery attempts must be non-negative")
        if self.minimum_epoch_map_gain_cells < 0:
            raise ValueError("minimum epoch map gain cells must be non-negative")
        if (
            not math.isfinite(self.minimum_epoch_map_gain_ratio)
            or not 0.0 < self.minimum_epoch_map_gain_ratio <= 1.0
        ):
            raise ValueError("minimum epoch map gain ratio must be in (0, 1]")
        if not math.isfinite(self.map_settle_s) or self.map_settle_s < 0.0:
            raise ValueError("map settle time must be finite and non-negative")
        if (
            not math.isfinite(self.return_map_settle_s)
            or self.return_map_settle_s < 0.0
        ):
            raise ValueError(
                "return map settle time must be finite and non-negative"
            )
        if self.navigation_goal_count <= 0:
            raise ValueError("navigation goal count must be positive")
        if self.navigation_goal_minimum_separation_m < 0.0:
            raise ValueError("goal separation must be non-negative")
        if self.navigation_goal_clearance_m < 0.0:
            raise ValueError("goal clearance must be non-negative")
        if not self.initial_scan_text.strip() or not self.recovery_scan_text.strip():
            raise ValueError("unknown-world scan actions must be non-empty")


class UnknownWorldMissionRuntimePort(MissionRuntimePort, Protocol):
    def capture_mapping_start_pose(
        self,
        request: CommandRequest,
        *,
        timeout_s: float,
    ) -> PlanarPose: ...

    def quiesce_frontier(
        self,
        request: CommandRequest,
        *,
        timeout_s: float,
    ) -> FrontierTelemetry: ...

    def stop_motion_and_wait(
        self,
        request: CommandRequest,
        *,
        timeout_s: float,
    ) -> None: ...

    def run_recovery_backup(
        self,
        request: CommandRequest,
        *,
        distance_m: float,
        speed_mps: float,
        timeout_s: float,
    ) -> float: ...

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

    def run_mapping_return_goal(
        self,
        request: CommandRequest,
        *,
        start_pose: PlanarPose,
        spec: ReturnToStartSpec,
        timeout_s: float,
    ) -> ReturnToStartEvidence: ...

    def record_mapping_completion(
        self,
        *,
        completion_reason: str,
        saturation_evidence: SaturationRuntimeEvidence | None,
        saturation_assessment: SaturationAssessment | None,
        return_to_start: ReturnToStartEvidence,
    ) -> None: ...


@dataclass(frozen=True, slots=True)
class ExplorationRunOutcome:
    """探索阶段的运行时结论；地图完整度仍由离线 evaluator 独立判定。"""

    completion_reason: str
    saturation_evidence: SaturationRuntimeEvidence | None = None
    saturation_assessment: SaturationAssessment | None = None


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

        # 返航起点必须在第一次旋转/探索运动前从实时 TF 捕获；配置中绝不允许
        # 写死场景坐标，否则 unknown-world 演示会退化为已知路线回放。
        mapping_start_pose = self._runtime.capture_mapping_start_pose(
            request,
            timeout_s=self._spec.action_timeout_s,
        )

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
        exploration = self._explore_with_bounded_recovery(request)
        outcome = exploration.completion_reason
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

        self._runtime.transition(
            SessionPhase.AUTOMATIC_MAPPING,
            detail="returning to dynamically captured mapping start pose",
        )
        return_evidence = self._runtime.run_mapping_return_goal(
            request,
            start_pose=mapping_start_pose,
            spec=self._spec.return_to_start_spec,
            timeout_s=self._spec.return_to_start_timeout_s,
        )
        # 返航会再次观察起点附近并可能触发回环；必须先等 SLAM 尾帧稳定再
        # 保存地图，避免 map_saver 把返航前后的半完成优化状态写入磁盘。
        self._wait_for_map_quiet(
            request,
            deadline_monotonic=(
                time.monotonic()
                + max(
                    self._spec.return_map_settle_s,
                    self._spec.action_timeout_s,
                )
            ),
            required_quiet_s=self._spec.return_map_settle_s,
        )
        self._runtime.record_mapping_completion(
            completion_reason=outcome,
            saturation_evidence=exploration.saturation_evidence,
            saturation_assessment=exploration.saturation_assessment,
            return_to_start=return_evidence,
        )

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

    def _cleanup_failed_frontier_epoch(
        self,
        primary_error: BaseException,
        *,
        stop_explorer: bool,
    ) -> None:
        """尽力回收探索器并同步停车，但永远不覆盖触发清理的主异常。"""

        def add_cleanup_note(stage: str, cleanup_error: BaseException) -> None:
            add_note = getattr(primary_error, "add_note", None)
            if callable(add_note):
                add_note(f"{stage}: {cleanup_error}")

        if stop_explorer:
            try:
                self._manager.stop_explorer()
            except BaseException as cleanup_error:
                add_cleanup_note("explorer cleanup failed", cleanup_error)

        # 该内部请求故意不继承用户 request 的 canceled 位。停车仍经过 typed
        # Action/ActionGuard，因此既保留安全审计，又不会被已取消事务短路。
        stop_request = CommandRequest(
            command=SessionCommand.STOP_SESSION,
            source="frontier_failure_cleanup",
        )
        try:
            self._runtime.stop_motion_and_wait(
                stop_request,
                timeout_s=self._spec.action_timeout_s,
            )
        except BaseException as cleanup_error:
            add_cleanup_note("typed stop failed", cleanup_error)

    def _explore_with_bounded_recovery(
        self, request: CommandRequest
    ) -> ExplorationRunOutcome:
        remaining = self._spec.max_recovery_attempts
        saturation_tracker = SaturationEvidenceTracker()
        epoch_id = 0
        final_confirmation_active = False
        final_confirmation_deadline: float | None = None
        # timeout_s 是整轮自主探索预算，不是“每次重启 explorer 都重新赠送一份”。
        # 复用绝对 deadline 可保证内层最坏时间与 acceptance 的预算公式一致。
        exploration_deadline = time.monotonic() + self._spec.exploration_timeout_s
        while True:
            self._runtime.transition(
                SessionPhase.AUTOMATIC_MAPPING,
                detail=(
                    (
                        "frontier final confirmation running "
                        if final_confirmation_active
                        else "frontier exploration running "
                    )
                    + f"recovery_budget={remaining}"
                ),
            )
            self._observe_saturation_tracker(saturation_tracker)
            epoch_id += 1
            saturation_tracker.begin_epoch(epoch_id)
            self._manager.start_explorer(self._spec.explorer_config_path)
            try:
                outcome = self._runtime.wait_for_frontier(
                    request,
                    recovery_attempts_remaining=remaining,
                    deadline_monotonic=(
                        final_confirmation_deadline
                        if final_confirmation_active
                        and final_confirmation_deadline is not None
                        else exploration_deadline
                    ),
                )
            except BaseException as primary_error:
                # Frontier 等待失败时，终止进程只会异步取消 Nav2 goal，并不能
                # 证明 /cmd_vel 已归零。使用独立 request 发送 typed STOP，避免
                # 原用户请求已 canceled 时 Action gateway 提前返回、不真正刹车。
                self._cleanup_failed_frontier_epoch(
                    primary_error,
                    stop_explorer=True,
                )
                raise
            self._observe_saturation_tracker(saturation_tracker)
            saturation_tracker.finish_epoch()
            if outcome == "assessment_required:time_budget":
                # Explorer 仍可能持有一个 NavigateToPose UUID。先通过 ROS 控制面
                # 暂停策略并等账本归零，绝不能直接终止进程后丢失 terminal 回调。
                telemetry = self._runtime.quiesce_frontier(
                    request,
                    timeout_s=self._spec.action_timeout_s,
                )
                try:
                    self._manager.stop_explorer()
                except BaseException as primary_error:
                    self._cleanup_failed_frontier_epoch(
                        primary_error,
                        stop_explorer=False,
                    )
                    raise
                return self._assess_time_budget_saturation(
                    request,
                    tracker=saturation_tracker,
                    telemetry=telemetry,
                    recovery_attempts_remaining=remaining,
                )
            try:
                self._manager.stop_explorer()
            except BaseException as primary_error:
                # 即使 Explorer 生命周期清理自身失败，也要尝试同步停车，同时
                # 保留这个最初异常，方便现场报告给出真实根因。
                self._cleanup_failed_frontier_epoch(
                    primary_error,
                    stop_explorer=False,
                )
                raise
            if final_confirmation_active and outcome.startswith(
                "recovery_required:"
            ):
                self._evidence.finish_mapping_path()
                if outcome != (
                    "recovery_required:frontier_attempts_exhausted"
                ):
                    raise RuntimeError(
                        "final frontier confirmation did not converge: "
                        f"reason={outcome}"
                    )
                # monitor 已验证 active=0 且 accepted Action 全部进入终态。
                # 这里不再 BackUp/转圈，避免制造一张无人消费的新地图。
                self._runtime.stop_motion_and_wait(
                    request,
                    timeout_s=self._spec.action_timeout_s,
                )
                if request.canceled:
                    raise AutomaticMissionCancelled(
                        "automatic mission canceled during final confirmation"
                    )
                telemetry = self._evidence.snapshot().frontier_telemetry
                terminal_reason = (
                    "frontier_attempts_exhausted_after_final_confirmation"
                )
                self._evidence.completion_reason = terminal_reason
                terminal_count = (
                    telemetry.succeeded_goal_count
                    + telemetry.aborted_goal_count
                    + telemetry.canceled_goal_count
                )
                self._runtime.transition(
                    SessionPhase.AUTOMATIC_MAPPING,
                    detail=(
                        "final frontier confirmation completed "
                        f"accepted={telemetry.accepted_goal_count} "
                        f"terminal={terminal_count}"
                    ),
                )
                return ExplorationRunOutcome(terminal_reason)
            if not outcome.startswith("recovery_required:"):
                self._evidence.completion_reason = outcome
                return ExplorationRunOutcome(outcome)

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
            recovery_telemetry = recovery_snapshot.frontier_telemetry
            completed_recovery_epochs = (
                self._spec.max_recovery_attempts - remaining
            )
            stall_probe_active = (
                outcome
                == "recovery_required:reachable_frontiers_stalled"
                and remaining == 0
                and completed_recovery_epochs > 0
                and recovery_telemetry.available_frontier_count == 0
                and recovery_telemetry.active_goal_count == 0
            )
            if stall_probe_active:
                # 最后一次 360° 扫描是独立 final probe，不得混进 Explorer
                # epoch 的收益；这样 evaluator 能证明“重复停滞后仍无新区域”。
                saturation_tracker.begin_final_probe(
                    trigger=SaturationTrigger.REPEATED_REACHABLE_STALL
                )
            recovery_displacement_m: float | None = None
            if outcome in _RELOCATION_RECOVERY_REASONS and remaining > 0:
                backup = self._spec.recovery_backup
                # 原地旋转既不能改变 clearance 连通域，也不能让已耗尽的
                # approach 获得新观察视角。只有这两类 typed 原因且仍有预算时
                # 才执行碰撞检查 BackUp；无预算时禁止再移动后伪造新 epoch。
                recovery_displacement_m = self._runtime.run_recovery_backup(
                    request,
                    distance_m=backup.distance_m,
                    speed_mps=backup.speed_mps,
                    timeout_s=backup.timeout_s,
                )
                if request.canceled:
                    raise AutomaticMissionCancelled(
                        "automatic mission canceled during recovery backup"
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
                required_quiet_s=(
                    self._spec.saturation_policy.required_map_quiet_s
                    if stall_probe_active
                    else None
                ),
            )
            settled_stats = settled.map_stats or {}
            if stall_probe_active:
                self._observe_saturation_tracker(saturation_tracker)
                saturation_tracker.finish_final_probe()
            recovery_end_known_cells = int(
                settled_stats.get("known_cells", 0)
            )
            map_gain_cells = max(
                0,
                recovery_end_known_cells - recovery_start_known_cells,
            )
            map_gain_ratio = map_gain_cells / max(
                recovery_start_known_cells,
                1,
            )
            telemetry = recovery_telemetry
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
                detected_frontier_count=(
                    telemetry.detected_frontier_count
                ),
                completed_recovery_epochs=completed_recovery_epochs,
                final_confirmation_used=final_confirmation_active,
                map_gain_ratio=map_gain_ratio,
                minimum_gain_ratio=(
                    self._spec.minimum_epoch_map_gain_ratio
                ),
                recovery_displacement_m=recovery_displacement_m,
                minimum_recovery_displacement_m=(
                    self._spec.recovery_backup.minimum_displacement_m
                    if recovery_displacement_m is not None
                    else None
                ),
            )
            displacement_detail = (
                "none"
                if recovery_displacement_m is None
                else f"{recovery_displacement_m:.3f}"
            )
            decision_detail = (
                "frontier epoch recovery evaluated "
                f"reason={outcome} "
                f"known_before={recovery_start_known_cells} "
                f"known_after={recovery_end_known_cells} "
                f"gain={map_gain_cells} "
                f"threshold={self._spec.minimum_epoch_map_gain_cells} "
                f"gain_ratio={map_gain_ratio:.6f} "
                "ratio_threshold="
                f"{self._spec.minimum_epoch_map_gain_ratio:.6f} "
                f"detected={telemetry.detected_frontier_count} "
                f"available={telemetry.available_frontier_count} "
                f"active={telemetry.active_goal_count} "
                f"blacklisted={telemetry.blacklisted_frontier_count} "
                f"displacement={displacement_detail} "
                "completed_recoveries="
                f"{self._spec.max_recovery_attempts - remaining} "
                f"decision={decision.value}"
            )
            self._runtime.transition(
                SessionPhase.AUTOMATIC_MAPPING,
                detail=decision_detail,
            )

            if decision is EpochRecoveryDecision.COMPLETE_BELOW_MATERIAL_GAIN:
                terminal_reason = (
                    "frontier_attempts_exhausted_below_material_gain"
                )
                self._evidence.completion_reason = terminal_reason
                return ExplorationRunOutcome(terminal_reason)
            if decision is EpochRecoveryDecision.ASSESS_STALL_SATURATION:
                if not stall_probe_active:
                    raise RuntimeError(
                        "stall saturation assessment has no final probe"
                    )
                return self._finalize_saturation_assessment(
                    request,
                    tracker=saturation_tracker,
                    telemetry=telemetry,
                    recovery_attempts_remaining=remaining,
                    trigger=SaturationTrigger.REPEATED_REACHABLE_STALL,
                    settled=settled,
                    terminal_reason=(
                        "reachable_frontiers_stalled_bounded_saturation"
                    ),
                )
            if decision is EpochRecoveryDecision.RUN_FINAL_CONFIRMATION_EPOCH:
                final_confirmation_active = True
                # final confirmation 是独立且仅一次的 phase：普通 epoch 仍共享
                # 原 900s deadline，确认轮只获得声明的 240s，不按循环重置。
                final_confirmation_deadline = (
                    time.monotonic()
                    + self._spec.final_confirmation_timeout_s
                )
                # 保留跨 epoch Action 总账并继续累计建图里程；确认轮不消费
                # recovery budget，也不允许再递归创建第二个确认轮。
                self._evidence.reset_exploration(
                    preserve_action_totals=True
                )
                self._evidence.resume_mapping_path()
                continue
            if decision is EpochRecoveryDecision.FAIL_BELOW_MATERIAL_GAIN:
                raise RuntimeError(
                    "frontier recovery stayed below material map gain: "
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
                    "frontier recovery found material map growth but "
                    "recovery budget is "
                    f"exhausted: {decision_detail}"
                )

            if decision not in {
                EpochRecoveryDecision.ADVANCE_EPOCH,
                EpochRecoveryDecision.RUN_STALL_CONFIRMATION_EPOCH,
            }:
                raise RuntimeError(
                    f"unsupported epoch recovery decision: {decision.value}"
                )
            # 普通 ADVANCE 由显著扩图/真实位移授权；stall confirmation 则只为
            # 收集固定数量的独立低收益 epoch。二者都消费同一个有界预算，
            # 并复用原绝对 deadline，绝不能通过重启无限延长探索。
            remaining -= 1
            self._evidence.reset_exploration(preserve_action_totals=True)
            self._evidence.resume_mapping_path()

    def _observe_saturation_tracker(
        self,
        tracker: SaturationEvidenceTracker,
    ) -> None:
        snapshot = self._evidence.snapshot()
        stats = snapshot.map_stats or {}
        telemetry = snapshot.frontier_telemetry
        terminal_goal_count = (
            telemetry.succeeded_goal_count
            + telemetry.aborted_goal_count
            + telemetry.canceled_goal_count
        )
        tracker.observe(
            current_known_cells=int(stats.get("known_cells", 0)),
            terminal_goal_count=terminal_goal_count,
            mapping_path_m=snapshot.mapping_path_m,
        )

    def _assess_time_budget_saturation(
        self,
        request: CommandRequest,
        *,
        tracker: SaturationEvidenceTracker,
        telemetry: FrontierTelemetry,
        recovery_attempts_remaining: int,
    ) -> ExplorationRunOutcome:
        """在已排空 Explorer 后做一次最终扫描，再决定是否停止追逐角落。"""

        # 第一次 STOP 建立最终扫描的互斥边界；扫描结束后还会再发一次新 STOP，
        # 因而最终证据不会复用探索阶段的陈旧零速度。
        self._runtime.stop_motion_and_wait(
            request,
            timeout_s=self._spec.action_timeout_s,
        )
        self._evidence.finish_mapping_path()
        tracker.begin_final_probe(trigger=SaturationTrigger.TIME_BUDGET)
        self._runtime.run_agent_action(
            request,
            text=self._spec.recovery_scan_text,
            expected_action="turn",
            timeout_s=self._spec.action_timeout_s,
        )
        settled = self._wait_for_map_quiet(
            request,
            deadline_monotonic=(
                time.monotonic()
                + max(self._spec.map_settle_s, self._spec.action_timeout_s)
            ),
            required_quiet_s=self._spec.saturation_policy.required_map_quiet_s,
        )
        self._observe_saturation_tracker(tracker)
        tracker.finish_final_probe()
        return self._finalize_saturation_assessment(
            request,
            tracker=tracker,
            telemetry=telemetry,
            recovery_attempts_remaining=recovery_attempts_remaining,
            trigger=SaturationTrigger.TIME_BUDGET,
            settled=settled,
            terminal_reason="time_budget_exhausted",
        )

    def _finalize_saturation_assessment(
        self,
        request: CommandRequest,
        *,
        tracker: SaturationEvidenceTracker,
        telemetry: FrontierTelemetry,
        recovery_attempts_remaining: int,
        trigger: SaturationTrigger,
        settled,
        terminal_reason: str,
    ) -> ExplorationRunOutcome:
        """在 final probe 后统一完成停车、证据评估与任务层原因落盘。"""

        terminal_goal_count = (
            telemetry.succeeded_goal_count
            + telemetry.aborted_goal_count
            + telemetry.canceled_goal_count
        )
        pending_goal_count = max(
            0,
            telemetry.accepted_goal_count - terminal_goal_count,
        )
        # 两种饱和触发都必须在 final probe 之后再产生一帧 typed STOP；
        # 不能把探索前或扫描前的旧零速度冒充最终停车证据。
        self._runtime.stop_motion_and_wait(
            request,
            timeout_s=self._spec.action_timeout_s,
        )
        map_quiet_s = max(
            0.0,
            time.monotonic() - settled.last_map_growth_at,
        )
        runtime_evidence = SaturationRuntimeEvidence(
            history=tracker.snapshot(),
            trigger=trigger,
            recovery_attempts_remaining=recovery_attempts_remaining,
            residual_available_frontiers=(
                telemetry.available_frontier_count
            ),
            active_goal_count=telemetry.active_goal_count,
            pending_goal_count=pending_goal_count,
            map_quiet_s=map_quiet_s,
            typed_stop_confirmed=True,
            typed_stop_after_final_probe=True,
            # stop_motion_and_wait 在 ROS Adapter 中会等待一帧更新后的零速度；
            # 纯任务层紧接着评估，因此这里的 age 为同一事务内的 0 秒。
            final_stop_age_s=0.0,
        )
        assessment = assess_bounded_frontier_saturation(
            runtime_evidence,
            self._spec.saturation_policy,
        )
        if not assessment.complete:
            raise RuntimeError(
                "bounded saturation evidence is insufficient "
                f"trigger={trigger.value}: "
                + ",".join(assessment.unmet_requirements)
            )
        self._evidence.completion_reason = terminal_reason
        self._runtime.transition(
            SessionPhase.AUTOMATIC_MAPPING,
            detail=(
                "bounded frontier saturation accepted "
                f"trigger={trigger.value} "
                f"residual={telemetry.available_frontier_count} "
                f"epochs={len(runtime_evidence.history.epochs)}"
            ),
        )
        return ExplorationRunOutcome(
            completion_reason=terminal_reason,
            saturation_evidence=runtime_evidence,
            saturation_assessment=assessment,
        )

    def _wait_for_map_quiet(
        self,
        request: CommandRequest,
        *,
        deadline_monotonic: float,
        required_quiet_s: float | None = None,
    ):
        """在不可延长的硬预算内，等待最后增长后的完整安静窗口。"""

        with self._evidence.condition:
            while True:
                if request.canceled:
                    raise AutomaticMissionCancelled("automatic mission canceled")
                snapshot = self._evidence.snapshot()
                now = time.monotonic()
                quiet_deadline = snapshot.last_map_growth_at + (
                    self._spec.map_settle_s
                    if required_quiet_s is None
                    else required_quiet_s
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
