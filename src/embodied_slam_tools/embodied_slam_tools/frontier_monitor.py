"""frontier 探索完成策略；不依赖 ROS 消息和 Node。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Callable, Protocol

from .mapping_evidence import MappingEvidenceSnapshot, MappingEvidenceTracker
from .mission_executor import AutomaticMissionCancelled, CommandRequest
from .showcase_session import exploration_completion_reason


class ExplorationDecision(str, Enum):
    """Unknown-world 探索器在一次观测后应采取的领域动作。"""

    CONTINUE = "continue"
    RECOVER = "recover"
    COMPLETE = "complete"
    FAIL = "fail"


@dataclass(frozen=True, slots=True)
class ExplorationObservation:
    """与 ROS 消息解耦的 frontier 生命周期快照。"""

    native_completion_reported: bool
    reachable_frontier_count: int
    active_goal_count: int
    blacklisted_frontier_count: int
    plateau_detected: bool
    time_budget_reached: bool
    recovery_attempts_remaining: int
    map_quiet_s: float
    required_quiet_s: float
    frontier_idle_s: float = 0.0
    required_frontier_idle_s: float = 0.0


def decide_exploration(
    observation: ExplorationObservation,
) -> ExplorationDecision:
    """依据 frontier 生命周期决定继续、恢复、完成或失败。"""

    counters = (
        observation.reachable_frontier_count,
        observation.active_goal_count,
        observation.blacklisted_frontier_count,
        observation.recovery_attempts_remaining,
    )
    if any(value < 0 for value in counters):
        raise ValueError("exploration counters must be non-negative")
    quiet_windows = (
        observation.map_quiet_s,
        observation.required_quiet_s,
        observation.frontier_idle_s,
        observation.required_frontier_idle_s,
    )
    if any(not math.isfinite(value) or value < 0.0 for value in quiet_windows):
        raise ValueError("exploration quiet windows must be non-negative")

    if observation.active_goal_count > 0:
        # 原生 complete 事件和 Nav2 goal terminal 回调可能乱序；活动目标归零前
        # 宣布完成会让事务层关闭 explorer，并把本应继续的目标取消掉。
        return (
            ExplorationDecision.FAIL
            if observation.time_budget_reached
            else ExplorationDecision.CONTINUE
        )

    if observation.reachable_frontier_count > 0:
        if observation.time_budget_reached:
            return ExplorationDecision.FAIL
        if (
            observation.plateau_detected
            and observation.frontier_idle_s
            >= observation.required_frontier_idle_s
        ):
            # 地图平台期只能证明“当前没有增长”，不能证明环境已经探索完成；
            # 仍有可达边界时先恢复，恢复预算耗尽则显式失败。
            return (
                ExplorationDecision.RECOVER
                if observation.recovery_attempts_remaining > 0
                else ExplorationDecision.FAIL
            )
        return ExplorationDecision.CONTINUE

    if observation.blacklisted_frontier_count > 0:
        if (
            observation.frontier_idle_s
            < observation.required_frontier_idle_s
        ):
            # Nav2 goal 进入终态后，Explore Lite 需要到下一次 makePlan 才会
            # 重新发布 available frontier。交接窗内的 active=available=0 是
            # 瞬态，不能立刻重启 explorer、清空本 epoch 的尝试记忆。
            return ExplorationDecision.CONTINUE
        return (
            ExplorationDecision.RECOVER
            if observation.recovery_attempts_remaining > 0
            else ExplorationDecision.FAIL
        )

    if observation.native_completion_reported:
        if observation.map_quiet_s >= observation.required_quiet_s:
            return ExplorationDecision.COMPLETE
        return ExplorationDecision.CONTINUE

    if observation.time_budget_reached:
        return ExplorationDecision.FAIL
    if observation.plateau_detected:
        return (
            ExplorationDecision.RECOVER
            if observation.recovery_attempts_remaining > 0
            else ExplorationDecision.FAIL
        )
    return ExplorationDecision.CONTINUE


@dataclass(frozen=True, slots=True)
class FrontierMonitorConfig:
    timeout_s: float
    min_runtime_s: float
    stable_map_s: float
    completion_status: str
    min_known_cells: int
    min_occupied_cells: int
    min_mapping_path_m: float
    frontier_idle_grace_s: float = 0.0
    policy_mode: str = "known_world"
    status_available: bool = True
    dry_run: bool = False
    dry_run_delay_s: float = 0.0

    def __post_init__(self) -> None:
        # idle grace 是 Action terminal 与下一目标交接的容错窗，而非关闭严格门禁；
        # NaN/Inf 会让所有比较静默失效，因此必须在配置边界立即拒绝。
        if (
            not math.isfinite(self.frontier_idle_grace_s)
            or self.frontier_idle_grace_s < 0.0
        ):
            raise ValueError(
                "frontier_idle_grace_s must be finite and non-negative"
            )


class ExplorerHealthPort(Protocol):
    def explorer_exited_unexpectedly(self) -> tuple[bool, int | None]: ...


class FrontierExplorationMonitor:
    """融合地图、里程和 explorer 健康状态，给出可审计结束原因。"""

    def __init__(
        self,
        evidence: MappingEvidenceTracker,
        explorer: ExplorerHealthPort,
        config: FrontierMonitorConfig,
        *,
        cancel_motion: Callable[[], None],
        log_info: Callable[[str], None] = lambda _message: None,
        clock: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        self._evidence = evidence
        self._explorer = explorer
        self._config = config
        self._cancel_motion = cancel_motion
        self._log_info = log_info
        self._clock = clock
        self._sleep = sleep

    def wait(
        self,
        request: CommandRequest,
        *,
        recovery_attempts_remaining: int = 0,
        deadline_monotonic: float | None = None,
    ) -> str:
        if self._config.dry_run:
            return self._wait_dry_run(request)
        if not self._config.status_available:
            raise RuntimeError(
                "explore_lite_msgs is unavailable; run "
                "bash scripts/setup_frontier_exploration.sh"
            )

        started_at = self._clock()
        local_deadline = started_at + self._config.timeout_s
        # unknown-world recovery 会重启 Explore Lite，但整轮任务只能消费同一个
        # 绝对预算；取 min 防止子模块自己的 timeout 反过来延长上层事务。
        deadline = (
            local_deadline
            if deadline_monotonic is None
            else min(local_deadline, deadline_monotonic)
        )
        with self._evidence.condition:
            while self._clock() < deadline:
                self._raise_if_canceled(request)
                elapsed = self._clock() - started_at
                snapshot = self._evidence.snapshot()
                self._reject_premature_explorer_completion(snapshot, elapsed)
                reason = self._completion_reason(
                    snapshot,
                    elapsed,
                    recovery_attempts_remaining=recovery_attempts_remaining,
                )
                if reason is not None:
                    if reason.startswith((
                        "recovery_required:",
                        "assessment_required:",
                    )):
                        self._log_info(reason)
                        return reason
                    return self._complete(snapshot, reason)
                exited, code = self._explorer.explorer_exited_unexpectedly()
                if exited:
                    raise RuntimeError(
                        f"frontier explorer exited unexpectedly code={code}"
                    )
                self._evidence.condition.wait(
                    timeout=min(0.5, max(0.0, deadline - self._clock()))
                )

        snapshot = self._evidence.snapshot()
        reason = self._completion_reason(
            snapshot,
            self._clock() - started_at,
            time_budget_reached=True,
            recovery_attempts_remaining=recovery_attempts_remaining,
        )
        if reason is not None:
            if reason.startswith((
                "recovery_required:",
                "assessment_required:",
            )):
                self._log_info(reason)
                return reason
            return self._complete(snapshot, reason)
        stats = snapshot.map_stats or {}
        raise TimeoutError(
            "frontier exploration did not complete before timeout "
            f"known={stats.get('known_cells', 0)} "
            f"occupied={stats.get('occupied_cells', 0)} "
            f"path={snapshot.mapping_path_m:.2f}m"
        )

    def _wait_dry_run(self, request: CommandRequest) -> str:
        deadline = self._clock() + self._config.dry_run_delay_s
        while self._clock() < deadline:
            self._raise_if_canceled(request)
            self._sleep(0.05)
        return "dry_run"

    def _raise_if_canceled(self, request: CommandRequest) -> None:
        if not request.canceled:
            return
        # 急停必须先旁路普通队列发布 stop，再向事务层传播取消原因。
        self._cancel_motion()
        raise AutomaticMissionCancelled("automatic mission canceled")

    def _reject_premature_explorer_completion(
        self,
        snapshot: MappingEvidenceSnapshot,
        elapsed_s: float,
    ) -> None:
        if (
            snapshot.explore_status != self._config.completion_status
            or elapsed_s < self._config.min_runtime_s
        ):
            return
        stats = snapshot.map_stats or {}
        if stats.get("known_cells", 0) < self._config.min_known_cells:
            raise RuntimeError(
                "frontier exploration ended before known-cell threshold"
            )
        if stats.get("occupied_cells", 0) < self._config.min_occupied_cells:
            raise RuntimeError(
                "frontier exploration ended before occupied-cell threshold"
            )
        if snapshot.mapping_path_m < self._config.min_mapping_path_m:
            raise RuntimeError(
                "frontier exploration ended before mapping-path threshold "
                f"({snapshot.mapping_path_m:.2f}m < "
                f"{self._config.min_mapping_path_m:.2f}m)"
            )

    def _completion_reason(
        self,
        snapshot: MappingEvidenceSnapshot,
        elapsed_s: float,
        *,
        time_budget_reached: bool = False,
        recovery_attempts_remaining: int = 0,
    ) -> str | None:
        if self._config.policy_mode == "unknown_world":
            return self._unknown_world_reason(
                snapshot,
                elapsed_s=elapsed_s,
                time_budget_reached=time_budget_reached,
                recovery_attempts_remaining=recovery_attempts_remaining,
            )
        if self._config.policy_mode != "known_world":
            raise ValueError(
                f"unsupported frontier policy mode: {self._config.policy_mode}"
            )
        stats = snapshot.map_stats or {}
        return exploration_completion_reason(
            status=snapshot.explore_status,
            completion_status=self._config.completion_status,
            elapsed_s=elapsed_s,
            min_runtime_s=self._config.min_runtime_s,
            known_cells=stats.get("known_cells", 0),
            occupied_cells=stats.get("occupied_cells", 0),
            min_known_cells=self._config.min_known_cells,
            min_occupied_cells=self._config.min_occupied_cells,
            mapping_path_m=snapshot.mapping_path_m,
            min_mapping_path_m=self._config.min_mapping_path_m,
            seconds_since_map_growth=(
                self._clock() - snapshot.last_map_growth_at
            ),
            stable_map_s=self._config.stable_map_s,
            time_budget_reached=time_budget_reached,
        )

    def _unknown_world_reason(
        self,
        snapshot: MappingEvidenceSnapshot,
        *,
        elapsed_s: float,
        time_budget_reached: bool,
        recovery_attempts_remaining: int,
    ) -> str | None:
        telemetry = snapshot.frontier_telemetry
        no_clearance = (
            telemetry.status == "exploration_blocked"
            and telemetry.completion_reason
            == "no_clearance_safe_frontier_approach"
        )
        if no_clearance:
            # 这是 provider 完成一次真实 frontier 搜索后给出的类型化阻塞，
            # 不是“运行时间太短”。若仍套用 min_runtime_s，第二个 epoch 会在
            # 已知不可接近的状态下空等 60 秒，随后还会丢失真正的恢复原因。
            if (
                telemetry.detected_frontier_count <= 0
                or telemetry.available_frontier_count != 0
                or telemetry.blacklisted_frontier_count != 0
            ):
                raise RuntimeError(
                    "invalid no-clearance telemetry: "
                    f"detected={telemetry.detected_frontier_count} "
                    f"available={telemetry.available_frontier_count} "
                    f"blacklisted={telemetry.blacklisted_frontier_count}"
                )
            if telemetry.active_goal_count > 0:
                # provider 已请求停止不代表 Nav2 goal 已进入终态；必须等 UUID
                # 清空后，上层才可独占底盘执行 BackUp 恢复。
                return None
            terminal_count = (
                telemetry.succeeded_goal_count
                + telemetry.aborted_goal_count
                + telemetry.canceled_goal_count
            )
            if telemetry.accepted_goal_count != terminal_count:
                raise RuntimeError(
                    "no-clearance recovery refused: Action ledger is not "
                    "drained "
                    f"accepted={telemetry.accepted_goal_count} "
                    f"terminal={terminal_count}"
                )
            return "recovery_required:no_clearance_safe_frontier_approach"
        attempts_exhausted = (
            telemetry.completion_reason
            == "frontier_attempts_exhausted_recoverable"
        )
        if attempts_exhausted:
            # attempts_exhausted 与 no_clearance 一样，是 provider 完成一次
            # 真实搜索后的 typed epoch terminal；放在 min_runtime_s 之前，
            # 避免新 epoch 快速耗尽时空等，或在超时分支丢失真实原因。
            if (
                telemetry.status != "exploration_blocked"
                or telemetry.detected_frontier_count <= 0
                or telemetry.available_frontier_count != 0
                or telemetry.blacklisted_frontier_count
                > telemetry.detected_frontier_count
            ):
                raise RuntimeError(
                    "invalid attempt-exhaustion telemetry: "
                    f"status={telemetry.status} "
                    f"detected={telemetry.detected_frontier_count} "
                    f"available={telemetry.available_frontier_count} "
                    f"blacklisted={telemetry.blacklisted_frontier_count}"
                )
            # blacklisted 表示本 epoch 中已失败/不可取的 approach。provider
            # 明确发布 attempts_exhausted 时，这正是 BackUp 后重建观察视角的
            # 输入，不能误判为遥测非法；最终是否允许完成仍由任务层预算与
            # 地图质量门禁决定。
            if telemetry.active_goal_count > 0:
                return None
            terminal_count = (
                telemetry.succeeded_goal_count
                + telemetry.aborted_goal_count
                + telemetry.canceled_goal_count
            )
            if telemetry.accepted_goal_count != terminal_count:
                raise RuntimeError(
                    "attempt-exhaustion recovery refused: Action ledger is "
                    "not drained "
                    f"accepted={telemetry.accepted_goal_count} "
                    f"terminal={terminal_count}"
                )
            if time_budget_reached:
                # 整轮硬预算已经耗尽时不能继续等待本 epoch 的地图静默窗；
                # 否则刚到达的 SLAM 尾帧会让这里返回 None，monitor 随后抛通用
                # TimeoutError，使上层永远无法执行 final probe 与有界饱和评估。
                # Action 账本已在上方排空，任务层还会再次 typed STOP 后才扫描。
                return "assessment_required:time_budget"
            map_quiet_s = max(
                0.0, self._clock() - snapshot.last_map_growth_at
            )
            if map_quiet_s < self._config.stable_map_s:
                return None
            # 这里只授权上层停车、换视角并重新观测，不表示地图完成。
            return "recovery_required:frontier_attempts_exhausted"
        if elapsed_s < self._config.min_runtime_s:
            return None
        map_quiet_s = max(0.0, self._clock() - snapshot.last_map_growth_at)
        progress_stalled = (
            telemetry.status == "exploration_blocked"
            and telemetry.completion_reason
            == "frontier_progress_stalled_recoverable"
        )
        if progress_stalled:
            if telemetry.active_goal_count > 0:
                # Explore 已请求取消并不等于 Nav2 已 terminal；active UUID
                # 归零前启动恢复扫描，会让两个控制事务同时占有底盘。
                return None
            terminal_count = (
                telemetry.succeeded_goal_count
                + telemetry.aborted_goal_count
                + telemetry.canceled_goal_count
            )
            if telemetry.accepted_goal_count != terminal_count:
                raise RuntimeError(
                    "frontier progress recovery refused: Action ledger is "
                    "not drained "
                    f"accepted={telemetry.accepted_goal_count} "
                    f"terminal={terminal_count}"
                )
            # 连续真实停滞本身就是恢复证据，无需再等待地图 quiet；上层仍会
            # 先 typed STOP、再旋转扫描并用地图增益决定是否开启新 epoch。
            return "recovery_required:frontier_progress_stalled"
        if time_budget_reached:
            # 普通探索态的硬预算只触发“是否边际收益耗尽”的收口评估，不能
            # 在这里自证地图完整。任务层会先暂停 Explorer、排空 Action 账本、
            # typed STOP 并做最终扫描；任何证据不足仍然 fail-close。
            return "assessment_required:time_budget"
        observation = ExplorationObservation(
            native_completion_reported=(
                telemetry.status == self._config.completion_status
                and telemetry.completion_reason == "no_frontiers"
            ),
            reachable_frontier_count=telemetry.available_frontier_count,
            active_goal_count=telemetry.active_goal_count,
            blacklisted_frontier_count=telemetry.blacklisted_frontier_count,
            plateau_detected=map_quiet_s >= self._config.stable_map_s,
            time_budget_reached=time_budget_reached,
            recovery_attempts_remaining=recovery_attempts_remaining,
            map_quiet_s=map_quiet_s,
            required_quiet_s=self._config.stable_map_s,
            frontier_idle_s=max(
                0.0, self._clock() - snapshot.last_frontier_activity_at
            ),
            required_frontier_idle_s=self._config.frontier_idle_grace_s,
        )
        decision = decide_exploration(observation)
        if decision is ExplorationDecision.CONTINUE:
            return None
        if decision is ExplorationDecision.COMPLETE:
            return "no_reachable_frontiers"
        if decision is ExplorationDecision.RECOVER:
            if telemetry.blacklisted_frontier_count > 0:
                return "recovery_required:blacklisted_frontiers"
            return "recovery_required:reachable_frontiers_stalled"

        # all-blacklisted 和平台期都只是可恢复故障；恢复次数耗尽后必须明确失败，
        # 不能复用旧 coverage_plateau 名称伪装成完整地图。
        raise RuntimeError(
            "frontier recovery exhausted: "
            f"available={telemetry.available_frontier_count} "
            f"active={telemetry.active_goal_count} "
            f"blacklisted={telemetry.blacklisted_frontier_count}"
        )

    def _complete(
        self,
        snapshot: MappingEvidenceSnapshot,
        reason: str,
    ) -> str:
        self._evidence.completion_reason = reason
        stats = snapshot.map_stats or {}
        self._log_info(
            f"frontier exploration complete reason={reason}: "
            f"known={stats.get('known_cells', 0)} "
            f"occupied={stats.get('occupied_cells', 0)} "
            f"path={snapshot.mapping_path_m:.2f}m"
        )
        return reason
