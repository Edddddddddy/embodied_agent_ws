"""未知环境探索的有界饱和判定；不依赖 ROS、真值地图或场景尺寸。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math


@dataclass(frozen=True, slots=True)
class ExplorationCounters:
    """一次运行时采样的累计计数器。"""

    peak_known_cells: int
    terminal_goal_count: int
    mapping_path_m: float

    def __post_init__(self) -> None:
        if self.peak_known_cells < 0 or self.terminal_goal_count < 0:
            raise ValueError("exploration counters must be non-negative")
        if not math.isfinite(self.mapping_path_m) or self.mapping_path_m < 0.0:
            raise ValueError("mapping path must be finite and non-negative")


@dataclass(frozen=True, slots=True)
class ExplorationEpochEvidence:
    """一个 frontier epoch 的跨周期收益。"""

    epoch_id: int
    start: ExplorationCounters
    end: ExplorationCounters

    def __post_init__(self) -> None:
        if self.epoch_id <= 0:
            raise ValueError("epoch id must be positive")
        if self.end.peak_known_cells < self.start.peak_known_cells:
            raise ValueError("peak known cells must not decrease across an epoch")
        if self.end.terminal_goal_count < self.start.terminal_goal_count:
            raise ValueError("terminal goal count must not decrease across an epoch")
        if self.end.mapping_path_m < self.start.mapping_path_m:
            raise ValueError("mapping path must not decrease across an epoch")

    @property
    def map_gain_cells(self) -> int:
        return self.end.peak_known_cells - self.start.peak_known_cells

    @property
    def map_gain_ratio(self) -> float:
        return self.map_gain_cells / max(1, self.start.peak_known_cells)

    @property
    def terminal_goals(self) -> int:
        return self.end.terminal_goal_count - self.start.terminal_goal_count

    @property
    def map_gain_cells_per_terminal(self) -> float:
        return self.map_gain_cells / max(1, self.terminal_goals)

    @property
    def map_gain_ratio_per_terminal(self) -> float:
        return self.map_gain_ratio / max(1, self.terminal_goals)

    @property
    def path_gain_m(self) -> float:
        return self.end.mapping_path_m - self.start.mapping_path_m


@dataclass(frozen=True, slots=True)
class FinalProbeEvidence:
    """硬预算触发后，最后一次短探测得到的独立增益证据。"""

    started_after_hard_budget: bool
    start: ExplorationCounters
    end: ExplorationCounters

    def __post_init__(self) -> None:
        if self.end.peak_known_cells < self.start.peak_known_cells:
            raise ValueError("probe peak known cells must not decrease")
        if self.end.terminal_goal_count < self.start.terminal_goal_count:
            raise ValueError("probe terminal goal count must not decrease")
        if self.end.mapping_path_m < self.start.mapping_path_m:
            raise ValueError("probe mapping path must not decrease")

    @property
    def map_gain_cells(self) -> int:
        return self.end.peak_known_cells - self.start.peak_known_cells

    @property
    def map_gain_ratio(self) -> float:
        return self.map_gain_cells / max(1, self.start.peak_known_cells)


@dataclass(frozen=True, slots=True)
class SaturationHistory:
    """跨 explorer 重启保留的证据快照。"""

    counters: ExplorationCounters
    epochs: tuple[ExplorationEpochEvidence, ...]
    final_probe: FinalProbeEvidence | None


class SaturationEvidenceTracker:
    """把易重置的运行时数值压缩成跨 epoch 的单调证据。

    ``current_known_cells`` 可以因回环优化而缩小；内部只保留历史峰值。
    Action 终态总数和里程必须由上游提供任务级累计值，若倒退则立即报错，
    避免把 provider 重启造成的计数清零误当作一个低收益 epoch。
    """

    def __init__(self) -> None:
        self._counters = ExplorationCounters(0, 0, 0.0)
        self._epochs: list[ExplorationEpochEvidence] = []
        self._active_epoch: tuple[int, ExplorationCounters] | None = None
        self._probe_start: tuple[bool, ExplorationCounters] | None = None
        self._final_probe: FinalProbeEvidence | None = None

    def observe(
        self,
        *,
        current_known_cells: int,
        terminal_goal_count: int,
        mapping_path_m: float,
    ) -> None:
        """记录累计采样；回环后的 current 缩小不会冲掉曾发现的地图。"""

        if current_known_cells < 0 or terminal_goal_count < 0:
            raise ValueError("exploration observations must be non-negative")
        if not math.isfinite(mapping_path_m) or mapping_path_m < 0.0:
            raise ValueError("mapping path must be finite and non-negative")
        if terminal_goal_count < self._counters.terminal_goal_count:
            raise ValueError("terminal goal total must be monotonic")
        if mapping_path_m < self._counters.mapping_path_m:
            raise ValueError("mapping path total must be monotonic")
        self._counters = ExplorationCounters(
            peak_known_cells=max(
                self._counters.peak_known_cells, int(current_known_cells)
            ),
            terminal_goal_count=int(terminal_goal_count),
            mapping_path_m=float(mapping_path_m),
        )

    def begin_epoch(self, epoch_id: int) -> None:
        if epoch_id <= 0:
            raise ValueError("epoch id must be positive")
        if self._active_epoch is not None:
            raise RuntimeError("an exploration epoch is already active")
        if self._probe_start is not None:
            raise RuntimeError("cannot start an epoch during the final probe")
        if self._epochs and epoch_id <= self._epochs[-1].epoch_id:
            raise ValueError("epoch ids must increase")
        self._active_epoch = (int(epoch_id), self._counters)

    def finish_epoch(self) -> ExplorationEpochEvidence:
        if self._active_epoch is None:
            raise RuntimeError("no exploration epoch is active")
        epoch_id, start = self._active_epoch
        evidence = ExplorationEpochEvidence(epoch_id, start, self._counters)
        self._epochs.append(evidence)
        self._active_epoch = None
        return evidence

    def begin_final_probe(self, *, hard_budget_reached: bool) -> None:
        if self._active_epoch is not None:
            raise RuntimeError("finish the active epoch before the final probe")
        if self._probe_start is not None or self._final_probe is not None:
            raise RuntimeError("the final probe may run only once")
        self._probe_start = (bool(hard_budget_reached), self._counters)

    def finish_final_probe(self) -> FinalProbeEvidence:
        if self._probe_start is None:
            raise RuntimeError("the final probe has not started")
        after_budget, start = self._probe_start
        self._final_probe = FinalProbeEvidence(
            started_after_hard_budget=after_budget,
            start=start,
            end=self._counters,
        )
        self._probe_start = None
        return self._final_probe

    def snapshot(self) -> SaturationHistory:
        return SaturationHistory(
            counters=self._counters,
            epochs=tuple(self._epochs),
            final_probe=self._final_probe,
        )


@dataclass(frozen=True, slots=True)
class SaturationPolicy:
    """与场景大小无关的保守收敛门槛。"""

    minimum_low_yield_epochs: int = 2
    minimum_terminal_goals_per_epoch: int = 3
    minimum_mapping_path_m: float = 20.0
    maximum_residual_available_frontiers: int = 4
    maximum_low_yield_gain_cells: int = 40
    maximum_low_yield_gain_ratio: float = 0.002
    required_map_quiet_s: float = 10.0
    maximum_final_stop_age_s: float = 5.0

    def __post_init__(self) -> None:
        integer_values = (
            self.minimum_low_yield_epochs,
            self.minimum_terminal_goals_per_epoch,
            self.maximum_residual_available_frontiers,
            self.maximum_low_yield_gain_cells,
        )
        if any(value < 0 for value in integer_values):
            raise ValueError("saturation policy counters must be non-negative")
        if self.minimum_low_yield_epochs < 2:
            raise ValueError("at least two low-yield epochs are required")
        if self.minimum_terminal_goals_per_epoch < 3:
            raise ValueError("at least three terminal goals per epoch are required")
        finite_positive = (
            self.minimum_mapping_path_m,
            self.maximum_low_yield_gain_ratio,
            self.required_map_quiet_s,
            self.maximum_final_stop_age_s,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in finite_positive):
            raise ValueError("saturation policy durations and ratios must be positive")
        if self.maximum_low_yield_gain_cells <= 0:
            raise ValueError("low-yield cell threshold must be positive")
        if self.maximum_low_yield_gain_ratio > 1.0:
            raise ValueError("low-yield ratio threshold must not exceed one")


@dataclass(frozen=True, slots=True)
class SaturationRuntimeEvidence:
    """硬预算边界上做最终决策所需的最小运行时状态。"""

    history: SaturationHistory
    hard_budget_reached: bool
    recovery_attempts_remaining: int
    residual_available_frontiers: int
    active_goal_count: int
    pending_goal_count: int
    map_quiet_s: float
    typed_stop_confirmed: bool
    typed_stop_after_final_probe: bool
    final_stop_age_s: float | None

    def __post_init__(self) -> None:
        counters = (
            self.recovery_attempts_remaining,
            self.residual_available_frontiers,
            self.active_goal_count,
            self.pending_goal_count,
        )
        if any(value < 0 for value in counters):
            raise ValueError("runtime saturation counters must be non-negative")
        if not math.isfinite(self.map_quiet_s) or self.map_quiet_s < 0.0:
            raise ValueError("map quiet duration must be finite and non-negative")
        if self.final_stop_age_s is not None and (
            not math.isfinite(self.final_stop_age_s)
            or self.final_stop_age_s < 0.0
        ):
            raise ValueError("final stop age must be finite and non-negative")


class SaturationDecision(str, Enum):
    CONTINUE = "continue"
    COMPLETE = "complete"


@dataclass(frozen=True, slots=True)
class SaturationAssessment:
    decision: SaturationDecision
    unmet_requirements: tuple[str, ...]

    @property
    def complete(self) -> bool:
        return self.decision is SaturationDecision.COMPLETE


def consecutive_low_yield_epoch_count(
    history: SaturationHistory,
    policy: SaturationPolicy = SaturationPolicy(),
) -> int:
    """统计末尾连续低收益 epoch；一旦遇到高收益轮次立即停止。"""

    count = 0
    for epoch in reversed(history.epochs):
        if epoch.terminal_goals < policy.minimum_terminal_goals_per_epoch:
            break
        material = (
            epoch.map_gain_cells_per_terminal
            >= policy.maximum_low_yield_gain_cells
            and epoch.map_gain_ratio_per_terminal
            >= policy.maximum_low_yield_gain_ratio
        )
        if material:
            break
        count += 1
    return count


def assess_bounded_frontier_saturation(
    evidence: SaturationRuntimeEvidence,
    policy: SaturationPolicy = SaturationPolicy(),
) -> SaturationAssessment:
    """纯函数判定探索是否已进入“继续追角落收益很低”的有界终态。"""

    unmet: list[str] = []
    if not evidence.hard_budget_reached:
        unmet.append("hard_budget_not_reached")
    # recovery_attempts_remaining 是诊断字段，不是完成门槛。现场可能在单个
    # Explorer epoch 中耗尽整轮时间预算，而根本没有触发需要 BackUp/重启的
    # provider reason；若强制 remaining==0，机器人会在已经连续低收益且完成
    # 最终探测后仍被判失败。硬时间预算 + 下方独立收益/账本/停车证据才是边界。

    recent_epochs = evidence.history.epochs[-policy.minimum_low_yield_epochs :]
    if len(recent_epochs) < policy.minimum_low_yield_epochs:
        unmet.append("insufficient_low_yield_epochs")
    else:
        if any(
            epoch.terminal_goals < policy.minimum_terminal_goals_per_epoch
            for epoch in recent_epochs
        ):
            unmet.append("insufficient_terminal_goals_per_epoch")
        if consecutive_low_yield_epoch_count(evidence.history, policy) < (
            policy.minimum_low_yield_epochs
        ):
            # 只检查最近连续 epoch，防止从历史中挑两个低收益轮而忽略刚发生的
            # 大幅扩图。按终态目标归一化，避免“epoch 运行更久”天然累积更多栅格；
            # 仍沿用旧恢复策略的双门槛：绝对值与相对值都显著才算高收益。
            unmet.append("recent_epochs_still_material")

    if evidence.history.counters.mapping_path_m < policy.minimum_mapping_path_m:
        unmet.append("insufficient_mapping_path")
    if (
        evidence.residual_available_frontiers
        > policy.maximum_residual_available_frontiers
    ):
        unmet.append("too_many_residual_frontiers")
    if evidence.active_goal_count or evidence.pending_goal_count:
        unmet.append("action_ledger_not_drained")

    probe = evidence.history.final_probe
    if probe is None:
        unmet.append("final_probe_missing")
    else:
        if not probe.started_after_hard_budget:
            unmet.append("final_probe_not_after_hard_budget")
        if (
            probe.map_gain_cells >= policy.maximum_low_yield_gain_cells
            or probe.map_gain_ratio >= policy.maximum_low_yield_gain_ratio
        ):
            unmet.append("final_probe_still_material")

    if evidence.map_quiet_s < policy.required_map_quiet_s:
        unmet.append("map_not_quiet")
    if not evidence.typed_stop_confirmed:
        unmet.append("final_stop_missing")
    if not evidence.typed_stop_after_final_probe:
        unmet.append("final_stop_not_after_probe")
    if (
        evidence.final_stop_age_s is None
        or evidence.final_stop_age_s > policy.maximum_final_stop_age_s
    ):
        unmet.append("final_stop_stale")

    return SaturationAssessment(
        decision=(
            SaturationDecision.COMPLETE
            if not unmet
            else SaturationDecision.CONTINUE
        ),
        unmet_requirements=tuple(unmet),
    )
