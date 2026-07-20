"""采集建图证据；领域状态不依赖 ROS 消息和 Node 类型。"""

from __future__ import annotations

from dataclasses import dataclass, replace
from enum import IntEnum
import math
import threading
import time
from typing import Iterable


@dataclass(frozen=True, slots=True)
class FrontierTelemetry:
    """Explore Lite 一次发布的完整 frontier/Action 生命周期快照。"""

    status: str = ""
    detected_frontier_count: int = 0
    available_frontier_count: int = 0
    blacklisted_frontier_count: int = 0
    active_goal_count: int = 0
    active_goal_id: str = ""
    accepted_goal_count: int = 0
    succeeded_goal_count: int = 0
    aborted_goal_count: int = 0
    canceled_goal_count: int = 0
    rejected_goal_count: int = 0
    last_goal_terminal: str = ""
    completion_reason: str = ""

    def __post_init__(self) -> None:
        counters = (
            self.detected_frontier_count,
            self.available_frontier_count,
            self.blacklisted_frontier_count,
            self.active_goal_count,
            self.accepted_goal_count,
            self.succeeded_goal_count,
            self.aborted_goal_count,
            self.canceled_goal_count,
            self.rejected_goal_count,
        )
        if any(value < 0 for value in counters):
            raise ValueError("frontier telemetry counters must be non-negative")


class NavigationGoalStatus(IntEnum):
    """与强类型 ROS 证据消息保持一致的任务层目标状态。"""

    UNSPECIFIED = 0
    PLANNED = 1
    REQUESTED = 2
    ACCEPTED = 3
    EXECUTING = 4
    SUCCEEDED = 5
    REJECTED = 6
    ABORTED = 7
    CANCELED = 8
    TIMED_OUT = 9


_TERMINAL_NAVIGATION_STATUSES = frozenset(
    {
        NavigationGoalStatus.SUCCEEDED,
        NavigationGoalStatus.REJECTED,
        NavigationGoalStatus.ABORTED,
        NavigationGoalStatus.CANCELED,
        NavigationGoalStatus.TIMED_OUT,
    }
)


@dataclass(frozen=True, slots=True)
class NavigationGoalEvidence:
    sequence: int
    goal_xy: tuple[float, float]
    status: NavigationGoalStatus = NavigationGoalStatus.PLANNED
    started_at_ns: int = 0
    finished_at_ns: int = 0
    nav2_status: int = 0
    nav2_error_code: int = 0
    plan_count: int = 0
    max_unknown_cell_count: int = 0
    max_occupied_cell_count: int = 0
    max_outside_map_cell_count: int = 0
    detail: str = ""

    @property
    def all_plans_known_free(self) -> bool:
        return (
            self.plan_count > 0
            and self.max_unknown_cell_count == 0
            and self.max_occupied_cell_count == 0
            and self.max_outside_map_cell_count == 0
        )


class NavigationGoalLedger:
    """串行目标的线程安全证据账本；Node 只负责 ROS Adapter。"""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._mission_sequence = 0
        self._records: list[NavigationGoalEvidence] = []

    @property
    def mission_sequence(self) -> int:
        with self._lock:
            return self._mission_sequence

    def reset(self, mission_sequence: int) -> None:
        if mission_sequence <= 0:
            raise ValueError("mission sequence must be positive")
        with self._lock:
            self._mission_sequence = mission_sequence
            self._records = []

    def plan(self, goals: Iterable[tuple[float, float]]) -> None:
        records = [
            NavigationGoalEvidence(
                sequence=index,
                goal_xy=(float(goal[0]), float(goal[1])),
            )
            for index, goal in enumerate(goals, start=1)
        ]
        if not records:
            raise ValueError("at least one navigation goal is required")
        with self._lock:
            self._records = records

    def transition(
        self,
        sequence: int,
        status: NavigationGoalStatus,
        *,
        timestamp_ns: int,
        nav2_status: int = 0,
        nav2_error_code: int = 0,
        detail: str = "",
    ) -> None:
        if timestamp_ns < 0:
            raise ValueError("navigation evidence timestamp must be non-negative")
        with self._lock:
            index = self._index(sequence)
            current = self._records[index]
            if current.status in _TERMINAL_NAVIGATION_STATUSES:
                raise ValueError(f"navigation goal {sequence} is already terminal")
            started_at_ns = current.started_at_ns
            if status is NavigationGoalStatus.REQUESTED and started_at_ns == 0:
                started_at_ns = timestamp_ns
            finished_at_ns = (
                timestamp_ns if status in _TERMINAL_NAVIGATION_STATUSES else 0
            )
            self._records[index] = replace(
                current,
                status=status,
                started_at_ns=started_at_ns,
                finished_at_ns=finished_at_ns,
                nav2_status=int(nav2_status),
                nav2_error_code=int(nav2_error_code),
                detail=str(detail),
            )

    def record_plan(
        self,
        sequence: int,
        *,
        unknown_cell_count: int,
        occupied_cell_count: int,
        outside_map_cell_count: int,
    ) -> None:
        counts = (
            unknown_cell_count,
            occupied_cell_count,
            outside_map_cell_count,
        )
        if any(value < 0 for value in counts):
            raise ValueError("path occupancy counts must be non-negative")
        with self._lock:
            index = self._index(sequence)
            current = self._records[index]
            self._records[index] = replace(
                current,
                plan_count=current.plan_count + 1,
                max_unknown_cell_count=max(
                    current.max_unknown_cell_count, unknown_cell_count
                ),
                max_occupied_cell_count=max(
                    current.max_occupied_cell_count, occupied_cell_count
                ),
                max_outside_map_cell_count=max(
                    current.max_outside_map_cell_count, outside_map_cell_count
                ),
            )

    def active_sequence(self) -> int | None:
        with self._lock:
            active = [
                item.sequence
                for item in self._records
                if item.status
                in {
                    NavigationGoalStatus.REQUESTED,
                    NavigationGoalStatus.ACCEPTED,
                    NavigationGoalStatus.EXECUTING,
                }
            ]
        if len(active) > 1:
            raise RuntimeError("sampled navigation goals must execute serially")
        return active[0] if active else None

    def get(self, sequence: int) -> NavigationGoalEvidence:
        """读取单个不可变快照，供 ROS Adapter 做 endpoint/终态校验。"""

        with self._lock:
            return self._records[self._index(sequence)]

    def active(self) -> NavigationGoalEvidence | None:
        sequence = self.active_sequence()
        return self.get(sequence) if sequence is not None else None

    def is_terminal(self, sequence: int) -> bool:
        return self.get(sequence).status in _TERMINAL_NAVIGATION_STATUSES

    def snapshot(self) -> tuple[NavigationGoalEvidence, ...]:
        with self._lock:
            return tuple(self._records)

    def _index(self, sequence: int) -> int:
        index = int(sequence) - 1
        if index < 0 or index >= len(self._records):
            raise IndexError(f"unknown navigation goal sequence: {sequence}")
        return index


@dataclass(frozen=True, slots=True)
class MappingEvidenceSnapshot:
    """供 frontier 判定和验收报告使用的一致状态快照。"""

    map_stats: dict[str, int] | None
    best_known_map_cells: int
    last_map_growth_at: float
    last_frontier_activity_at: float
    exploration_completion_reason: str
    mapping_path_m: float
    explore_status: str
    frontier_telemetry: FrontierTelemetry


class MappingEvidenceTracker:
    """用同一条件锁管理地图、里程计和探索器的可变观测。"""

    def __init__(self, min_growth_cells: int, *, clock=time.monotonic) -> None:
        if min_growth_cells < 0:
            raise ValueError("min_growth_cells must be non-negative")
        self._min_growth_cells = min_growth_cells
        self._clock = clock
        self.condition = threading.Condition()
        self.scan_ready = threading.Event()
        self._map_stats: dict[str, int] | None = None
        self._best_known_map_cells = 0
        self._last_map_growth_at = clock()
        self._last_frontier_activity_at = clock()
        self._exploration_completion_reason = ""
        self._mapping_path_m = 0.0
        self._mapping_last_position: tuple[float, float] | None = None
        self._record_mapping_path = False
        self._explore_status = ""
        self._frontier_telemetry = FrontierTelemetry()
        self._frontier_action_totals = {
            "accepted_goal_count": 0,
            "succeeded_goal_count": 0,
            "aborted_goal_count": 0,
            "canceled_goal_count": 0,
            "rejected_goal_count": 0,
        }

    def record_map(self, values: Iterable[int]) -> None:
        known_cells = 0
        occupied_cells = 0
        for value in values:
            if value >= 0:
                known_cells += 1
            if value >= 65:
                occupied_cells += 1
        stats = {
            "known_cells": known_cells,
            "occupied_cells": occupied_cells,
        }
        with self.condition:
            self._map_stats = stats
            if known_cells >= self._best_known_map_cells + self._min_growth_cells:
                self._best_known_map_cells = known_cells
                self._last_map_growth_at = self._clock()
            self.condition.notify_all()

    def mark_scan_ready(self) -> None:
        self.scan_ready.set()

    def record_odom(self, x: float, y: float) -> None:
        current = (float(x), float(y))
        with self.condition:
            if not self._record_mapping_path:
                return
            previous = self._mapping_last_position
            self._mapping_last_position = current
            if previous is None:
                return
            step = math.hypot(current[0] - previous[0], current[1] - previous[1])
            # Gazebo reset 或定位跳变不能计入真实建图里程。
            if 0.001 <= step <= 0.5:
                self._mapping_path_m += step
            self.condition.notify_all()

    def record_explore_status(self, status: str) -> None:
        """兼容旧 ExploreStatus；unknown-world 必须改用完整 telemetry。"""

        self.record_frontier_telemetry(FrontierTelemetry(status=status))

    def record_frontier_telemetry(self, telemetry: FrontierTelemetry) -> None:
        with self.condition:
            previous = self._frontier_telemetry
            if telemetry != previous:
                # 周期性重复发布不算“进展”；只有 frontier/Action 状态真正改变
                # 才刷新时钟。比较完整值对象还能捕获 active_goal_id 交接，
                # 既覆盖 terminal cooldown，又不会被心跳消息无限推迟停滞检测。
                self._last_frontier_activity_at = self._clock()
            self._frontier_telemetry = telemetry
            self._explore_status = telemetry.status
            self.condition.notify_all()

    def begin_mapping_path(self) -> None:
        with self.condition:
            self._mapping_path_m = 0.0
            self._mapping_last_position = None
            self._record_mapping_path = True

    def finish_mapping_path(self) -> None:
        with self.condition:
            self._record_mapping_path = False

    def resume_mapping_path(self) -> None:
        """恢复累计而不清零；恢复动作本身已在暂停窗口中被排除。"""

        with self.condition:
            self._mapping_last_position = None
            self._record_mapping_path = True

    def reset_exploration(self, *, preserve_action_totals: bool = False) -> None:
        with self.condition:
            if preserve_action_totals:
                for name in self._frontier_action_totals:
                    self._frontier_action_totals[name] += int(
                        getattr(self._frontier_telemetry, name)
                    )
            else:
                for name in self._frontier_action_totals:
                    self._frontier_action_totals[name] = 0
            self._explore_status = ""
            self._best_known_map_cells = 0
            self._last_map_growth_at = self._clock()
            self._last_frontier_activity_at = self._clock()
            self._exploration_completion_reason = ""
            self._frontier_telemetry = FrontierTelemetry()
            self.condition.notify_all()

    def set_completion_reason(self, reason: str) -> None:
        with self.condition:
            self._exploration_completion_reason = reason

    def snapshot(self) -> MappingEvidenceSnapshot:
        with self.condition:
            current = self._frontier_telemetry
            # Explore Lite 每次恢复都会重启并把计数归零；终端验收需要跨尝试证明
            # 所有已接受目标都有终态，因此累计动作计数，但保留当前 frontier 数量。
            telemetry = replace(
                current,
                **{
                    name: int(getattr(current, name)) + total
                    for name, total in self._frontier_action_totals.items()
                },
            )
            return MappingEvidenceSnapshot(
                map_stats=dict(self._map_stats) if self._map_stats else None,
                best_known_map_cells=self._best_known_map_cells,
                last_map_growth_at=self._last_map_growth_at,
                last_frontier_activity_at=self._last_frontier_activity_at,
                exploration_completion_reason=self._exploration_completion_reason,
                mapping_path_m=self._mapping_path_m,
                explore_status=self._explore_status,
                frontier_telemetry=telemetry,
            )

    @property
    def map_stats(self) -> dict[str, int] | None:
        return self.snapshot().map_stats

    @property
    def last_map_growth_at(self) -> float:
        return self.snapshot().last_map_growth_at

    @property
    def completion_reason(self) -> str:
        return self.snapshot().exploration_completion_reason

    @completion_reason.setter
    def completion_reason(self, reason: str) -> None:
        self.set_completion_reason(reason)

    @property
    def mapping_path_m(self) -> float:
        return self.snapshot().mapping_path_m

    @property
    def explore_status(self) -> str:
        return self.snapshot().explore_status

    @property
    def frontier_telemetry(self) -> FrontierTelemetry:
        return self.snapshot().frontier_telemetry
