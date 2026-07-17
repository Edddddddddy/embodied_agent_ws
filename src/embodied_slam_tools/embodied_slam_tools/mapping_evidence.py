"""采集建图证据；领域状态不依赖 ROS 消息和 Node 类型。"""

from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Iterable


@dataclass(frozen=True, slots=True)
class MappingEvidenceSnapshot:
    """供 frontier 判定和验收报告使用的一致状态快照。"""

    map_stats: dict[str, int] | None
    best_known_map_cells: int
    last_map_growth_at: float
    exploration_completion_reason: str
    mapping_path_m: float
    explore_status: str


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
        self._exploration_completion_reason = ""
        self._mapping_path_m = 0.0
        self._mapping_last_position: tuple[float, float] | None = None
        self._record_mapping_path = False
        self._explore_status = ""

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
        with self.condition:
            self._explore_status = status
            self.condition.notify_all()

    def begin_mapping_path(self) -> None:
        with self.condition:
            self._mapping_path_m = 0.0
            self._mapping_last_position = None
            self._record_mapping_path = True

    def finish_mapping_path(self) -> None:
        with self.condition:
            self._record_mapping_path = False

    def reset_exploration(self) -> None:
        with self.condition:
            self._explore_status = ""
            self._best_known_map_cells = 0
            self._last_map_growth_at = self._clock()
            self._exploration_completion_reason = ""

    def set_completion_reason(self, reason: str) -> None:
        with self.condition:
            self._exploration_completion_reason = reason

    def snapshot(self) -> MappingEvidenceSnapshot:
        with self.condition:
            return MappingEvidenceSnapshot(
                map_stats=dict(self._map_stats) if self._map_stats else None,
                best_known_map_cells=self._best_known_map_cells,
                last_map_growth_at=self._last_map_growth_at,
                exploration_completion_reason=self._exploration_completion_reason,
                mapping_path_m=self._mapping_path_m,
                explore_status=self._explore_status,
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
