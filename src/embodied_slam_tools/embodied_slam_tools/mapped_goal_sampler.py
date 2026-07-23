"""从本次 OccupancyGrid 的已知自由连通区抽取可复现导航目标。"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import random


Cell = tuple[int, int]
Point2D = tuple[float, float]


class GoalSamplingError(RuntimeError):
    """地图中没有足够的安全、连通目标。"""


@dataclass(frozen=True, slots=True)
class OccupancySnapshot:
    """无 ROS 依赖的 OccupancyGrid 快照；cell 行序与 ROS data 一致。"""

    width: int
    height: int
    resolution_m: float
    origin_xy: Point2D
    cells: tuple[int, ...]

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("occupancy dimensions must be positive")
        if self.resolution_m <= 0.0:
            raise ValueError("occupancy resolution must be positive")
        if len(self.cells) != self.width * self.height:
            raise ValueError("occupancy cell count does not match dimensions")

    def value(self, cell: Cell) -> int:
        column, row = cell
        return self.cells[row * self.width + column]

    def contains(self, cell: Cell) -> bool:
        column, row = cell
        return 0 <= column < self.width and 0 <= row < self.height

    def world_to_cell(self, point: Point2D) -> Cell | None:
        column = math.floor((point[0] - self.origin_xy[0]) / self.resolution_m)
        row = math.floor((point[1] - self.origin_xy[1]) / self.resolution_m)
        cell = (column, row)
        return cell if self.contains(cell) else None

    def cell_center(self, cell: Cell) -> Point2D:
        column, row = cell
        return (
            self.origin_xy[0] + (column + 0.5) * self.resolution_m,
            self.origin_xy[1] + (row + 0.5) * self.resolution_m,
        )


@dataclass(frozen=True, slots=True)
class PathOccupancyStats:
    """一条规划路径在当前 OccupancyGrid 中的安全采样计数。"""

    sample_count: int
    unknown_cell_count: int
    occupied_cell_count: int
    outside_map_cell_count: int

    @property
    def known_free(self) -> bool:
        return (
            self.sample_count > 0
            and self.unknown_cell_count == 0
            and self.occupied_cell_count == 0
            and self.outside_map_cell_count == 0
        )


def inspect_path_occupancy(
    snapshot: OccupancySnapshot,
    points: tuple[Point2D, ...],
) -> PathOccupancyStats:
    """以半栅格步长检查整条 path，而不是只信 Nav2 的离散 pose。"""

    if not points:
        raise ValueError("navigation path must contain at least one point")
    dense: list[Point2D] = [points[0]]
    maximum_step_m = snapshot.resolution_m * 0.5
    for start, end in zip(points, points[1:]):
        distance = math.dist(start, end)
        steps = max(1, math.ceil(distance / maximum_step_m))
        dense.extend(
            (
                start[0] + (end[0] - start[0]) * index / steps,
                start[1] + (end[1] - start[1]) * index / steps,
            )
            for index in range(1, steps + 1)
        )
    unknown = 0
    occupied = 0
    outside = 0
    for point in dense:
        cell = snapshot.world_to_cell(point)
        if cell is None:
            outside += 1
            continue
        value = snapshot.value(cell)
        if value < 0:
            unknown += 1
        elif value != 0:
            occupied += 1
    return PathOccupancyStats(
        sample_count=len(dense),
        unknown_cell_count=unknown,
        occupied_cell_count=occupied,
        outside_map_cell_count=outside,
    )


def _clearance_offsets(
    resolution_m: float, clearance_m: float
) -> tuple[Cell, ...]:
    if not math.isfinite(clearance_m) or clearance_m < 0.0:
        raise ValueError("clearance must be non-negative")
    # ROS OccupancyGrid 的 resolution 来自 float32，0.05m 读回 Python 后可能略大。
    # 若按“栅格中心距离”比较，第 5 格会因微小浮点误差被漏掉。这里改为计算
    # 候选点到整个栅格方块边缘的最短距离，更符合实体障碍占据一个 cell 的语义。
    radius = math.ceil(clearance_m / resolution_m + 0.5)
    tolerance_m = max(1e-12, resolution_m * 1e-9)
    return tuple(
        (column_delta, row_delta)
        for row_delta in range(-radius, radius + 1)
        for column_delta in range(-radius, radius + 1)
        if math.hypot(
            max(0.0, (abs(column_delta) - 0.5) * resolution_m),
            max(0.0, (abs(row_delta) - 0.5) * resolution_m),
        )
        <= clearance_m + tolerance_m
    )


def _safe_free_cells(
    snapshot: OccupancySnapshot, clearance_m: float
) -> frozenset[Cell]:
    offsets = _clearance_offsets(snapshot.resolution_m, clearance_m)
    safe: set[Cell] = set()
    for row in range(snapshot.height):
        for column in range(snapshot.width):
            cell = (column, row)
            if snapshot.value(cell) != 0:
                continue
            if all(
                snapshot.contains((column + dx, row + dy))
                and snapshot.value((column + dx, row + dy)) == 0
                for dx, dy in offsets
            ):
                # unknown 与 occupied 都不是可通行假设；统一膨胀可防止目标落在
                # 地图边界或尚未观测区域旁，随后被 Nav2 costmap 拒绝。
                safe.add(cell)
    return frozenset(safe)


def _connected_component(start: Cell, safe: frozenset[Cell]) -> frozenset[Cell]:
    queue = deque([start])
    connected = {start}
    while queue:
        column, row = queue.popleft()
        for neighbor in (
            (column - 1, row),
            (column + 1, row),
            (column, row - 1),
            (column, row + 1),
        ):
            if neighbor in safe and neighbor not in connected:
                connected.add(neighbor)
                queue.append(neighbor)
    return frozenset(connected)


def _rank_farthest_goals(
    candidates: tuple[Point2D, ...],
    *,
    start_xy: Point2D,
    candidate_limit: int,
    seed: int,
    minimum_separation_m: float,
) -> tuple[Point2D, ...]:
    rng = random.Random(seed)
    remaining = list(candidates)
    # seed 只打破等距候选，不编码场景；主排序始终选择离现有目标最远的点。
    rng.shuffle(remaining)
    selected: list[Point2D] = []
    anchors: list[Point2D] = [start_xy]
    while remaining and len(selected) < candidate_limit:
        best = max(
            remaining,
            key=lambda point: min(math.dist(point, anchor) for anchor in anchors),
        )
        distance = min(math.dist(best, point) for point in selected) if selected else math.inf
        if distance + 1e-12 < minimum_separation_m:
            break
        selected.append(best)
        anchors.append(best)
        remaining.remove(best)
    return tuple(selected)


def rank_mapped_goal_candidates(
    snapshot: OccupancySnapshot,
    *,
    start_xy: Point2D,
    seed: int,
    minimum_separation_m: float,
    clearance_m: float,
    candidate_limit: int,
) -> tuple[Point2D, ...]:
    """从本次地图连通域返回确定性排序的安全目标候选。

    连通性使用 raw known-free cell，目标落点再单独应用 clearance。起点可能贴近
    新建地图边缘而不适合作为目标，但这不代表其所在自由连通域不可遍历。这里不
    复制 Nav2 inflation，只为任务层证明候选来自本次地图且目标自身具有净空。
    """

    if candidate_limit <= 0:
        raise ValueError("candidate limit must be positive")
    if not math.isfinite(minimum_separation_m) or minimum_separation_m < 0.0:
        raise ValueError("minimum separation must be non-negative")

    start = snapshot.world_to_cell(start_xy)
    if start is None or snapshot.value(start) != 0:
        raise GoalSamplingError("robot start is not in known free space")

    known_free = frozenset(
        (column, row)
        for row in range(snapshot.height)
        for column in range(snapshot.width)
        if snapshot.value((column, row)) == 0
    )
    reachable = _connected_component(start, known_free)
    goal_safe = _safe_free_cells(snapshot, clearance_m)
    candidates = tuple(
        snapshot.cell_center(cell)
        for cell in sorted(reachable & goal_safe)
        if cell != start
    )
    # 隔墙后的 free cell 不在 reachable 中；即使 goal-safe，也不会进入任务候选。
    return _rank_farthest_goals(
        candidates,
        start_xy=start_xy,
        candidate_limit=candidate_limit,
        seed=seed,
        minimum_separation_m=minimum_separation_m,
    )
