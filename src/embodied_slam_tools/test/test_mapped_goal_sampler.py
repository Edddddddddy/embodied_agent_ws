"""已建图自由空间目标采样器的纯领域契约。"""

from __future__ import annotations

import math

import pytest

from embodied_slam_tools.mapped_goal_sampler import (
    OccupancySnapshot,
    inspect_path_occupancy,
    rank_mapped_goal_candidates,
)


def _snapshot(rows: list[list[int]], *, resolution_m: float = 1.0) -> OccupancySnapshot:
    """用人类易读的地图行创建 ROS 风格的 row-major 快照。"""

    assert rows and rows[0]
    width = len(rows[0])
    assert all(len(row) == width for row in rows)
    return OccupancySnapshot(
        width=width,
        height=len(rows),
        resolution_m=resolution_m,
        origin_xy=(0.0, 0.0),
        cells=tuple(value for row in rows for value in row),
    )


def _cell(point_xy: tuple[float, float], *, resolution_m: float = 1.0) -> tuple[int, int]:
    return (
        int(math.floor(point_xy[0] / resolution_m)),
        int(math.floor(point_xy[1] / resolution_m)),
    )


def test_samples_three_spaced_goals_reproducibly_from_start_component():
    snapshot = _snapshot([[0] * 9 for _ in range(9)])

    first = rank_mapped_goal_candidates(
        snapshot,
        start_xy=(0.5, 0.5),
        seed=20260719,
        minimum_separation_m=3.0,
        clearance_m=0.0,
        candidate_limit=3,
    )
    second = rank_mapped_goal_candidates(
        snapshot,
        start_xy=(0.5, 0.5),
        seed=20260719,
        minimum_separation_m=3.0,
        clearance_m=0.0,
        candidate_limit=3,
    )

    assert first == second
    assert len(first) == 3
    assert all(
        math.dist(left, right) >= 3.0
        for index, left in enumerate(first)
        for right in first[index + 1 :]
    )


def test_unknown_and_occupied_cells_apply_the_same_clearance_exclusion():
    rows = [[0] * 11 for _ in range(11)]
    rows[5][3] = 100
    rows[5][7] = -1
    snapshot = _snapshot(rows)

    goals = rank_mapped_goal_candidates(
        snapshot,
        # 起点远离地图外缘；地图外同样属于 unknown 风险带，不应绕过 clearance。
        start_xy=(5.5, 2.5),
        seed=7,
        minimum_separation_m=0.0,
        clearance_m=1.5,
        candidate_limit=8,
    )

    # unknown 不是“尚未确认的自由区”；与实体障碍一样膨胀，防止目标落在地图边界风险带。
    blocked_centres = ((3.5, 5.5), (7.5, 5.5))
    assert all(
        math.dist(goal, blocked) > 1.5
        for goal in goals
        for blocked in blocked_centres
    )


def test_never_samples_free_cells_behind_an_occupied_wall():
    rows = [[0] * 9 for _ in range(7)]
    for row in rows:
        row[4] = 100
    snapshot = _snapshot(rows)

    goals = rank_mapped_goal_candidates(
        snapshot,
        start_xy=(1.5, 3.5),
        seed=11,
        minimum_separation_m=0.0,
        clearance_m=0.0,
        candidate_limit=5,
    )

    # 右侧虽然是已知 free，但与机器人不连通，不能伪装成可导航验收目标。
    assert all(_cell(goal)[0] < 4 for goal in goals)


def test_float_resolution_does_not_shrink_fifth_cell_clearance_ring():
    resolution = 0.05000000074505806
    rows = [[0] * 21 for _ in range(21)]
    rows[10][5] = 100
    snapshot = _snapshot(rows, resolution_m=resolution)

    goals = rank_mapped_goal_candidates(
        snapshot,
        start_xy=((15.5 * resolution), (15.5 * resolution)),
        seed=9,
        minimum_separation_m=0.0,
        clearance_m=0.25,
        candidate_limit=500,
    )
    cells = {_cell(goal, resolution_m=resolution) for goal in goals}

    # 障碍中心相距第 5 格时，ROS float resolution 略大于 0.05；该栅格方块
    # 仍侵入 0.25m clearance，不能因绝对 1e-12 比较而漏掉整圈。
    assert (10, 10) not in cells
    assert (11, 10) in cells


def test_goal_clearance_does_not_require_start_itself_to_be_goal_safe():
    rows = [[0] * 17 for _ in range(17)]
    snapshot = _snapshot(rows)

    goals = rank_mapped_goal_candidates(
        snapshot,
        # 起点贴地图边缘，不能作为目标；但它位于 raw known-free 连通域中。
        start_xy=(0.5, 0.5),
        seed=4,
        minimum_separation_m=0.0,
        clearance_m=1.0,
        candidate_limit=6,
    )

    assert len(goals) == 6
    assert all(1 <= x < 16 and 1 <= y < 16 for x, y in map(_cell, goals))


def test_ranked_candidates_are_deterministic_and_respect_limit():
    snapshot = _snapshot([[0] * 13 for _ in range(13)])
    arguments = dict(
        start_xy=(6.5, 6.5),
        seed=20260719,
        minimum_separation_m=1.0,
        clearance_m=0.0,
        candidate_limit=8,
    )

    first = rank_mapped_goal_candidates(snapshot, **arguments)
    second = rank_mapped_goal_candidates(snapshot, **arguments)

    assert first == second
    assert len(first) == 8
    with pytest.raises(ValueError, match="candidate limit must be positive"):
        rank_mapped_goal_candidates(
            snapshot,
            **{**arguments, "candidate_limit": 0},
        )


def test_ranked_candidates_never_cross_an_occupied_wall():
    rows = [[0] * 11 for _ in range(9)]
    for row in rows:
        row[5] = 100
    snapshot = _snapshot(rows)

    goals = rank_mapped_goal_candidates(
        snapshot,
        start_xy=(1.5, 4.5),
        seed=5,
        minimum_separation_m=0.0,
        clearance_m=0.0,
        candidate_limit=100,
    )

    assert goals
    assert all(_cell(goal)[0] < 5 for goal in goals)


def test_path_occupancy_densifies_between_free_endpoints():
    rows = [[0] * 5 for _ in range(5)]
    rows[2][2] = -1
    snapshot = _snapshot(rows)

    stats = inspect_path_occupancy(snapshot, ((0.5, 0.5), (4.5, 4.5)))

    assert stats.known_free is False
    assert stats.unknown_cell_count > 0


def test_path_occupancy_counts_outside_map_separately():
    snapshot = _snapshot([[0] * 3 for _ in range(3)])

    stats = inspect_path_occupancy(snapshot, ((0.5, 0.5), (4.0, 0.5)))

    assert stats.known_free is False
    assert stats.outside_map_cell_count > 0
