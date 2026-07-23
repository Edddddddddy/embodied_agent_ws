"""Unknown-world SLAM 验收证据；静态真值只能在本模块中用于离线评分。"""

from __future__ import annotations

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Iterable, Mapping

import yaml


FREE = 0
OCCUPIED = 1
UNKNOWN = -1
MISSION_OUTCOME_SUCCEEDED = 2
NAVIGATION_GOAL_STATUS_SUCCEEDED = 5
NAV2_GOAL_STATUS_SUCCEEDED = 4

_DYNAMIC_NAVIGATION_CHECKS = frozenset(
    {
        "gazebo_entity_moved",
        "tracker_confident",
        "tracker_estimated_motion",
        "future_cell_marked_lethal",
        "dynamic_path_increased_clearance",
        "nav2_replanned",
        "navigate_to_pose_succeeded",
        "robot_moved",
        "cmd_vel_zero",
    }
)

_RETURN_TO_START_CHECKS = frozenset(
    {
        "typed_action_command",
        "typed_action_succeeded",
        "return_before_map_save",
        "same_pose_frame",
        "final_pose_in_return_window",
        "xy_within_tolerance",
        "yaw_within_tolerance",
        "cmd_vel_after_return",
        "final_cmd_vel_fresh",
        "final_cmd_vel_zero",
    }
)


@dataclass(frozen=True, slots=True)
class MapQualityThresholds:
    minimum_reachable_coverage_ratio: float
    minimum_region_coverage_ratio: float
    maximum_reachable_unknown_ratio: float
    minimum_obstacle_boundary_recall_ratio: float = 0.60
    maximum_obstacle_false_free_ratio: float = 0.05

    def __post_init__(self) -> None:
        values = (
            self.minimum_reachable_coverage_ratio,
            self.minimum_region_coverage_ratio,
            self.maximum_reachable_unknown_ratio,
            self.minimum_obstacle_boundary_recall_ratio,
            self.maximum_obstacle_false_free_ratio,
        )
        if any(not 0.0 <= value <= 1.0 for value in values):
            raise ValueError("map quality thresholds must be within [0, 1]")


@dataclass(frozen=True, slots=True)
class MapRegion:
    name: str
    min_x_m: float
    min_y_m: float
    max_x_m: float
    max_y_m: float

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("map region name must be non-empty")
        if self.min_x_m >= self.max_x_m or self.min_y_m >= self.max_y_m:
            raise ValueError(f"map region {self.name!r} has invalid bounds")

    def contains(self, x_m: float, y_m: float) -> bool:
        return (
            self.min_x_m <= x_m < self.max_x_m
            and self.min_y_m <= y_m < self.max_y_m
        )


@dataclass(frozen=True, slots=True)
class LocalizationSample:
    """已由采集 Adapter 转换到同一坐标系的二维定位样本。"""

    timestamp_s: float
    x_m: float
    y_m: float

    def __post_init__(self) -> None:
        if not all(
            math.isfinite(value)
            for value in (self.timestamp_s, self.x_m, self.y_m)
        ):
            raise ValueError("localization samples must contain finite values")


@dataclass(frozen=True, slots=True)
class MapWorldTransform:
    """Evaluator-only 的 Gazebo world 与本次 SLAM map 固定 SE(2) 变换。"""

    spawn_world_x_m: float
    spawn_world_y_m: float
    spawn_world_yaw_rad: float

    def world_to_map(self, x_m: float, y_m: float) -> tuple[float, float]:
        delta_x = x_m - self.spawn_world_x_m
        delta_y = y_m - self.spawn_world_y_m
        cosine = math.cos(self.spawn_world_yaw_rad)
        sine = math.sin(self.spawn_world_yaw_rad)
        return (
            cosine * delta_x + sine * delta_y,
            -sine * delta_x + cosine * delta_y,
        )

    def map_to_world(self, x_m: float, y_m: float) -> tuple[float, float]:
        cosine = math.cos(self.spawn_world_yaw_rad)
        sine = math.sin(self.spawn_world_yaw_rad)
        return (
            self.spawn_world_x_m + cosine * x_m - sine * y_m,
            self.spawn_world_y_m + sine * x_m + cosine * y_m,
        )


@dataclass(frozen=True, slots=True)
class NavigationGoalObservation:
    """一个动态采样目标及其实际 Nav2 规划证据。"""

    x_m: float
    y_m: float
    succeeded: bool
    planned_paths: tuple[tuple[tuple[float, float], ...], ...]
    producer_evidence: Mapping[str, object] | None = None

    def __post_init__(self) -> None:
        if not math.isfinite(self.x_m) or not math.isfinite(self.y_m):
            raise ValueError("navigation goal coordinates must be finite")
        for path in self.planned_paths:
            for point in path:
                if len(point) != 2 or not all(math.isfinite(value) for value in point):
                    raise ValueError("navigation paths must contain finite x/y points")


@dataclass(frozen=True, slots=True)
class SceneEvaluationContext:
    world_name: str
    transform: MapWorldTransform
    regions: tuple[MapRegion, ...]


@dataclass(frozen=True, slots=True)
class UnknownWorldThresholds:
    map_quality: MapQualityThresholds
    maximum_position_error_p95_m: float = 0.25
    maximum_alignment_gap_s: float = 0.10
    minimum_aligned_samples: int = 20
    minimum_navigation_goal_count: int = 3
    minimum_navigation_goal_separation_m: float = 1.50


@dataclass(frozen=True, slots=True)
class UnknownWorldObservation:
    session_id: str
    session_start_ns: int
    mission_profile_unknown: bool
    mission_sequence: int
    mission_completed: bool
    mission_outcome: int
    mission_message: str
    map_saved: bool
    built_map_yaml: Path
    truth_map_yaml: Path
    robot_start_xy: tuple[float, float]
    regions: tuple[MapRegion, ...]
    map_provenance: Mapping[str, object] | None
    frontier_telemetry: Mapping[str, object]
    navigation_goals: tuple[NavigationGoalObservation, ...]
    amcl_samples: tuple[LocalizationSample, ...]
    gazebo_samples: tuple[LocalizationSample, ...]
    gazebo_truth_error: str
    nav2_lifecycle_active: bool
    dynamic_navigation: Mapping[str, object] | None
    final_linear_x: float
    final_angular_z: float
    cmd_vel_sample_count: int = 0
    nonzero_cmd_vel_sample_count: int = 0
    last_cmd_vel_received_at_s: float = 0.0
    final_stop_boundary_at_s: float = 0.0
    cmd_vel_samples_after_boundary: int = 0
    nonzero_cmd_vel_samples_after_boundary: int = 0
    # 运行时只提供与场景尺寸无关的饱和证据；是否达到地图质量门槛仍由
    # evaluator 使用 truth map 独立判定，避免任务层“自己宣布自己完成”。
    mapping_completion_evidence: Mapping[str, object] | None = None
    # 以下三项是展示/诊断字段，不参与 PASS 判定；仍由同一 ROS 会话采集，
    # 避免控制台在链路成功时打印 null，给现场验收造成“证据缺失”的误解。
    final_phase: int = 0
    mapping_path_m: float = 0.0
    frontier_goal_count: int = 0
    source_revision: str = ""
    source_dirty: bool | None = None


@dataclass(frozen=True, slots=True)
class _OccupancyMap:
    source: Path
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    origin_yaw: float
    cells: tuple[int, ...]

    def state(self, row: int, column: int) -> int:
        return self.cells[row * self.width + column]

    def cell_center_world(self, row: int, column: int) -> tuple[float, float]:
        local_x = (column + 0.5) * self.resolution
        local_y = (self.height - row - 0.5) * self.resolution
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        return (
            self.origin_x + cosine * local_x - sine * local_y,
            self.origin_y + sine * local_x + cosine * local_y,
        )

    def world_to_cell(self, x_m: float, y_m: float) -> tuple[int, int] | None:
        delta_x = x_m - self.origin_x
        delta_y = y_m - self.origin_y
        cosine = math.cos(self.origin_yaw)
        sine = math.sin(self.origin_yaw)
        # ROS map origin 可以带 yaw；先做逆旋转，再按 PGM 自上而下的行序换算。
        local_x = cosine * delta_x + sine * delta_y
        local_y = -sine * delta_x + cosine * delta_y
        column = math.floor(local_x / self.resolution)
        grid_y = math.floor(local_y / self.resolution)
        row = self.height - 1 - grid_y
        if not (0 <= row < self.height and 0 <= column < self.width):
            return None
        return row, column

    def state_at_world(self, x_m: float, y_m: float) -> int:
        cell = self.world_to_cell(x_m, y_m)
        if cell is None:
            # SLAM 尚未扩展到的地图外区域与 unknown 等价，不能算作已建图。
            return UNKNOWN
        return self.state(*cell)


def _next_pgm_token(data: bytes, offset: int) -> tuple[bytes, int]:
    size = len(data)
    while offset < size:
        if data[offset] in b" \t\r\n":
            offset += 1
            continue
        if data[offset] == ord("#"):
            newline = data.find(b"\n", offset)
            offset = size if newline < 0 else newline + 1
            continue
        break
    start = offset
    while offset < size and data[offset] not in b" \t\r\n#":
        offset += 1
    if start == offset:
        raise ValueError("unexpected end of PGM header")
    return data[start:offset], offset


def _read_pgm(path: Path) -> tuple[int, int, int, tuple[int, ...]]:
    data = path.read_bytes()
    magic, offset = _next_pgm_token(data, 0)
    width_token, offset = _next_pgm_token(data, offset)
    height_token, offset = _next_pgm_token(data, offset)
    max_value_token, offset = _next_pgm_token(data, offset)
    width = int(width_token)
    height = int(height_token)
    max_value = int(max_value_token)
    if width <= 0 or height <= 0 or not 0 < max_value <= 255:
        raise ValueError(f"invalid PGM metadata in {path}")

    expected = width * height
    if magic == b"P2":
        values: list[int] = []
        while len(values) < expected:
            token, offset = _next_pgm_token(data, offset)
            values.append(int(token))
    elif magic == b"P5":
        if offset >= len(data) or data[offset] not in b" \t\r\n":
            raise ValueError(f"missing PGM raster separator in {path}")
        if data[offset : offset + 2] == b"\r\n":
            offset += 2
        else:
            offset += 1
        values = list(data[offset : offset + expected])
    else:
        raise ValueError(f"unsupported PGM format {magic!r} in {path}")
    if len(values) != expected:
        raise ValueError(
            f"PGM raster size mismatch in {path}: {len(values)} != {expected}"
        )
    if any(value < 0 or value > max_value for value in values):
        raise ValueError(f"PGM sample outside declared range in {path}")
    return width, height, max_value, tuple(values)


def _load_map(yaml_path: Path) -> _OccupancyMap:
    yaml_path = yaml_path.resolve()
    document = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError(f"map yaml must contain a mapping: {yaml_path}")
    image_value = str(document.get("image", "")).strip()
    if not image_value:
        raise ValueError(f"map yaml does not define image: {yaml_path}")
    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    width, height, max_value, pixels = _read_pgm(image_path.resolve())
    resolution = float(document["resolution"])
    origin = document["origin"]
    if resolution <= 0.0 or not isinstance(origin, (list, tuple)) or len(origin) < 3:
        raise ValueError(f"invalid map resolution/origin: {yaml_path}")
    negate = int(document.get("negate", 0))
    occupied_threshold = float(document.get("occupied_thresh", 0.65))
    free_threshold = float(document.get("free_thresh", 0.196))
    if not 0.0 <= free_threshold < occupied_threshold <= 1.0:
        raise ValueError(f"invalid occupancy thresholds: {yaml_path}")

    cells: list[int] = []
    for pixel in pixels:
        normalized = pixel / max_value
        occupancy = normalized if negate else 1.0 - normalized
        if occupancy > occupied_threshold:
            cells.append(OCCUPIED)
        elif occupancy < free_threshold:
            cells.append(FREE)
        else:
            cells.append(UNKNOWN)
    return _OccupancyMap(
        source=yaml_path,
        width=width,
        height=height,
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        origin_yaw=float(origin[2]),
        cells=tuple(cells),
    )


def _clearance_offsets(resolution: float, clearance_m: float) -> tuple[tuple[int, int], ...]:
    if clearance_m < 0.0:
        raise ValueError("robot clearance must be non-negative")
    radius = math.ceil(clearance_m / resolution)
    return tuple(
        (row_delta, column_delta)
        for row_delta in range(-radius, radius + 1)
        for column_delta in range(-radius, radius + 1)
        if math.hypot(row_delta * resolution, column_delta * resolution)
        <= clearance_m + 1e-12
    )


def _clearance_safe_cells(
    truth: _OccupancyMap, clearance_m: float
) -> frozenset[tuple[int, int]]:
    offsets = _clearance_offsets(truth.resolution, clearance_m)
    safe: set[tuple[int, int]] = set()
    for row in range(truth.height):
        for column in range(truth.width):
            if truth.state(row, column) != FREE:
                continue
            if all(
                0 <= row + row_delta < truth.height
                and 0 <= column + column_delta < truth.width
                and truth.state(row + row_delta, column + column_delta) == FREE
                for row_delta, column_delta in offsets
            ):
                safe.add((row, column))
    return frozenset(safe)


def _reachable_component(
    truth: _OccupancyMap,
    robot_start_xy: tuple[float, float],
    clearance_m: float,
) -> frozenset[tuple[int, int]]:
    safe = _clearance_safe_cells(truth, clearance_m)
    start = truth.world_to_cell(*robot_start_xy)
    if start not in safe:
        raise ValueError(
            "robot start is outside the clearance-safe truth free space: "
            f"{robot_start_xy}"
        )
    queue = deque([start])
    reachable = {start}
    while queue:
        row, column = queue.popleft()
        for candidate in (
            (row - 1, column),
            (row + 1, column),
            (row, column - 1),
            (row, column + 1),
        ):
            if candidate in safe and candidate not in reachable:
                reachable.add(candidate)
                queue.append(candidate)
    return frozenset(reachable)


def _reachable_obstacle_boundary(
    truth: _OccupancyMap,
    robot_start_xy: tuple[float, float],
) -> frozenset[tuple[int, int]]:
    """返回机器人可达自由区能观测到的真值障碍边界。

    覆盖率只检查 free/unknown，若没有这层约束，把整张图涂成 free 也会得到
    高分。这里仅评分与可达自由连通域相邻的墙体，避免要求激光穿墙看到场外物体。
    """

    raw_reachable = _reachable_component(truth, robot_start_xy, 0.0)
    boundary: set[tuple[int, int]] = set()
    for row, column in raw_reachable:
        for row_delta in (-1, 0, 1):
            for column_delta in (-1, 0, 1):
                if row_delta == 0 and column_delta == 0:
                    continue
                candidate = row + row_delta, column + column_delta
                if (
                    0 <= candidate[0] < truth.height
                    and 0 <= candidate[1] < truth.width
                    and truth.state(*candidate) == OCCUPIED
                ):
                    boundary.add(candidate)
    return frozenset(boundary)


def _occupied_near_world(
    occupancy: _OccupancyMap,
    x_m: float,
    y_m: float,
    tolerance_m: float,
) -> bool:
    """容忍 SLAM 墙面相对真值产生少量栅格偏移。"""

    center = occupancy.world_to_cell(x_m, y_m)
    if center is None:
        return False
    cell_radius = max(1, math.ceil(tolerance_m / occupancy.resolution))
    # 用半个栅格对角线补偿 cell-center 量化误差，但不能无限放宽到邻室墙体。
    center_tolerance = tolerance_m + occupancy.resolution / math.sqrt(2.0)
    for row_delta in range(-cell_radius, cell_radius + 1):
        for column_delta in range(-cell_radius, cell_radius + 1):
            candidate = center[0] + row_delta, center[1] + column_delta
            if not (
                0 <= candidate[0] < occupancy.height
                and 0 <= candidate[1] < occupancy.width
            ):
                continue
            if occupancy.state(*candidate) != OCCUPIED:
                continue
            candidate_xy = occupancy.cell_center_world(*candidate)
            if math.hypot(candidate_xy[0] - x_m, candidate_xy[1] - y_m) <= (
                center_tolerance + 1e-12
            ):
                return True
    return False


def _obstacle_boundary_metrics(
    *,
    truth: _OccupancyMap,
    built: _OccupancyMap,
    robot_start_xy: tuple[float, float],
) -> dict[str, float | int]:
    boundary = _reachable_obstacle_boundary(truth, robot_start_xy)
    tolerance_m = max(0.10, 1.5 * max(truth.resolution, built.resolution))
    represented = 0
    false_free = 0
    for row, column in boundary:
        x_m, y_m = truth.cell_center_world(row, column)
        matched = _occupied_near_world(built, x_m, y_m, tolerance_m)
        represented += matched
        # unknown 代表“尚未观测”，由 recall 失败处理；只有明确涂成 free 才计入
        # false-free，便于区分漏建和危险的穿墙地图。
        false_free += not matched and built.state_at_world(x_m, y_m) == FREE
    return {
        "observable_obstacle_boundary_cell_count": len(boundary),
        "represented_obstacle_boundary_cell_count": represented,
        "false_free_obstacle_boundary_cell_count": false_free,
        "obstacle_boundary_recall_ratio": (
            _ratio(represented, len(boundary)) if boundary else 1.0
        ),
        "obstacle_false_free_ratio": (
            _ratio(false_free, len(boundary)) if boundary else 0.0
        ),
        "obstacle_match_tolerance_m": tolerance_m,
    }


def _ratio(numerator: int, denominator: int) -> float:
    return numerator / denominator if denominator else 0.0


def _unique_timestamp_samples(
    samples: tuple[LocalizationSample, ...],
) -> tuple[tuple[LocalizationSample, ...], int]:
    """按时间戳去重，避免重复消息把定位有效样本数虚增。"""

    unique: list[LocalizationSample] = []
    duplicate_count = 0
    for sample in samples:
        if unique and sample.timestamp_s == unique[-1].timestamp_s:
            duplicate_count += 1
            continue
        unique.append(sample)
    return tuple(unique), duplicate_count


def _aligned_truth_sample(
    sample: LocalizationSample,
    truth_samples: tuple[LocalizationSample, ...],
    maximum_gap_s: float,
) -> tuple[LocalizationSample, tuple[int, int], float] | None:
    """返回时间对齐真值、支撑帧和最大时间间隔。

    两侧真值都足够近时做线性插值；否则退化到最近帧。支撑帧 ID 由
    调用方用于一对一计数，防止一帧 Gazebo truth 被大量 AMCL 回调复用。
    """

    if not truth_samples:
        return None

    timestamps = tuple(candidate.timestamp_s for candidate in truth_samples)
    right_index = bisect_left(timestamps, sample.timestamp_s)
    if right_index < len(truth_samples) and (
        truth_samples[right_index].timestamp_s == sample.timestamp_s
    ):
        return truth_samples[right_index], (right_index, right_index), 0.0

    if right_index == 0:
        nearest_index = 0
    elif right_index == len(truth_samples):
        nearest_index = len(truth_samples) - 1
    else:
        left_index = right_index - 1
        left = truth_samples[left_index]
        right = truth_samples[right_index]
        left_gap = sample.timestamp_s - left.timestamp_s
        right_gap = right.timestamp_s - sample.timestamp_s
        if (
            left_gap <= maximum_gap_s + 1e-12
            and right_gap <= maximum_gap_s + 1e-12
        ):
            ratio = left_gap / (right.timestamp_s - left.timestamp_s)
            interpolated = LocalizationSample(
                timestamp_s=sample.timestamp_s,
                x_m=left.x_m + ratio * (right.x_m - left.x_m),
                y_m=left.y_m + ratio * (right.y_m - left.y_m),
            )
            return interpolated, (left_index, right_index), max(left_gap, right_gap)
        nearest_index = left_index if left_gap <= right_gap else right_index

    nearest = truth_samples[nearest_index]
    gap_s = abs(nearest.timestamp_s - sample.timestamp_s)
    if gap_s > maximum_gap_s + 1e-12:
        return None
    return nearest, (nearest_index, nearest_index), gap_s


def _nearest_rank_percentile(values: Iterable[float], percentile: float) -> float | None:
    ordered = sorted(values)
    if not ordered:
        return None
    rank = max(1, math.ceil(percentile * len(ordered)))
    return ordered[rank - 1]


def evaluate_localization_quality(
    *,
    amcl_samples: Iterable[LocalizationSample],
    gazebo_samples: Iterable[LocalizationSample],
    maximum_position_error_p95_m: float,
    maximum_alignment_gap_s: float,
    minimum_aligned_samples: int,
) -> dict[str, object]:
    """按时间对齐定位与真值，并以位置误差 P95 判定质量。"""

    if maximum_position_error_p95_m < 0.0:
        raise ValueError("maximum position error must be non-negative")
    if maximum_alignment_gap_s < 0.0:
        raise ValueError("maximum alignment gap must be non-negative")
    if minimum_aligned_samples <= 0:
        raise ValueError("minimum aligned samples must be positive")
    raw_amcl = tuple(sorted(amcl_samples, key=lambda sample: sample.timestamp_s))
    raw_gazebo = tuple(sorted(gazebo_samples, key=lambda sample: sample.timestamp_s))
    amcl, duplicate_amcl_count = _unique_timestamp_samples(raw_amcl)
    gazebo, duplicate_gazebo_count = _unique_timestamp_samples(raw_gazebo)
    time_ranges_overlap = bool(amcl and gazebo) and (
        max(amcl[0].timestamp_s, gazebo[0].timestamp_s)
        <= min(amcl[-1].timestamp_s, gazebo[-1].timestamp_s)
    )
    # 每组 truth 支撑帧只保留时间间隔最小的 AMCL 样本。否则一个暂停的真值
    # 帧可被高频重复定位消息复用，轻易凑够 minimum_aligned_samples。
    aligned_by_support: dict[
        tuple[int, int], tuple[float, LocalizationSample, LocalizationSample]
    ] = {}
    if time_ranges_overlap:
        for sample in amcl:
            aligned = _aligned_truth_sample(
                sample, gazebo, maximum_alignment_gap_s
            )
            if aligned is None:
                continue
            truth, support, gap_s = aligned
            previous = aligned_by_support.get(support)
            if previous is None or gap_s < previous[0]:
                aligned_by_support[support] = (gap_s, sample, truth)
    errors = [
        math.hypot(sample.x_m - truth.x_m, sample.y_m - truth.y_m)
        for _, sample, truth in aligned_by_support.values()
    ]
    p95 = _nearest_rank_percentile(errors, 0.95)
    checks = {
        "time_ranges_overlap": time_ranges_overlap,
        "unique_sample_timestamps": (
            duplicate_amcl_count == 0 and duplicate_gazebo_count == 0
        ),
        "minimum_aligned_samples": len(errors) >= minimum_aligned_samples,
        # P95 能暴露持续漂移，又不会让单帧传输毛刺支配整场验收；输入坐标系
        # 的对齐属于 ROS/Gazebo Adapter 责任，证据层不猜测场景出生点。
        "position_error_p95": p95 is not None
        and p95 <= maximum_position_error_p95_m + 1e-12,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": {
            "amcl_sample_count": len(raw_amcl),
            "gazebo_sample_count": len(raw_gazebo),
            "unique_amcl_sample_count": len(amcl),
            "unique_gazebo_sample_count": len(gazebo),
            "duplicate_amcl_timestamp_count": duplicate_amcl_count,
            "duplicate_gazebo_timestamp_count": duplicate_gazebo_count,
            "aligned_sample_count": len(errors),
            "alignment_support_count": len(aligned_by_support),
            "position_error_p95_m": p95,
            "position_error_max_m": max(errors) if errors else None,
        },
        "thresholds": {
            "maximum_position_error_p95_m": maximum_position_error_p95_m,
            "maximum_alignment_gap_s": maximum_alignment_gap_s,
            "minimum_aligned_samples": minimum_aligned_samples,
        },
    }


def _densify_path(
    path: tuple[tuple[float, float], ...], maximum_step_m: float
) -> tuple[tuple[float, float], ...]:
    """补齐相邻 path pose 之间的栅格采样，避免只检查离散端点而漏掉穿墙。"""

    if maximum_step_m <= 0.0:
        raise ValueError("path sampling step must be positive")
    if not path:
        return ()
    points = [path[0]]
    for start, end in zip(path, path[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        steps = max(1, math.ceil(distance / maximum_step_m))
        points.extend(
            (
                start[0] + (end[0] - start[0]) * index / steps,
                start[1] + (end[1] - start[1]) * index / steps,
            )
            for index in range(1, steps + 1)
        )
    return tuple(points)


def evaluate_sampled_navigation(
    *,
    built_map_yaml: Path,
    goals: Iterable[NavigationGoalObservation],
    minimum_goal_count: int,
    minimum_goal_separation_m: float = 0.0,
) -> dict[str, object]:
    """验证动态目标、Action 终态与实际规划路径均落在本次已知自由区。"""

    if minimum_goal_count <= 0:
        raise ValueError("minimum navigation goal count must be positive")
    if minimum_goal_separation_m < 0.0:
        raise ValueError("minimum goal separation must be non-negative")
    occupancy = _load_map(Path(built_map_yaml))
    observations = tuple(goals)
    goal_reports: list[dict[str, object]] = []
    for index, observation in enumerate(observations):
        path_reports: list[dict[str, object]] = []
        for path in observation.planned_paths:
            # 半个栅格分辨率可覆盖斜向线段；若只检查 Nav2 发布的稀疏 pose，
            # 两个自由端点之间仍可能跨过 unknown/occupied cell。
            dense = _densify_path(path, occupancy.resolution * 0.5)
            states = [occupancy.state_at_world(x_m, y_m) for x_m, y_m in dense]
            path_reports.append(
                {
                    "point_count": len(path),
                    "sample_count": len(states),
                    "unknown_sample_count": sum(state == UNKNOWN for state in states),
                    "occupied_sample_count": sum(state == OCCUPIED for state in states),
                    "passed": bool(states)
                    and all(state == FREE for state in states),
                }
            )
        goal_free = occupancy.state_at_world(
            observation.x_m, observation.y_m
        ) == FREE
        producer = observation.producer_evidence
        started_at = float(producer.get("started_at", 0.0)) if producer else 0.0
        finished_at = float(producer.get("finished_at", 0.0)) if producer else 0.0
        producer_known_free = bool(
            producer
            and str(producer.get("frame_id", "")) == "map"
            and int(producer.get("plan_count", 0)) > 0
            and int(producer.get("max_unknown_cell_count", 0)) == 0
            and int(producer.get("max_occupied_cell_count", 0)) == 0
            and int(producer.get("max_outside_map_cell_count", 0)) == 0
            and producer.get("all_plans_known_free") is True
        )
        producer_lifecycle_valid = bool(
            producer
            and int(producer.get("sequence", 0)) > 0
            and int(producer.get("status", 0))
            == NAVIGATION_GOAL_STATUS_SUCCEEDED
            # typed producer 成功只能证明编排器写入了终态；仍需 Nav2 Action
            # 自身 SUCCEEDED 且 error_code=0，避免上层误报掩盖规划/控制失败。
            and int(producer.get("nav2_status", 0))
            == NAV2_GOAL_STATUS_SUCCEEDED
            and int(producer.get("nav2_error_code", -1)) == 0
            and started_at > 0.0
            and finished_at >= started_at
        )
        producer_goal_matches = bool(
            producer
            and math.hypot(
                float(producer.get("x", math.inf)) - observation.x_m,
                float(producer.get("y", math.inf)) - observation.y_m,
            )
            <= 1e-6
        )
        checks = {
            "goal_in_known_free_space": goal_free,
            "action_succeeded": observation.succeeded,
            "producer_lifecycle_valid": producer_lifecycle_valid,
            "producer_goal_matches": producer_goal_matches,
            "producer_plan_evidence_known_free": producer_known_free,
            "plan_observed": bool(path_reports),
            "all_plans_known_free": bool(path_reports)
            and all(bool(item["passed"]) for item in path_reports),
        }
        goal_reports.append(
            {
                "index": index,
                "goal": {"x": observation.x_m, "y": observation.y_m},
                "checks": checks,
                "plans": path_reports,
                "producer_evidence": dict(producer) if producer else None,
                "passed": all(checks.values()),
            }
        )
    producer_sequences = [
        int(item.producer_evidence.get("sequence", 0))
        for item in observations
        if item.producer_evidence
    ]
    pair_distances = [
        math.hypot(left.x_m - right.x_m, left.y_m - right.y_m)
        for index, left in enumerate(observations)
        for right in observations[index + 1 :]
    ]
    minimum_observed_separation = min(pair_distances) if pair_distances else None
    checks = {
        "minimum_goal_count": len(observations) >= minimum_goal_count,
        "unique_positive_goal_sequences": (
            len(producer_sequences) == len(observations)
            and all(sequence > 0 for sequence in producer_sequences)
            and len(set(producer_sequences)) == len(producer_sequences)
        ),
        # 三次“成功”若落在同一个点并不能证明导航闭环；目标间距必须由最终
        # evaluator 再验一次，不能只相信采样器的运行时配置。
        "minimum_goal_separation": (
            minimum_observed_separation is None
            or minimum_observed_separation + 1e-12
            >= minimum_goal_separation_m
        ),
        "all_goals_succeeded_with_known_free_paths": bool(goal_reports)
        and all(bool(item["passed"]) for item in goal_reports),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": {
            "goal_count": len(observations),
            "succeeded_goal_count": sum(item.succeeded for item in observations),
            "minimum_goal_separation_m": minimum_observed_separation,
        },
        "goals": goal_reports,
        "thresholds": {
            "minimum_goal_count": minimum_goal_count,
            "minimum_goal_separation_m": minimum_goal_separation_m,
        },
        "map": str(occupancy.source),
    }


def evaluate_frontier_completion(telemetry: Mapping[str, object]) -> dict[str, object]:
    """验证 Explore Lite 结束时没有活动、可达或被掩盖的黑名单 frontier。"""

    available = int(telemetry.get("available_frontier_count", 0))
    active = int(telemetry.get("active_goal_count", 0))
    blacklisted = int(telemetry.get("blacklisted_frontier_count", 0))
    detected = int(telemetry.get("detected_frontier_count", 0))
    accepted = int(telemetry.get("accepted_goal_count", 0))
    terminal = sum(
        int(telemetry.get(name, 0))
        for name in (
            "succeeded_goal_count",
            "aborted_goal_count",
            "canceled_goal_count",
        )
    )
    provider_reason = str(
        telemetry.get(
            "provider_completion_reason",
            telemetry.get("completion_reason", ""),
        )
    )
    mission_reason = str(telemetry.get("mission_completion_reason", ""))
    exhausted_reason_pairs = {
        (
            "frontier_attempts_exhausted_recoverable",
            "frontier_attempts_exhausted_below_material_gain",
        ),
        (
            "frontier_attempts_exhausted_recoverable",
            "frontier_attempts_exhausted_after_final_confirmation",
        ),
    }
    accepted_reason_pairs = {
        ("no_frontiers", "no_reachable_frontiers"),
        *exhausted_reason_pairs,
    }
    reason_pair = (provider_reason, mission_reason)
    residual_blacklist_is_terminal = (
        reason_pair in exhausted_reason_pairs
        and detected > 0
        and 0 <= blacklisted <= detected
    )
    checks = {
        "typed_evidence_valid": telemetry.get("valid") is True,
        # provider 原因与任务层结论必须成对出现；交叉组合会把“本轮尝试耗尽”
        # 偷换成“环境没有 frontier”，因此即使最终计数都是零也必须拒绝。
        "completion_reason_pair": (
            provider_reason,
            mission_reason,
        ) in accepted_reason_pairs,
        "no_reachable_frontier": available == 0,
        "no_active_goal": active == 0,
        # no_frontiers 必须没有黑名单；多视角恢复预算已消费完时，provider
        # 的黑名单只是本 epoch 已尝试失败的候选子集，可由独立地图质量门禁
        # 最终否决，但绝不能大于 detected 或与错误 reason pair 混用。
        "blacklist_consistent_with_reason": (
            blacklisted == 0 or residual_blacklist_is_terminal
        ),
        "all_accepted_goals_terminal": accepted == terminal,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": {
            "available_frontier_count": available,
            "active_goal_count": active,
            "blacklisted_frontier_count": blacklisted,
            "detected_frontier_count": detected,
            "accepted_goal_count": accepted,
            "terminal_goal_count": terminal,
        },
        "telemetry": dict(telemetry),
    }


def evaluate_return_to_start_evidence(
    mapping_completion: Mapping[str, object] | None,
) -> dict[str, object]:
    """独立复核建图阶段的返航证据，strict 与近似完成共用同一门禁。

    运行时报告中的 ``passed`` 只是 producer 结论。这里仍逐项检查强类型 Action
    请求/结果、位姿容差、返航后新鲜零速度，以及“返航完成早于地图保存”。这样
    strict frontier 收敛也不能绕过返航，避免保存一张地图后在导航阶段重启 Gazebo
    再声称“已经回到起点”。
    """

    completion = dict(mapping_completion) if mapping_completion else {}
    return_home_value = completion.get("return_home")
    return_home = (
        dict(return_home_value)
        if isinstance(return_home_value, Mapping)
        else {}
    )
    raw_checks = return_home.get("checks")
    producer_checks = dict(raw_checks) if isinstance(raw_checks, Mapping) else {}
    failed_checks = return_home.get("failed_checks")
    thresholds_value = return_home.get("thresholds")
    errors_value = return_home.get("errors")
    thresholds = (
        dict(thresholds_value)
        if isinstance(thresholds_value, Mapping)
        else {}
    )
    errors = dict(errors_value) if isinstance(errors_value, Mapping) else {}

    def finite_positive(name: str) -> float | None:
        try:
            value = float(thresholds[name])
        except (KeyError, TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value > 0.0 else None

    def finite_non_negative(name: str) -> float | None:
        try:
            value = float(errors[name])
        except (KeyError, TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value >= 0.0 else None

    max_xy_error_m = finite_positive("max_xy_error_m")
    max_yaw_error_rad = finite_positive("max_yaw_error_rad")
    zero_velocity_tolerance = finite_positive("zero_velocity_tolerance")
    max_cmd_vel_age_s = finite_positive("max_cmd_vel_age_s")
    xy_error_m = finite_non_negative("xy_error_m")
    yaw_error_rad = finite_non_negative("yaw_error_rad")
    required_checks_present = _RETURN_TO_START_CHECKS.issubset(producer_checks)
    all_producer_checks_pass = bool(producer_checks) and all(
        value is True for value in producer_checks.values()
    )

    checks = {
        "typed_mapping_completion_evidence": (
            completion.get("valid") is True
            and completion.get("mode")
            in {"strict_frontier", "bounded_saturation"}
        ),
        "return_report_schema_v1": return_home.get("schema_version") == 1,
        "return_report_passed": return_home.get("passed") is True,
        "required_return_checks_present": required_checks_present,
        "typed_action_command": producer_checks.get("typed_action_command")
        is True,
        "typed_action_succeeded": producer_checks.get("typed_action_succeeded")
        is True,
        "return_before_map_save": producer_checks.get("return_before_map_save")
        is True,
        "same_pose_frame": producer_checks.get("same_pose_frame") is True,
        "final_pose_in_return_window": producer_checks.get(
            "final_pose_in_return_window"
        )
        is True,
        "xy_within_tolerance": (
            producer_checks.get("xy_within_tolerance") is True
            and xy_error_m is not None
            and max_xy_error_m is not None
            and xy_error_m <= max_xy_error_m
        ),
        "yaw_within_tolerance": (
            producer_checks.get("yaw_within_tolerance") is True
            and yaw_error_rad is not None
            and max_yaw_error_rad is not None
            and max_yaw_error_rad <= math.pi
            and yaw_error_rad <= max_yaw_error_rad
        ),
        "cmd_vel_after_return": producer_checks.get("cmd_vel_after_return")
        is True,
        "final_cmd_vel_fresh": (
            producer_checks.get("final_cmd_vel_fresh") is True
            and max_cmd_vel_age_s is not None
        ),
        "final_cmd_vel_zero": (
            producer_checks.get("final_cmd_vel_zero") is True
            and zero_velocity_tolerance is not None
        ),
        # required-set 防旧报告缺字段；all-values 防未来新增检查失败却被忽略。
        "all_producer_checks_pass": all_producer_checks_pass,
        "no_failed_checks": (
            isinstance(failed_checks, (list, tuple)) and not failed_checks
        ),
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "mode": completion.get("mode"),
        "producer_report": return_home or None,
        "metrics": {
            "xy_error_m": xy_error_m,
            "yaw_error_rad": yaw_error_rad,
        },
        "thresholds": {
            "max_xy_error_m": max_xy_error_m,
            "max_yaw_error_rad": max_yaw_error_rad,
            "zero_velocity_tolerance": zero_velocity_tolerance,
            "max_cmd_vel_age_s": max_cmd_vel_age_s,
        },
    }


def _return_to_start_evidence_valid(
    mapping_completion: Mapping[str, object] | None,
) -> bool:
    """兼容内部调用；唯一真相来自完整返航 evaluator。"""

    return bool(evaluate_return_to_start_evidence(mapping_completion)["passed"])


def evaluate_approximate_completion(
    *,
    telemetry: Mapping[str, object],
    evidence: Mapping[str, object] | None,
    map_quality_passed: bool,
    final_cmd_vel_fresh: bool,
    final_cmd_vel_zero: bool,
) -> dict[str, object]:
    """派生“近似完成”，运行时超时原因本身不具有完成语义。

    unknown-world 运行时看不到真值地图和场景总面积，所以它只能报告“硬时间
    预算耗尽”或“重复可达停滞”这类有界触发。最终是否允许收口，由本函数把
    运行时证据、离线地图质量、Action 总账、返航和真实零速度合并后独立派生。
    """

    completion = dict(evidence) if evidence else {}
    provider_reason = str(
        telemetry.get(
            "provider_completion_reason",
            telemetry.get("completion_reason", ""),
        )
    )
    mission_reason = str(telemetry.get("mission_completion_reason", ""))
    trigger_reason = str(completion.get("trigger_reason", ""))
    bounded_runtime_reasons = {
        "time_budget_exhausted",
        "reachable_frontiers_stalled_bounded_saturation",
    }
    active = int(telemetry.get("active_goal_count", 0))
    available = int(telemetry.get("available_frontier_count", 0))
    accepted = int(telemetry.get("accepted_goal_count", 0))
    terminal = sum(
        int(telemetry.get(name, 0))
        for name in (
            "succeeded_goal_count",
            "aborted_goal_count",
            "canceled_goal_count",
        )
    )

    try:
        low_yield_epochs = int(completion.get("low_yield_epoch_count", -1))
        required_epochs = int(
            completion.get("required_low_yield_epoch_count", -1)
        )
        residual = int(completion.get("residual_available_frontiers", -1))
        final_gain_cells = int(completion.get("final_probe_gain_cells", -1))
        final_gain_ratio = float(completion.get("final_probe_gain_ratio", -1.0))
    except (TypeError, ValueError):
        low_yield_epochs = required_epochs = residual = final_gain_cells = -1
        final_gain_ratio = -1.0

    checks = {
        # 两种 reason 都只是“允许进入独立证据评估”的中性运行时事实；
        # provider 不能自行伪造完成，producer 写入的触发原因也必须逐字一致。
        "neutral_runtime_reason": (
            mission_reason in bounded_runtime_reasons and not provider_reason
        ),
        "trigger_reason_matches": trigger_reason == mission_reason,
        "typed_saturation_evidence_valid": (
            completion.get("mode") == "bounded_saturation"
            and completion.get("valid") is True
        ),
        "consecutive_low_yield_epochs": (
            required_epochs >= 2 and low_yield_epochs >= required_epochs
        ),
        "residual_frontier_count_consistent": (
            residual >= 0 and residual == available
        ),
        "final_probe_recorded": (
            final_gain_cells >= 0
            and math.isfinite(final_gain_ratio)
            and final_gain_ratio >= 0.0
        ),
        "no_active_goal": active == 0,
        "all_accepted_goals_terminal": (
            completion.get("ledger_drained") is True and accepted == terminal
        ),
        "typed_stop_succeeded": completion.get("typed_stop_succeeded") is True,
        "map_quality": map_quality_passed,
        "return_to_start": _return_to_start_evidence_valid(completion),
        "final_cmd_vel_fresh": final_cmd_vel_fresh,
        "final_cmd_vel_zero": final_cmd_vel_zero,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": {
            "available_frontier_count": available,
            "active_goal_count": active,
            "accepted_goal_count": accepted,
            "terminal_goal_count": terminal,
            "low_yield_epoch_count": low_yield_epochs,
            "required_low_yield_epoch_count": required_epochs,
            "final_probe_gain_cells": final_gain_cells,
            "final_probe_gain_ratio": final_gain_ratio,
        },
        "runtime_reason": mission_reason,
        "trigger_reason": trigger_reason,
        "evidence": completion or None,
        "return_to_start": evaluate_return_to_start_evidence(completion),
    }


def load_scene_evaluation_context(scene_yaml: Path) -> SceneEvaluationContext:
    """从 evaluator-only 场景清单构造坐标变换与区域边界。"""

    document = yaml.safe_load(Path(scene_yaml).read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("scene evaluator manifest must contain a mapping")
    world = document.get("world")
    if not isinstance(world, dict):
        raise ValueError("scene evaluator manifest is missing world")
    spawn = world.get("spawn")
    if not isinstance(spawn, dict):
        raise ValueError("scene evaluator manifest is missing world.spawn")
    transform = MapWorldTransform(
        spawn_world_x_m=float(spawn["x"]),
        spawn_world_y_m=float(spawn["y"]),
        spawn_world_yaw_rad=float(spawn.get("yaw", 0.0)),
    )
    regions: list[MapRegion] = []
    for item in document.get("objects", ()):
        if not isinstance(item, dict) or item.get("category") != "floor":
            continue
        size = item.get("size")
        if not isinstance(size, (list, tuple)) or len(size) < 2:
            raise ValueError(f"floor {item.get('name', '')!r} has invalid size")
        half_x = float(size[0]) * 0.5
        half_y = float(size[1]) * 0.5
        corners = tuple(
            transform.world_to_map(
                float(item["x"]) + dx,
                float(item["y"]) + dy,
            )
            for dx in (-half_x, half_x)
            for dy in (-half_y, half_y)
        )
        regions.append(
            MapRegion(
                name=str(item["name"]).removesuffix("_floor"),
                min_x_m=min(point[0] for point in corners),
                min_y_m=min(point[1] for point in corners),
                max_x_m=max(point[0] for point in corners),
                max_y_m=max(point[1] for point in corners),
            )
        )
    if not regions:
        raise ValueError("scene evaluator manifest defines no floor regions")
    return SceneEvaluationContext(
        world_name=str(world["name"]),
        transform=transform,
        regions=tuple(sorted(regions, key=lambda item: item.name)),
    )


def _dynamic_navigation_evidence_valid(
    evidence: Mapping[str, object] | None,
) -> bool:
    """复核动态避障报告的原始检查项，而不是信任一个可手写的 passed 位。"""

    if not evidence or evidence.get("passed") is not True:
        return False
    checks = evidence.get("checks")
    if not isinstance(checks, Mapping):
        return False
    if not _DYNAMIC_NAVIGATION_CHECKS.issubset(checks):
        return False
    if not all(checks.get(name) is True for name in _DYNAMIC_NAVIGATION_CHECKS):
        return False
    # required-set 防止旧报告缺字段；all-values 防止未来新增失败项却被外层忽略。
    if not all(value is True for value in checks.values()):
        return False
    try:
        published_plan_count = int(evidence.get("published_plan_count", 0))
        unique_plan_count = int(evidence.get("unique_plan_count", 0))
        nav2_status = int(evidence.get("navigate_to_pose_status", 0))
    except (TypeError, ValueError):
        return False
    return bool(str(evidence.get("scenario_id", "")).strip()) and (
        nav2_status == NAV2_GOAL_STATUS_SUCCEEDED
        and published_plan_count >= unique_plan_count >= 1
    )


def build_unknown_world_report(
    observation: UnknownWorldObservation,
    thresholds: UnknownWorldThresholds,
) -> dict[str, object]:
    """生成 unknown-world schema v4 的唯一 PASS/FAIL 判定。"""

    map_quality = evaluate_unknown_world_map(
        built_map_yaml=observation.built_map_yaml,
        truth_map_yaml=observation.truth_map_yaml,
        robot_start_xy=observation.robot_start_xy,
        regions=observation.regions,
        thresholds=thresholds.map_quality,
    )
    localization = evaluate_localization_quality(
        amcl_samples=observation.amcl_samples,
        gazebo_samples=observation.gazebo_samples,
        maximum_position_error_p95_m=(
            thresholds.maximum_position_error_p95_m
        ),
        maximum_alignment_gap_s=thresholds.maximum_alignment_gap_s,
        minimum_aligned_samples=thresholds.minimum_aligned_samples,
    )
    frontier = evaluate_frontier_completion(observation.frontier_telemetry)
    navigation = evaluate_sampled_navigation(
        built_map_yaml=observation.built_map_yaml,
        goals=observation.navigation_goals,
        minimum_goal_count=thresholds.minimum_navigation_goal_count,
        minimum_goal_separation_m=(
            thresholds.minimum_navigation_goal_separation_m
        ),
    )
    provenance = observation.map_provenance
    fresh_map = bool(
        provenance
        and observation.session_start_ns > 0
        and int(provenance.get("yaml_mtime_ns", 0))
        >= observation.session_start_ns
        and int(provenance.get("image_mtime_ns", 0))
        >= observation.session_start_ns
    )
    final_cmd_vel_fresh = (
        observation.final_stop_boundary_at_s > 0.0
        and observation.last_cmd_vel_received_at_s
        >= observation.final_stop_boundary_at_s
    )
    final_cmd_vel_zero = abs(observation.final_linear_x) < 1e-3 and abs(
        observation.final_angular_z
    ) < 1e-3
    return_to_start = evaluate_return_to_start_evidence(
        observation.mapping_completion_evidence
    )
    approximate_completion = evaluate_approximate_completion(
        telemetry=observation.frontier_telemetry,
        evidence=observation.mapping_completion_evidence,
        map_quality_passed=bool(map_quality["passed"]),
        # approximate 分支比旧 strict 路径多要求一帧可计数的边界后 Twist，
        # 防止仅靠初始化时间戳拼出“新鲜零速度”。
        final_cmd_vel_fresh=(
            final_cmd_vel_fresh
            and observation.cmd_vel_samples_after_boundary > 0
        ),
        final_cmd_vel_zero=final_cmd_vel_zero,
    )
    checks = {
        "unknown_world_profile": observation.mission_profile_unknown,
        "mission_sequence_present": observation.mission_sequence > 0,
        "mission_completed": observation.mission_completed,
        # phase 只表示 ROS 栈当前所处阶段；导航失败后栈可能仍保持 NAVIGATING。
        # 因此最终 PASS 必须同时看到强类型 mission_outcome=SUCCEEDED。
        "mission_outcome_succeeded": (
            observation.mission_outcome == MISSION_OUTCOME_SUCCEEDED
        ),
        "map_saved": observation.map_saved,
        "fresh_session_map": fresh_map,
        "map_quality": bool(map_quality["passed"]),
        "frontier_complete": bool(frontier["passed"])
        or bool(approximate_completion["passed"]),
        # strict 与 bounded-saturation 都必须在建图进程仍运行时真实返航，
        # 并在返航完成之后保存地图；二者不能拥有不同的安全收口标准。
        "return_to_start": bool(return_to_start["passed"]),
        "localization_quality": bool(localization["passed"])
        and not observation.gazebo_truth_error,
        "sampled_navigation": bool(navigation["passed"]),
        "nav2_lifecycle_active": observation.nav2_lifecycle_active,
        "dynamic_navigation": _dynamic_navigation_evidence_valid(
            observation.dynamic_navigation
        ),
        "cmd_vel_observed": observation.cmd_vel_sample_count > 0,
        "robot_motion_observed": observation.nonzero_cmd_vel_sample_count > 0,
        "final_task_motion_observed": (
            observation.nonzero_cmd_vel_samples_after_boundary > 0
        ),
        # 初始化值也是 0；必须看到任务终态之后的新 Twist，才能证明控制链真的停住。
        "final_cmd_vel_fresh": final_cmd_vel_fresh,
        "final_cmd_vel_zero": final_cmd_vel_zero,
    }
    return {
        "schema_version": 4,
        "evidence_kind": "unknown_world_slam_nav_dynamic_replan",
        "passed": all(checks.values()),
        "session_id": observation.session_id,
        "session_start_ns": observation.session_start_ns,
        "mission_sequence": observation.mission_sequence,
        "mission_outcome": observation.mission_outcome,
        "mission_message": observation.mission_message,
        "final_phase": observation.final_phase,
        "mapping_path_m": round(observation.mapping_path_m, 3),
        "frontier_goal_count": observation.frontier_goal_count,
        "source_revision": observation.source_revision,
        "source_dirty": observation.source_dirty,
        "map_saved": observation.map_saved,
        "map_yaml_path": str(observation.built_map_yaml),
        "map_provenance": dict(provenance) if provenance else None,
        "map_quality": map_quality,
        "frontier": frontier,
        "approximate_completion": approximate_completion,
        "return_to_start": return_to_start,
        "localization": {
            **localization,
            "gazebo_truth_error": observation.gazebo_truth_error or None,
        },
        "sampled_navigation": navigation,
        "dynamic_navigation": (
            dict(observation.dynamic_navigation)
            if observation.dynamic_navigation
            else None
        ),
        "checks": checks,
        "final_cmd_vel": {
            "linear_x": observation.final_linear_x,
            "angular_z": observation.final_angular_z,
            "sample_count": observation.cmd_vel_sample_count,
            "nonzero_sample_count": observation.nonzero_cmd_vel_sample_count,
            "last_received_at_s": observation.last_cmd_vel_received_at_s,
            "final_stop_boundary_at_s": observation.final_stop_boundary_at_s,
            "samples_after_boundary": observation.cmd_vel_samples_after_boundary,
            "nonzero_samples_after_boundary": (
                observation.nonzero_cmd_vel_samples_after_boundary
            ),
        },
    }


def _region_ratios(
    *,
    truth: _OccupancyMap,
    built: _OccupancyMap,
    reachable: Iterable[tuple[int, int]],
    regions: Iterable[MapRegion],
) -> dict[str, float]:
    cells = tuple(reachable)
    ratios: dict[str, float] = {}
    for region in sorted(regions, key=lambda item: item.name):
        region_cells = [
            (row, column, x_m, y_m)
            for row, column in cells
            for x_m, y_m in (truth.cell_center_world(row, column),)
            if region.contains(x_m, y_m)
        ]
        covered = sum(
            built.state_at_world(x_m, y_m) == FREE
            for _row, _column, x_m, y_m in region_cells
        )
        ratios[region.name] = _ratio(covered, len(region_cells))
    return ratios


def evaluate_unknown_world_map(
    *,
    built_map_yaml: Path,
    truth_map_yaml: Path,
    robot_start_xy: tuple[float, float],
    regions: Iterable[MapRegion],
    thresholds: MapQualityThresholds,
    robot_clearance_m: float = 0.22,
) -> dict[str, object]:
    """比较本次 SLAM 地图与 evaluator-only 真值，生成可审计质量报告。"""

    built = _load_map(Path(built_map_yaml))
    truth = _load_map(Path(truth_map_yaml))
    reachable = _reachable_component(truth, robot_start_xy, robot_clearance_m)
    if not reachable:
        raise ValueError("truth map has no reachable clearance-safe free cells")

    states = [
        built.state_at_world(*truth.cell_center_world(row, column))
        for row, column in reachable
    ]
    covered_count = sum(state == FREE for state in states)
    unknown_count = sum(state == UNKNOWN for state in states)
    reachable_coverage = _ratio(covered_count, len(states))
    reachable_unknown = _ratio(unknown_count, len(states))
    region_ratios = _region_ratios(
        truth=truth,
        built=built,
        reachable=reachable,
        regions=tuple(regions),
    )
    obstacle_metrics = _obstacle_boundary_metrics(
        truth=truth,
        built=built,
        robot_start_xy=robot_start_xy,
    )
    checks = {
        "reachable_free_coverage": reachable_coverage
        >= thresholds.minimum_reachable_coverage_ratio,
        "all_regions_covered": bool(region_ratios)
        and min(region_ratios.values())
        >= thresholds.minimum_region_coverage_ratio,
        "reachable_unknown_ratio": reachable_unknown
        <= thresholds.maximum_reachable_unknown_ratio,
        "obstacle_boundary_recall": float(
            obstacle_metrics["obstacle_boundary_recall_ratio"]
        )
        >= thresholds.minimum_obstacle_boundary_recall_ratio,
        "obstacle_false_free_ratio": float(
            obstacle_metrics["obstacle_false_free_ratio"]
        )
        <= thresholds.maximum_obstacle_false_free_ratio,
    }
    return {
        "status": "COMPLETE" if all(checks.values()) else "INCOMPLETE",
        "passed": all(checks.values()),
        "checks": checks,
        "metrics": {
            "reachable_cell_count": len(states),
            "reachable_free_coverage_ratio": reachable_coverage,
            "reachable_unknown_ratio": reachable_unknown,
            "region_coverage_ratios": region_ratios,
            **obstacle_metrics,
        },
        "thresholds": {
            "minimum_reachable_coverage_ratio": (
                thresholds.minimum_reachable_coverage_ratio
            ),
            "minimum_region_coverage_ratio": (
                thresholds.minimum_region_coverage_ratio
            ),
            "maximum_reachable_unknown_ratio": (
                thresholds.maximum_reachable_unknown_ratio
            ),
            "minimum_obstacle_boundary_recall_ratio": (
                thresholds.minimum_obstacle_boundary_recall_ratio
            ),
            "maximum_obstacle_false_free_ratio": (
                thresholds.maximum_obstacle_false_free_ratio
            ),
            "robot_clearance_m": robot_clearance_m,
        },
        "maps": {
            "built": str(built.source),
            "truth": str(truth.source),
        },
    }
