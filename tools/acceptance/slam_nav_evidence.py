"""SLAM/Nav2 验收证据：纯几何、阈值判定和 JSON 友好报告。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence


Point2D = tuple[float, float]
Path2D = tuple[Point2D, ...]


def cumulative_distance(points: Sequence[Point2D]) -> float:
    """累计离散位姿里程；空序列和单点自然返回 0。"""

    return sum(
        math.hypot(current[0] - previous[0], current[1] - previous[1])
        for previous, current in zip(points, points[1:])
    )


def path_clearance(points: Sequence[Point2D], x: float, y: float) -> float:
    """保持现有门禁语义：计算目标点到离散规划采样点的最小距离。"""

    if not points:
        raise ValueError("path must contain at least one point")
    return min(math.hypot(px - x, py - y) for px, py in points)


def path_signature(points: Sequence[Point2D]) -> Path2D:
    """按厘米级稳定化规划路径，过滤浮点抖动后统计真正不同的 plan。"""

    return tuple((round(x, 2), round(y, 2)) for x, y in points)


@dataclass(frozen=True, slots=True)
class DynamicNavigationThresholds:
    minimum_track_confidence: float
    minimum_moving_velocity_mps: float
    minimum_predicted_cost: int
    minimum_clearance_gain_m: float
    minimum_unique_navigation_plans: int
    minimum_travel_distance_m: float

    @classmethod
    def from_mapping(
        cls, values: Mapping[str, float | int]
    ) -> "DynamicNavigationThresholds":
        """在 Adapter 边缘一次性把 JSON 字典收紧为验收值对象。"""

        return cls(
            minimum_track_confidence=float(
                values["minimum_track_confidence"]
            ),
            minimum_moving_velocity_mps=float(
                values["minimum_moving_velocity_mps"]
            ),
            minimum_predicted_cost=int(values["minimum_predicted_cost"]),
            minimum_clearance_gain_m=float(
                values["minimum_clearance_gain_m"]
            ),
            minimum_unique_navigation_plans=int(
                values["minimum_unique_navigation_plans"]
            ),
            minimum_travel_distance_m=float(
                values["minimum_travel_distance_m"]
            ),
        )


@dataclass(frozen=True, slots=True)
class DynamicNavigationObservation:
    """ROS/Gazebo Adapter 收集的一次动态导航事实快照。"""

    scenario_id: str
    goal: Mapping[str, object]
    route_attempts: tuple[Mapping[str, object], ...]
    motion_anchor: Mapping[str, float]
    gazebo_pose_updates: int
    expected_pose_updates: int
    track_id: str
    track_confidence: float
    track_velocity_x_mps: float
    track_velocity_y_mps: float
    predicted_x: float
    predicted_y: float
    predicted_cost: int
    baseline_path: Path2D
    dynamic_path: Path2D
    published_plans: tuple[Path2D, ...]
    odom_traveled_distance_m: float
    navigate_to_pose_status: int
    navigation_succeeded: bool
    final_linear_x: float
    final_angular_z: float


def evaluate_dynamic_navigation(
    observation: DynamicNavigationObservation,
    thresholds: DynamicNavigationThresholds,
) -> dict:
    """集中生成检查项和报告，避免不同探针各自定义“动态重规划成功”。"""

    baseline_clearance = path_clearance(
        observation.baseline_path,
        observation.predicted_x,
        observation.predicted_y,
    )
    dynamic_clearance = path_clearance(
        observation.dynamic_path,
        observation.predicted_x,
        observation.predicted_y,
    )
    unique_plans = {
        path_signature(plan) for plan in observation.published_plans if plan
    }
    checks = {
        "gazebo_entity_moved": observation.gazebo_pose_updates
        == observation.expected_pose_updates,
        "tracker_confident": observation.track_confidence
        >= thresholds.minimum_track_confidence,
        "tracker_estimated_motion": math.hypot(
            observation.track_velocity_x_mps,
            observation.track_velocity_y_mps,
        )
        >= thresholds.minimum_moving_velocity_mps,
        "future_cell_marked_lethal": observation.predicted_cost
        >= thresholds.minimum_predicted_cost,
        "dynamic_path_increased_clearance": dynamic_clearance
        >= baseline_clearance + thresholds.minimum_clearance_gain_m,
        "nav2_replanned": len(unique_plans)
        >= thresholds.minimum_unique_navigation_plans,
        "navigate_to_pose_succeeded": observation.navigation_succeeded,
        "robot_moved": observation.odom_traveled_distance_m
        >= thresholds.minimum_travel_distance_m,
        "cmd_vel_zero": abs(observation.final_linear_x) < 1e-3
        and abs(observation.final_angular_z) < 1e-3,
    }
    return {
        "passed": all(checks.values()),
        "checks": checks,
        "scenario_id": observation.scenario_id,
        "goal": dict(observation.goal),
        "route_attempts": [dict(item) for item in observation.route_attempts],
        "motion_anchor": {
            key: round(value, 4)
            for key, value in observation.motion_anchor.items()
        },
        "gazebo_pose_updates": observation.gazebo_pose_updates,
        "track": {
            "id": str(observation.track_id),
            "confidence": round(observation.track_confidence, 4),
            "velocity_x_mps": round(observation.track_velocity_x_mps, 4),
            "velocity_y_mps": round(observation.track_velocity_y_mps, 4),
        },
        "prediction": {
            "x": round(observation.predicted_x, 4),
            "y": round(observation.predicted_y, 4),
            "cost": observation.predicted_cost,
        },
        "baseline_clearance_m": round(baseline_clearance, 4),
        "dynamic_clearance_m": round(dynamic_clearance, 4),
        "published_plan_count": len(observation.published_plans),
        "unique_plan_count": len(unique_plans),
        "odom_traveled_distance_m": round(
            observation.odom_traveled_distance_m, 3
        ),
        "navigate_to_pose_status": observation.navigate_to_pose_status,
    }


@dataclass(frozen=True, slots=True)
class AutomaticMissionThresholds:
    minimum_mapping_path_m: float
    minimum_known_map_cells: int
    minimum_occupied_map_cells: int

    @classmethod
    def from_mapping(
        cls, values: Mapping[str, float | int]
    ) -> "AutomaticMissionThresholds":
        return cls(
            minimum_mapping_path_m=float(
                values.get("min_mapping_path_m", 0.0)
            ),
            minimum_known_map_cells=int(
                values.get("min_known_map_cells", 0)
            ),
            minimum_occupied_map_cells=int(
                values.get("min_occupied_map_cells", 0)
            ),
        )


@dataclass(frozen=True, slots=True)
class AutomaticMissionObservation:
    """一次自动建图导航会话的跨层事实，不包含 ROS 消息对象。"""

    session_id: str
    session_start_ns: int
    state_sequence: tuple[int, ...]
    map_saved: bool
    map_yaml_path: str
    final_phase: int
    evidence_kind: str
    action_candidates: tuple[Mapping[str, object], ...]
    action_results: tuple[Mapping[str, object], ...]
    map_stats: Mapping[str, int] | None
    map_provenance: Mapping[str, object] | None
    mapping_path_m: float
    frontier_goal_count: int
    exploration_completion_reason: str
    localization_tf_count: int
    amcl_pose_count: int
    lifecycle_states: Mapping[str, int]
    nav2_lifecycle_active: bool
    dynamic_navigation: Mapping[str, object] | None
    dynamic_navigation_required: bool
    provenance: Mapping[str, str]
    final_linear_x: float
    final_angular_z: float


def _semantic_navigation_succeeded(
    candidates: Sequence[Mapping[str, object]],
    results: Sequence[Mapping[str, object]],
) -> bool:
    """同时验证目标导航、完整多航点结果和 command_id 关联。"""

    result_by_id = {
        str(item.get("command_id", "")): item
        for item in results
        if item.get("command_id")
    }
    navigate_ids = {
        str(item.get("request_id", ""))
        for item in candidates
        if item.get("name") == "navigate_to" and item.get("request_id")
    }
    follow_ids = {
        str(item.get("request_id", ""))
        for item in candidates
        if item.get("name") == "follow_waypoints" and item.get("request_id")
    }
    all_ids = navigate_ids | follow_ids
    if not navigate_ids or not follow_ids:
        return False
    if not all(
        result_by_id.get(command_id, {}).get("success") is True
        for command_id in all_ids
    ):
        return False
    # Nav2 Action 的协议 SUCCEEDED 可能仍包含 missed waypoint；业务验收必须为零。
    return all(
        "missed_waypoints=0"
        in str(result_by_id.get(command_id, {}).get("message", ""))
        for command_id in follow_ids
    )


def build_automatic_mission_report(
    observation: AutomaticMissionObservation,
    thresholds: AutomaticMissionThresholds,
) -> dict:
    """从跨层事实生成唯一的 E2E PASS 判定与 schema v3 报告。"""

    runtime_required = observation.evidence_kind != "dry_run_process_adapter"
    map_stats = observation.map_stats or {}
    provenance = observation.map_provenance
    fresh_map = bool(
        provenance
        and observation.session_start_ns
        and int(provenance.get("yaml_mtime_ns", 0))
        >= observation.session_start_ns
        and int(provenance.get("image_mtime_ns", 0))
        >= observation.session_start_ns
    )
    semantic_success = _semantic_navigation_succeeded(
        observation.action_candidates,
        observation.action_results,
    )
    checks = {
        "map_saved": observation.map_saved,
        "fresh_session_map": fresh_map if runtime_required else True,
        "frontier_goal_observed": observation.frontier_goal_count >= 1
        if runtime_required
        else True,
        "mapping_path_threshold": observation.mapping_path_m
        >= thresholds.minimum_mapping_path_m,
        "known_cells_threshold": int(map_stats.get("known_cells", 0))
        >= thresholds.minimum_known_map_cells
        if runtime_required
        else True,
        "occupied_cells_threshold": int(map_stats.get("occupied_cells", 0))
        >= thresholds.minimum_occupied_map_cells
        if runtime_required
        else True,
        "auditable_exploration_completion": (
            observation.exploration_completion_reason
            in {"no_frontiers", "coverage_plateau", "time_budget_coverage"}
        )
        if runtime_required
        else True,
        "localization_tf": observation.localization_tf_count > 0
        if runtime_required
        else True,
        "amcl_pose": observation.amcl_pose_count > 0
        if runtime_required
        else True,
        "nav2_lifecycle_active": observation.nav2_lifecycle_active
        if runtime_required
        else True,
        "semantic_navigation_succeeded": semantic_success
        if runtime_required
        else True,
        "dynamic_navigation_succeeded": bool(
            observation.dynamic_navigation
            and observation.dynamic_navigation.get("passed") is True
        )
        if observation.dynamic_navigation_required
        else True,
        "final_cmd_vel_zero": abs(observation.final_linear_x) < 1e-3
        and abs(observation.final_angular_z) < 1e-3,
    }
    return {
        "schema_version": 3,
        "passed": all(checks.values()),
        "session_id": observation.session_id,
        "session_start_ns": observation.session_start_ns,
        "state_sequence": list(observation.state_sequence),
        "map_saved": observation.map_saved,
        "map_yaml_path": observation.map_yaml_path,
        "final_phase": observation.final_phase,
        "evidence_kind": observation.evidence_kind,
        "automatic_mission": True,
        "action_candidates": [
            dict(item) for item in observation.action_candidates
        ],
        "action_results": [dict(item) for item in observation.action_results],
        "map": dict(observation.map_stats) if observation.map_stats else None,
        "map_provenance": dict(provenance) if provenance else None,
        "mapping_path_m": round(observation.mapping_path_m, 3),
        "frontier_goal_count": observation.frontier_goal_count,
        "exploration_completion_reason": (
            observation.exploration_completion_reason
        ),
        "localization": {
            "map_to_odom_count": observation.localization_tf_count,
            "amcl_pose_count": observation.amcl_pose_count,
            "lifecycle_states": dict(observation.lifecycle_states),
        },
        "dynamic_navigation": (
            dict(observation.dynamic_navigation)
            if observation.dynamic_navigation
            else None
        ),
        "provenance": dict(observation.provenance),
        "checks": checks,
        "final_cmd_vel": {
            "linear_x": observation.final_linear_x,
            "angular_z": observation.final_angular_z,
        },
    }
