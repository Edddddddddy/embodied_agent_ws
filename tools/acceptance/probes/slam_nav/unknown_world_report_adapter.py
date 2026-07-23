"""把 ROS 会话快照适配为纯 evaluator 的 unknown-world 报告。"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Mapping

from embodied_agent_interfaces.msg import (
    SlamNavigationGoalEvidence,
    SlamSessionState,
)

from tools.acceptance.unknown_world_evidence import (
    MapQualityThresholds,
    NavigationGoalObservation,
    SceneEvaluationContext,
    UnknownWorldObservation,
    UnknownWorldThresholds,
    build_unknown_world_report,
)


def _path_points(path: Any) -> tuple[tuple[float, float], ...]:
    return tuple(
        (float(item.pose.position.x), float(item.pose.position.y))
        for item in path.poses
    )


def build_session_report(
    *,
    node: Any,
    final_state: SlamSessionState,
    session_id: str,
    session_start_ns: int,
    built_map_yaml: Path,
    truth_map_yaml: Path,
    scene_context: SceneEvaluationContext,
    map_provenance: Mapping[str, object] | None,
    nav2_lifecycle_active: bool,
    dynamic_navigation: Mapping[str, object] | None,
    mapping_path_m: float,
) -> dict[str, object]:
    """在一个边界内完成 ROS snapshot → evaluator domain 的转换。"""

    source_dirty_value = os.environ.get("ACCEPTANCE_SOURCE_DIRTY")
    source_dirty = (
        True
        if source_dirty_value == "true"
        else False
        if source_dirty_value == "false"
        else None
    )
    amcl_samples, gazebo_samples, gazebo_truth_error = (
        node.localization_evidence()
    )
    goal_evidence = node.sampled_goal_evidence_snapshot()
    motion_evidence = node.motion_evidence()
    navigation_goals = tuple(
        NavigationGoalObservation(
            x_m=float(snapshot["x"]),
            y_m=float(snapshot["y"]),
            succeeded=(
                int(snapshot["status"])
                == SlamNavigationGoalEvidence.STATUS_SUCCEEDED
            ),
            planned_paths=tuple(
                _path_points(path)
                for path in goal_evidence.plans.get(sequence, ())
            ),
            producer_evidence=snapshot,
        )
        for sequence, snapshot in sorted(goal_evidence.goals.items())
    )
    final_mission_sequence = int(final_state.mission_sequence)
    goal_evidence_error = goal_evidence.evidence_error
    if goal_evidence.mission_sequence != final_mission_sequence:
        goal_evidence_error = (
            f"sampled-goal mission mismatch: {goal_evidence.mission_sequence} "
            f"!= {final_mission_sequence}"
        )
    # Gazebo truth 与场景区域只在这个 evaluator Adapter 中出现；把构造过程
    # 从主编排器抽离，可避免未来新增指标时反向污染机器人运行时策略。
    observation = UnknownWorldObservation(
        session_id=session_id,
        session_start_ns=session_start_ns,
        mission_profile_unknown=(
            node.mission_profile == SlamSessionState.PROFILE_UNKNOWN_WORLD
        ),
        mission_sequence=final_mission_sequence,
        mission_completed=(
            int(final_state.phase) == SlamSessionState.MISSION_COMPLETED
        ),
        mission_outcome=int(final_state.mission_outcome),
        mission_message=str(final_state.mission_message),
        map_saved=bool(final_state.map_saved),
        built_map_yaml=built_map_yaml,
        truth_map_yaml=truth_map_yaml,
        robot_start_xy=(0.0, 0.0),
        regions=scene_context.regions,
        map_provenance=map_provenance,
        frontier_telemetry=node.frontier_evidence,
        mapping_completion_evidence=node.mapping_completion_evidence,
        navigation_goals=navigation_goals,
        amcl_samples=amcl_samples,
        gazebo_samples=gazebo_samples,
        gazebo_truth_error=(
            gazebo_truth_error or goal_evidence_error
        ),
        nav2_lifecycle_active=nav2_lifecycle_active,
        dynamic_navigation=dynamic_navigation,
        final_linear_x=float(motion_evidence["linear_x"]),
        final_angular_z=float(motion_evidence["angular_z"]),
        cmd_vel_sample_count=int(motion_evidence["sample_count"]),
        nonzero_cmd_vel_sample_count=int(
            motion_evidence["nonzero_sample_count"]
        ),
        last_cmd_vel_received_at_s=float(
            motion_evidence["last_received_at_s"]
        ),
        final_stop_boundary_at_s=float(
            motion_evidence["final_stop_boundary_at_s"]
        ),
        cmd_vel_samples_after_boundary=int(
            motion_evidence["samples_after_boundary"]
        ),
        nonzero_cmd_vel_samples_after_boundary=int(
            motion_evidence["nonzero_samples_after_boundary"]
        ),
        final_phase=int(final_state.phase),
        mapping_path_m=mapping_path_m,
        frontier_goal_count=len(node.frontier_goal_ids),
        source_revision=os.environ.get(
            "ACCEPTANCE_SOURCE_REVISION", ""
        ),
        source_dirty=source_dirty,
    )
    return build_unknown_world_report(
        observation,
        UnknownWorldThresholds(
            map_quality=MapQualityThresholds(
                minimum_reachable_coverage_ratio=0.90,
                minimum_region_coverage_ratio=0.85,
                maximum_reachable_unknown_ratio=0.10,
            )
        ),
    )
