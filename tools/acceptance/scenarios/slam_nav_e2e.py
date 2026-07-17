#!/usr/bin/env python3
"""Canonical frontier SLAM -> AMCL/Nav2 -> dynamic replan scenario."""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Mapping

from tools.acceptance.paths import repository_root
from tools.acceptance.session import (
    AcceptanceCommandError,
    AcceptanceSession,
    AcceptanceSessionConfig,
)


EVIDENCE_KIND = "gazebo_frontier_slam_map_saver_amcl_nav2_dynamic_replan"


def _positive_float(name: str, default: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError as error:
        raise ValueError(f"{name} must be a number") from error
    if value <= 0:
        raise ValueError(f"{name} must be positive")
    return value


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise ValueError(message)


def verify_report(
    report: Mapping[str, Any],
    *,
    expected_session_id: str,
    session_start_ns: int,
) -> dict[str, Any]:
    """验证一份 fresh-map 报告，并返回适合终端展示的稳定摘要。"""

    _require(report.get("passed") is True, "report did not pass")
    _require(report.get("schema_version") == 3, "unexpected report schema")
    _require(
        report.get("session_id") == expected_session_id,
        "report belongs to another acceptance session",
    )
    _require(report.get("automatic_mission") is True, "automatic mission missing")
    _require(report.get("map_saved") is True, "map was not saved")
    _require(report.get("final_phase") == 12, "mission did not complete")

    map_stats = report.get("map") or {}
    _require(map_stats.get("known_cells", 0) >= 6000, "known map area too small")
    _require(map_stats.get("occupied_cells", 0) >= 150, "map obstacles too sparse")
    _require(report.get("mapping_path_m", 0.0) >= 10.0, "mapping path too short")
    _require(report.get("frontier_goal_count", 0) >= 1, "no frontier goal observed")
    _require(
        report.get("exploration_completion_reason")
        in {"no_frontiers", "coverage_plateau", "time_budget_coverage"},
        "exploration completion reason is not auditable",
    )

    provenance = report.get("map_provenance") or {}
    _require(
        provenance.get("yaml_mtime_ns", 0) >= session_start_ns,
        "map yaml is not fresh for this session",
    )
    _require(
        provenance.get("image_mtime_ns", 0) >= session_start_ns,
        "map image is not fresh for this session",
    )
    dynamic = report.get("dynamic_navigation") or {}
    _require(dynamic.get("passed") is True, "dynamic navigation did not pass")
    checks = report.get("checks") or {}
    _require(bool(checks) and all(checks.values()), f"evidence checks failed: {checks}")

    follow_results = [
        item
        for item in report.get("action_results", [])
        if str(item.get("message", "")).startswith("nav2:follow_waypoints:")
    ]
    _require(bool(follow_results), "semantic waypoint result missing")
    _require(
        all("missed_waypoints=0" in item["message"] for item in follow_results),
        "semantic patrol missed at least one waypoint",
    )
    final_velocity = report.get("final_cmd_vel") or {}
    _require(
        abs(float(final_velocity.get("linear_x", 1.0))) < 1e-6
        and abs(float(final_velocity.get("angular_z", 1.0))) < 1e-6,
        "final cmd_vel is not zero",
    )
    return {
        "known_cells": int(map_stats["known_cells"]),
        "occupied_cells": int(map_stats["occupied_cells"]),
        "mapping_path_m": float(report["mapping_path_m"]),
        "frontier_goal_count": int(report["frontier_goal_count"]),
        "unique_dynamic_plans": int(dynamic["unique_plan_count"]),
    }


def main() -> int:
    workspace = repository_root(Path(__file__))
    gate_timeout_s = _positive_float("SLAM_NAV_GATE_TIMEOUT_S", 900.0)
    transition_timeout_s = _positive_float("SLAM_NAV_TRANSITION_TIMEOUT_S", 860.0)
    heartbeat_s = _positive_float("SLAM_NAV_PROGRESS_HEARTBEAT_S", 15.0)
    failure_lines = int(os.environ.get("SLAM_NAV_FAILURE_LOG_LINES", "120"))
    preferred_domain = (
        int(os.environ["SLAM_NAV_ROS_DOMAIN_ID"])
        if os.environ.get("SLAM_NAV_ROS_DOMAIN_ID")
        else None
    )
    artifact_dir = (
        Path(os.environ["SHOWCASE_SESSION_DIR"])
        if os.environ.get("SHOWCASE_SESSION_DIR")
        else None
    )
    session_environment = {
        # mock Agent 只固定语义输入；探索、SLAM、map_saver、AMCL、Nav2 与 Gazebo
        # 仍走真实运行时，从而把机器人自治证据与云服务波动分开。
        "NAV2_PROVIDER_MODE": "mock",
        "NAV2_MICROPHONE_ENABLED": "false",
        "NAV2_CAPTURE_ENABLED": "false",
        "WAKE_WORD_ENABLED": "false",
        "CONTINUOUS_PREFLIGHT_ENABLED": "false",
        "CONTINUOUS_MONITOR_ENABLED": "false",
        "CONTINUOUS_READINESS_ENABLED": "false",
        "VAD_PROVIDER": "energy",
        "HEADLESS": os.environ.get("HEADLESS", "true"),
        "USE_RVIZ": os.environ.get("USE_RVIZ", "false"),
        "SYSTEM_READINESS_TIMEOUT": "120",
        "SHOWCASE_DYNAMIC_OBSTACLE_ENABLED": "true",
    }
    config = AcceptanceSessionConfig(
        name="slam-nav-e2e",
        workspace=workspace,
        artifact_root=workspace / "logs" / "acceptance" / "slam_nav",
        artifact_dir=artifact_dir,
        session_id=os.environ.get("SLAM_NAV_SESSION_ID"),
        timeout_s=gate_timeout_s + 60.0,
        preferred_domain=preferred_domain,
        # canonical gate 不继承终端里通用 ROS_DOMAIN_ID，避免误接入旧 graph；
        # 需要复现固定 domain 时使用专用 SLAM_NAV_ROS_DOMAIN_ID。
        inherit_ros_domain_id=False,
        failure_log_lines=max(1, failure_lines),
        environment=session_environment,
    )

    with AcceptanceSession(config) as session:
        map_prefix = session.artifact_dir / "voice_built_map"
        report_path = session.artifact_dir / "slam_nav_e2e_report.json"
        runtime_log = session.log_path("runtime")
        mission_plan = (
            workspace
            / "src/embodied_simulation/config/showcase_workplace_mission.yaml"
        )
        world_file = (
            workspace
            / "src/embodied_simulation/worlds/showcase_apartment.sdf.xacro"
        )
        dynamic_scenario = (
            workspace
            / "src/embodied_navigation/config/showcase_dynamic_obstacle_scenario.json"
        )
        for stale in (
            map_prefix.with_suffix(".yaml"),
            map_prefix.with_suffix(".pgm"),
            report_path,
            runtime_log,
        ):
            stale.unlink(missing_ok=True)

        print("SLAM/Nav2 end-to-end acceptance")
        print(f"  session: {session.session_id}")
        print("  scene: showcase_apartment")
        print(
            "  stages: frontier SLAM -> fresh map -> AMCL/Nav2 -> "
            "semantic patrol -> dynamic replan"
        )
        print(f"  heartbeat: {heartbeat_s:g}s")
        print(f"  ROS domain: {session.domain_id}")
        print(f"  evidence: {report_path}")
        print(f"  session manifest: {session.manifest_path}")
        print(f"  runtime log: {runtime_log}")

        if session.run(
            ["ros2", "pkg", "prefix", "explore_lite"],
            timeout_s=10.0,
            check=False,
        ) != 0:
            raise AcceptanceCommandError(
                "Explore Lite is missing. Run: bash scripts/setup_frontier_exploration.sh"
            )

        session.spawn(
            "orchestrator",
            [
                "ros2",
                "run",
                "embodied_slam_tools",
                "voice_slam_session_orchestrator",
                "--ros-args",
                "-p",
                f"workspace:={workspace}",
                "-p",
                "mode:=offline",
                "-p",
                f"map_prefix:={map_prefix}",
                "-p",
                f"mission_plan:={mission_plan}",
                "-p",
                "startup_timeout_s:=150.0",
            ],
            log_path=runtime_log,
        )
        session.run(
            [
                "bash",
                str(workspace / "tools/acceptance/run_probe.sh"),
                str(
                    workspace
                    / "tools/acceptance/probes/slam_nav/session_orchestrator.py"
                ),
                "--output",
                str(report_path),
                "--transition-timeout",
                str(transition_timeout_s),
                "--gate-timeout-s",
                str(gate_timeout_s),
                "--progress-heartbeat-s",
                str(heartbeat_s),
                "--runtime-log",
                str(runtime_log),
                "--summary-only",
                "--evidence-kind",
                EVIDENCE_KIND,
                "--session-id",
                session.session_id,
                "--session-start-ns",
                str(session.started_ns),
                "--world-file",
                str(world_file),
                "--mission-plan",
                str(mission_plan),
                "--dynamic-scenario",
                str(dynamic_scenario),
                "--automatic-mission",
            ],
            timeout_s=gate_timeout_s,
        )

        map_yaml = map_prefix.with_suffix(".yaml")
        map_image = map_prefix.with_suffix(".pgm")
        if not map_yaml.is_file() or map_yaml.stat().st_size == 0:
            raise ValueError("saved map yaml is missing or empty")
        if not map_image.is_file() or map_image.stat().st_size == 0:
            raise ValueError("saved map image is missing or empty")
        report = json.loads(report_path.read_text(encoding="utf-8"))
        summary = verify_report(
            report,
            expected_session_id=session.session_id,
            session_start_ns=session.started_ns,
        )
        print("Evidence verified; cleaning up acceptance process groups ...")

    # PASS 必须晚于 __exit__：若进程树无法回收或 manifest 写入失败，绝不能先报成功。
    print(
        "Verified evidence: "
        f"map={summary['known_cells']}/{summary['occupied_cells']} cells, "
        f"mapping_path={summary['mapping_path_m']:.3f}m, "
        f"frontiers={summary['frontier_goal_count']}, "
        f"unique_dynamic_plans={summary['unique_dynamic_plans']}"
    )
    print(
        "PASS: one intent -> autonomous frontier SLAM -> saved map -> "
        "AMCL/Nav2 patrol"
    )
    print(f"Evidence: {report_path}")
    print(f"Session manifest: {session.manifest_path}")
    print(f"Runtime log: {runtime_log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
