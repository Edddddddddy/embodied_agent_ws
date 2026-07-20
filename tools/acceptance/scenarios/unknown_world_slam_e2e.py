#!/usr/bin/env python3
"""未知场地自主探索、SLAM、AMCL/Nav2 与动态避障完整门禁。"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml

from tools.acceptance.paths import repository_root
from tools.acceptance.scenarios.unknown_world_contract import (
    UNKNOWN_WORLD_CLEARED_ENVIRONMENT_KEYS,
    audit_unknown_world_policy_spawn,
    build_unknown_world_runtime_environment,
    build_unknown_world_timeout_budget,
    validate_unknown_world_mission,
)
from tools.acceptance.session import (
    AcceptanceCommandError,
    AcceptanceSession,
    AcceptanceSessionConfig,
)


EVIDENCE_KIND = "unknown_world_slam_nav_dynamic_replan"
MAPPING_STARTUP_TIMEOUT_S = 150.0
SCAN_STARTUP_TIMEOUT_S = 20.0
STAGE_STOP_TIMEOUT_S = 15.0
MAP_SAVE_TIMEOUT_S = 35.0
DYNAMIC_NAVIGATION_TIMEOUT_S = 180.0


def _positive_float(name: str, default: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError as error:
        raise ValueError(f"{name} must be a number") from error
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


def verify_report(
    report: Mapping[str, Any],
    *,
    expected_session_id: str,
) -> dict[str, float | int]:
    """二次校验 schema v4，避免探针写出不属于本会话的陈旧 PASS。"""

    if report.get("schema_version") != 4 or report.get("passed") is not True:
        raise ValueError("unknown-world report did not pass schema v4")
    if report.get("session_id") != expected_session_id:
        raise ValueError("unknown-world report belongs to another session")
    checks = report.get("checks") or {}
    if not checks or not all(checks.values()):
        raise ValueError(f"unknown-world evidence checks failed: {checks}")
    coverage = report["map_quality"]["metrics"]
    localization = report["localization"]["metrics"]
    navigation = report["sampled_navigation"]["metrics"]
    return {
        "coverage_ratio": float(coverage["reachable_free_coverage_ratio"]),
        "minimum_region_ratio": float(
            min(coverage["region_coverage_ratios"].values())
        ),
        "localization_p95_m": float(localization["position_error_p95_m"]),
        "navigation_goal_count": int(navigation["goal_count"]),
    }


def _try_save_failed_exploration_map(
    session: AcceptanceSession,
    map_prefix: Path,
) -> bool:
    """在失败清理仿真前尽力保留当前 /map，且不改变正式验收状态。"""

    yaml_path = map_prefix.with_suffix(".yaml")
    image_path = map_prefix.with_suffix(".pgm")
    try:
        returncode = session.run(
            [
                "ros2",
                "run",
                "nav2_map_server",
                "map_saver_cli",
                "-f",
                str(map_prefix),
                "--ros-args",
                "-p",
                "save_map_timeout:=30.0",
                "-p",
                "free_thresh_default:=0.25",
                "-p",
                "occupied_thresh_default:=0.65",
            ],
            timeout_s=MAP_SAVE_TIMEOUT_S,
            check=False,
        )
        complete = returncode == 0 and all(
            path.is_file() and path.stat().st_size > 0
            for path in (yaml_path, image_path)
        )
    except Exception as error:
        print(
            "WARN: failed to preserve diagnostic exploration map: "
            f"{error}",
            file=sys.stderr,
        )
        return False
    if not complete:
        print(
            "WARN: diagnostic exploration map is incomplete "
            f"(returncode={returncode}, prefix={map_prefix})",
            file=sys.stderr,
        )
        return False
    print(f"Diagnostic exploration map: {map_prefix}.{{yaml,pgm}}")
    return True


def _run_probe_and_verify(
    session: AcceptanceSession,
    *,
    probe_command: Sequence[str],
    report_path: Path,
    expected_session_id: str,
    gate_timeout_s: float,
    failed_map_prefix: Path,
) -> dict[str, float | int]:
    """运行探针并验证报告；失败快照只用于诊断，不代表 map_saved。"""

    try:
        session.run(probe_command, timeout_s=gate_timeout_s)
        report = json.loads(report_path.read_text(encoding="utf-8"))
        return verify_report(report, expected_session_id=expected_session_id)
    except Exception:
        # 此处仍在 AcceptanceSession 内，仿真和 SLAM /map 尚未被 finally 清理。
        try:
            _try_save_failed_exploration_map(session, failed_map_prefix)
        except BaseException:
            # 诊断路径连输出流失败都不能替换正在传播的验收异常。
            pass
        raise


def main() -> int:
    workspace = repository_root(Path(__file__))
    heartbeat_s = _positive_float("SLAM_NAV_PROGRESS_HEARTBEAT_S", 15.0)
    preferred_domain = (
        int(os.environ["SLAM_NAV_ROS_DOMAIN_ID"])
        if os.environ.get("SLAM_NAV_ROS_DOMAIN_ID")
        else None
    )
    artifact_dir = (
        Path(os.environ["UNKNOWN_WORLD_ARTIFACT_DIR"])
        if os.environ.get("UNKNOWN_WORLD_ARTIFACT_DIR")
        else None
    )
    mission_plan = (
        workspace / "src/embodied_simulation/config/unknown_world_slam_mission.yaml"
    )
    mission_document = yaml.safe_load(mission_plan.read_text(encoding="utf-8"))
    validate_unknown_world_mission(mission_document)
    world_file = (
        workspace / "src/embodied_simulation/worlds/showcase_apartment.sdf.xacro"
    )
    scene_spec = workspace / "src/embodied_simulation/config/showcase_apartment.yaml"
    scene_document = yaml.safe_load(scene_spec.read_text(encoding="utf-8"))
    truth_map = (
        workspace
        / "src/embodied_simulation/maps/showcase_apartment_slam_frame.yaml"
    )
    dynamic_scenario = (
        workspace
        / "src/embodied_navigation/config/showcase_dynamic_obstacle_scenario.json"
    )
    budget = build_unknown_world_timeout_budget(
        mission_document,
        mapping_startup_s=MAPPING_STARTUP_TIMEOUT_S,
        scan_startup_s=SCAN_STARTUP_TIMEOUT_S,
        stage_stop_s=STAGE_STOP_TIMEOUT_S,
        map_save_s=MAP_SAVE_TIMEOUT_S,
        dynamic_navigation_s=DYNAMIC_NAVIGATION_TIMEOUT_S,
    )
    transition_timeout_s = _positive_float(
        "UNKNOWN_WORLD_TRANSITION_TIMEOUT_S",
        budget.mission_transition_s,
    )
    gate_timeout_s = _positive_float(
        "UNKNOWN_WORLD_GATE_TIMEOUT_S",
        budget.gate_s,
    )
    budget.validate_outer_timeouts(
        transition_timeout_s=transition_timeout_s,
        gate_timeout_s=gate_timeout_s,
    )
    runtime_environment = build_unknown_world_runtime_environment(
        world_path=world_file,
        spawn=scene_document["world"]["spawn"],
        headless=os.environ.get("HEADLESS", "true"),
        use_rviz=os.environ.get("USE_RVIZ", "false"),
        budget=budget,
        gate_timeout_s=gate_timeout_s,
        transition_timeout_s=transition_timeout_s,
    )
    runtime_environment.update(
        {
            "NAV2_PROVIDER_MODE": "mock",
            "NAV2_MICROPHONE_ENABLED": "false",
            "NAV2_CAPTURE_ENABLED": "false",
            "WAKE_WORD_ENABLED": "false",
            "CONTINUOUS_PREFLIGHT_ENABLED": "false",
            "CONTINUOUS_MONITOR_ENABLED": "false",
            "CONTINUOUS_READINESS_ENABLED": "false",
            "VAD_PROVIDER": "energy",
            "SYSTEM_READINESS_TIMEOUT": "120",
            "SHOWCASE_DYNAMIC_OBSTACLE_ENABLED": "true",
        }
    )
    config = AcceptanceSessionConfig(
        name="unknown-world-slam-e2e",
        workspace=workspace,
        artifact_root=workspace / "logs/acceptance/unknown_world_slam_nav",
        artifact_dir=artifact_dir,
        session_id=os.environ.get("SLAM_NAV_SESSION_ID"),
        timeout_s=gate_timeout_s + 60.0,
        preferred_domain=preferred_domain,
        inherit_ros_domain_id=False,
        environment=runtime_environment,
        unset_environment_keys=UNKNOWN_WORLD_CLEARED_ENVIRONMENT_KEYS,
        manifest_environment_keys=tuple(runtime_environment),
    )

    with AcceptanceSession(config) as session:
        map_prefix = session.artifact_dir / "unknown_world_map"
        failed_map_prefix = session.artifact_dir / "failed_exploration_map"
        report_path = session.artifact_dir / "unknown_world_slam_e2e_report.json"
        runtime_log = session.log_path("runtime")
        for stale in (
            map_prefix.with_suffix(".yaml"),
            map_prefix.with_suffix(".pgm"),
            failed_map_prefix.with_suffix(".yaml"),
            failed_map_prefix.with_suffix(".pgm"),
            report_path,
            runtime_log,
        ):
            stale.unlink(missing_ok=True)

        print("Unknown-world SLAM/Nav2 end-to-end acceptance")
        print(f"  session: {session.session_id}")
        print("  robot policy: online scan/odom/TF/map only; no truth map or route")
        print(
            "  stages: frontier exploration -> fresh map -> AMCL/Gazebo P95 -> "
            "3 sampled goals -> dynamic replan"
        )
        print(f"  ROS domain: {session.domain_id}")
        print(f"  Gazebo partition: {session.environment['GZ_PARTITION']}")
        print(
            "  timeout budget: "
            f"mission={budget.mission_transition_s:.0f}s "
            f"gate={budget.gate_s:.0f}s"
        )
        print(f"  evidence: {report_path}")

        if session.run(
            ["ros2", "pkg", "prefix", "explore_lite"],
            timeout_s=10.0,
            check=False,
        ) != 0:
            raise AcceptanceCommandError(
                "Explore Lite is missing. Run: bash scripts/setup_frontier_exploration.sh"
            )

        orchestrator_command = [
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
            f"startup_timeout_s:={MAPPING_STARTUP_TIMEOUT_S}",
            "-p",
            f"scan_startup_timeout_s:={SCAN_STARTUP_TIMEOUT_S}",
            "-p",
            f"stop_timeout_s:={STAGE_STOP_TIMEOUT_S}",
        ]
        # 审计最终的 session.environment，而不是另造一份“看起来安全”的测试
        # argv/env。这样宿主 shell 遗留的地图或路线变量也会在真正 spawn 前失败。
        audit_unknown_world_policy_spawn(
            argv=orchestrator_command,
            environment=session.environment,
            world_path=world_file,
            scene_spec_path=scene_spec,
            truth_map_path=truth_map,
        )
        session.spawn("orchestrator", orchestrator_command, log_path=runtime_log)
        probe_command = [
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
            "--dynamic-navigation-timeout",
            str(DYNAMIC_NAVIGATION_TIMEOUT_S),
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
            "--scene-spec",
            str(scene_spec),
            "--truth-map",
            str(truth_map),
            "--unknown-world",
            "--automatic-mission",
        ]
        summary = _run_probe_and_verify(
            session,
            probe_command=probe_command,
            report_path=report_path,
            expected_session_id=session.session_id,
            gate_timeout_s=gate_timeout_s,
            failed_map_prefix=failed_map_prefix,
        )
        print("Evidence verified; cleaning up acceptance process groups ...")

    print(
        "Verified evidence: "
        f"coverage={summary['coverage_ratio']:.3f}, "
        f"min_region={summary['minimum_region_ratio']:.3f}, "
        f"AMCL_P95={summary['localization_p95_m']:.3f}m, "
        f"goals={summary['navigation_goal_count']}"
    )
    print(
        "PASS: unknown-world frontier SLAM -> saved-map localization -> "
        "sampled Nav2 goals -> dynamic obstacle replan"
    )
    print(f"Evidence: {report_path}")
    print(f"Session manifest: {session.manifest_path}")
    print(f"Runtime log: {runtime_log}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
