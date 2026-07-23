#!/usr/bin/env python3
"""未知场地自主探索、SLAM、AMCL/Nav2 与动态避障完整门禁。"""

from __future__ import annotations

import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Sequence

import yaml

from tools.acceptance.paths import repository_root
from tools.acceptance.scenarios.unknown_world_contract import (
    UNKNOWN_WORLD_CLEARED_ENVIRONMENT_KEYS,
    audit_unknown_world_policy_spawn,
    build_unknown_world_runtime_environment,
    build_unknown_world_timeout_budget,
    validate_unknown_world_mission,
)
from tools.acceptance.scenarios.unknown_world_run_profile import (
    DEFAULT_LIVE_VOICE_TRIGGER_TIMEOUT_S,
    EVIDENCE_KIND,
    LIVE_VOICE_EVIDENCE_KIND,
    MAPPING_STARTUP_TIMEOUT_S,
    UnknownWorldRunProfile,
    _resolve_profile_runtime_environment,
    _verify_profile_report,
    verify_report,
)
from tools.acceptance.session import (
    AcceptanceCommandError,
    AcceptanceSession,
    AcceptanceSessionConfig,
    RosEnvironmentIsolation,
)


SCAN_STARTUP_TIMEOUT_S = 20.0
STAGE_STOP_TIMEOUT_S = 15.0
MAP_SAVE_TIMEOUT_S = 35.0
DYNAMIC_NAVIGATION_TIMEOUT_S = 180.0


def _source_revision_environment(workspace: Path) -> dict[str, str]:
    """在启动前绑定源码版本；证据必须能回答“究竟验收了哪份代码”。

    dirty 只作为发布审计字段，不参与算法 PASS。这样开发者仍可验证尚未提交的
    修复，但发布流程可以明确拒绝把 dirty run 当成 main 的正式证据。
    """

    revision = subprocess.run(
        ["git", "-C", str(workspace), "rev-parse", "HEAD"],
        check=True,
        capture_output=True,
        text=True,
        timeout=5.0,
    ).stdout.strip()
    status = subprocess.run(
        ["git", "-C", str(workspace), "status", "--porcelain"],
        check=True,
        capture_output=True,
        text=True,
        timeout=5.0,
    ).stdout
    return {
        "ACCEPTANCE_SOURCE_REVISION": revision,
        "ACCEPTANCE_SOURCE_DIRTY": "true" if status.strip() else "false",
    }


def _positive_float(name: str, default: float) -> float:
    try:
        value = float(os.environ.get(name, str(default)))
    except ValueError as error:
        raise ValueError(f"{name} must be a number") from error
    if value <= 0.0:
        raise ValueError(f"{name} must be positive")
    return value


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
    profile: UnknownWorldRunProfile | None = None,
) -> dict[str, float | int]:
    """运行探针并验证报告；失败快照只用于诊断，不代表 map_saved。"""

    try:
        session.run(probe_command, timeout_s=gate_timeout_s)
        report = json.loads(report_path.read_text(encoding="utf-8"))
        return _verify_profile_report(
            report,
            profile=profile or UnknownWorldRunProfile.synthetic(),
            expected_session_id=expected_session_id,
        )
    except Exception:
        # 此处仍在 AcceptanceSession 内，仿真和 SLAM /map 尚未被 finally 清理。
        try:
            _try_save_failed_exploration_map(session, failed_map_prefix)
        except BaseException:
            # 诊断路径连输出流失败都不能替换正在传播的验收异常。
            pass
        raise


def _build_orchestrator_command(
    *,
    profile: UnknownWorldRunProfile,
    workspace: Path,
    map_prefix: Path,
    mission_plan: Path,
) -> list[str]:
    """构造真实启动命令，Agent 模式必须来自本次验收 profile。"""

    command = [
        "ros2",
        "run",
        "embodied_slam_tools",
        "voice_slam_session_orchestrator",
        "--ros-args",
        "-p",
        f"workspace:={workspace}",
        "-p",
        f"mode:={profile.agent_mode}",
        "-p",
        "command_input_source:="
        + ("wake_event" if profile.trigger_source == "live_voice" else "raw_asr"),
        "-p",
        f"map_prefix:={map_prefix}",
        "-p",
        f"mission_plan:={mission_plan}",
        "-p",
        f"startup_timeout_s:={profile.mapping_startup_timeout_s}",
        "-p",
        f"scan_startup_timeout_s:={SCAN_STARTUP_TIMEOUT_S}",
        "-p",
        f"stop_timeout_s:={STAGE_STOP_TIMEOUT_S}",
    ]
    return command


def _build_probe_command(
    *,
    profile: UnknownWorldRunProfile,
    workspace: Path,
    report_path: Path,
    transition_timeout_s: float,
    gate_timeout_s: float,
    heartbeat_s: float,
    runtime_log: Path,
    session_id: str,
    session_started_ns: int,
    world_file: Path,
    mission_plan: Path,
    dynamic_scenario: Path,
    scene_spec: Path,
    truth_map: Path,
    voice_trigger_timeout_s: float,
) -> list[str]:
    """构造联合探针 argv；显式传递触发来源，禁止探针自行猜测。"""

    evidence_kind = (
        LIVE_VOICE_EVIDENCE_KIND
        if profile.trigger_source == "live_voice"
        else EVIDENCE_KIND
    )
    total_gate_timeout_s = gate_timeout_s + (
        voice_trigger_timeout_s
        if profile.trigger_source == "live_voice"
        else 0.0
    )
    command = [
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
        str(total_gate_timeout_s),
        "--dynamic-navigation-timeout",
        str(DYNAMIC_NAVIGATION_TIMEOUT_S),
        "--progress-heartbeat-s",
        str(heartbeat_s),
        "--runtime-log",
        str(runtime_log),
        "--summary-only",
        "--evidence-kind",
        evidence_kind,
        "--session-id",
        session_id,
        "--session-start-ns",
        str(session_started_ns),
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
    if profile.trigger_source == "live_voice":
        # 合成门禁保持原 argv 不变；只有联合验收需要启用新增探针接口。
        command.extend(
            [
                "--automatic-trigger-source",
                profile.trigger_source,
                "--agent-mode",
                profile.agent_mode,
                "--voice-trigger-timeout",
                str(voice_trigger_timeout_s),
            ]
        )
    return command


def run(profile: UnknownWorldRunProfile) -> int:
    """运行 strict unknown-world 门禁，差异仅由显式 profile 注入。"""

    workspace = repository_root(Path(__file__))
    heartbeat_s = _positive_float("SLAM_NAV_PROGRESS_HEARTBEAT_S", 15.0)
    preferred_domain = (
        int(os.environ["SLAM_NAV_ROS_DOMAIN_ID"])
        if os.environ.get("SLAM_NAV_ROS_DOMAIN_ID")
        else None
    )
    artifact_environment_key = (
        "VOICE_UNKNOWN_WORLD_ARTIFACT_DIR"
        if profile.trigger_source == "live_voice"
        else "UNKNOWN_WORLD_ARTIFACT_DIR"
    )
    artifact_dir = (
        Path(os.environ[artifact_environment_key])
        if os.environ.get(artifact_environment_key)
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
        mapping_startup_s=profile.mapping_startup_timeout_s,
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
    voice_trigger_timeout_s = (
        _positive_float(
            "VOICE_TRIGGER_TIMEOUT_S",
            DEFAULT_LIVE_VOICE_TRIGGER_TIMEOUT_S,
        )
        if profile.trigger_source == "live_voice"
        else 0.0
    )
    # 真人说出口令的等待窗口是交互成本，不属于探索/定位/导航预算；外层
    # deadline 必须显式叠加，否则用户稍晚开口会压缩 strict core 的执行时间。
    probe_timeout_s = gate_timeout_s + voice_trigger_timeout_s
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
    runtime_environment["SHOWCASE_DYNAMIC_OBSTACLE_ENABLED"] = "true"
    runtime_environment.update(_resolve_profile_runtime_environment(profile))
    runtime_environment.update(_source_revision_environment(workspace))
    config = AcceptanceSessionConfig(
        name=(
            "voice-unknown-world-slam-e2e"
            if profile.trigger_source == "live_voice"
            else "unknown-world-slam-e2e"
        ),
        workspace=workspace,
        artifact_root=(
            workspace / "logs/acceptance" / profile.artifact_directory_name
        ),
        artifact_dir=artifact_dir,
        session_id=os.environ.get("SLAM_NAV_SESSION_ID"),
        timeout_s=probe_timeout_s + 60.0,
        preferred_domain=preferred_domain,
        inherit_ros_domain_id=False,
        environment=runtime_environment,
        unset_environment_keys=UNKNOWN_WORLD_CLEARED_ENVIRONMENT_KEYS,
        manifest_environment_keys=tuple(runtime_environment),
        # 公开 strict 与真人语音入口必须复用同一套受控 ROS/Nav2 来源；宿主
        # 终端 source 过的 ~/nav2_ws 不能改变现场验收所运行的二进制。
        ros_environment_isolation=RosEnvironmentIsolation(),
    )

    with AcceptanceSession(config) as session:
        map_prefix = session.artifact_dir / "unknown_world_map"
        failed_map_prefix = session.artifact_dir / "failed_exploration_map"
        report_path = session.artifact_dir / profile.report_filename
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
        print(
            f"  trigger: {profile.trigger_source}; "
            f"agent: {profile.agent_mode}"
        )
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

        orchestrator_command = _build_orchestrator_command(
            profile=profile,
            workspace=workspace,
            map_prefix=map_prefix,
            mission_plan=mission_plan,
        )
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
        probe_command = _build_probe_command(
            profile=profile,
            workspace=workspace,
            report_path=report_path,
            transition_timeout_s=transition_timeout_s,
            gate_timeout_s=gate_timeout_s,
            heartbeat_s=heartbeat_s,
            runtime_log=runtime_log,
            session_id=session.session_id,
            session_started_ns=session.started_ns,
            world_file=world_file,
            mission_plan=mission_plan,
            dynamic_scenario=dynamic_scenario,
            scene_spec=scene_spec,
            truth_map=truth_map,
            voice_trigger_timeout_s=voice_trigger_timeout_s,
        )
        summary = _run_probe_and_verify(
            session,
            probe_command=probe_command,
            report_path=report_path,
            expected_session_id=session.session_id,
            gate_timeout_s=probe_timeout_s,
            failed_map_prefix=failed_map_prefix,
            profile=profile,
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


def main() -> int:
    """兼容原公开入口，继续执行确定性的无麦克风 strict 门禁。"""

    return run(UnknownWorldRunProfile.synthetic())


if __name__ == "__main__":
    raise SystemExit(main())
