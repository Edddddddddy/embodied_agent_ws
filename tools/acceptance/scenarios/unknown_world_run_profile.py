"""Strict unknown-world 的触发 profile、现场环境与报告验证。"""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path
from typing import Any, Mapping


EVIDENCE_KIND = "unknown_world_slam_nav_dynamic_replan"
LIVE_VOICE_EVIDENCE_KIND = "voice_unknown_world_slam_nav_e2e"
MAPPING_STARTUP_TIMEOUT_S = 150.0
DEFAULT_LIVE_VOICE_TRIGGER_TIMEOUT_S = 120.0

REQUIRED_CORE_CHECKS = frozenset(
    {
        "unknown_world_profile",
        "mission_sequence_present",
        "mission_completed",
        "mission_outcome_succeeded",
        "map_saved",
        "fresh_session_map",
        "map_quality",
        "frontier_complete",
        "return_to_start",
        "localization_quality",
        "sampled_navigation",
        "nav2_lifecycle_active",
        "dynamic_navigation",
        "cmd_vel_observed",
        "robot_motion_observed",
        "final_task_motion_observed",
        "final_cmd_vel_fresh",
        "final_cmd_vel_zero",
    }
)
REQUIRED_VOICE_CHECKS = frozenset(
    {
        "real_audio_observed",
        "endpoint_observed",
        "wake_accepted",
        "automatic_mission_asr_final",
        "strict_core_schema_v4_passed",
    }
)


@dataclass(frozen=True, slots=True)
class UnknownWorldRunProfile:
    """描述同一条 strict unknown-world 门禁的触发方式与运行依赖。"""

    agent_mode: str
    trigger_source: str
    artifact_directory_name: str
    report_filename: str
    mapping_startup_timeout_s: float
    runtime_environment: Mapping[str, str]

    @classmethod
    def synthetic(cls) -> "UnknownWorldRunProfile":
        """用确定性文本触发，完全不依赖现场音频设备。"""

        return cls(
            agent_mode="offline",
            trigger_source="synthetic",
            artifact_directory_name="unknown_world_slam_nav",
            report_filename="unknown_world_slam_e2e_report.json",
            mapping_startup_timeout_s=MAPPING_STARTUP_TIMEOUT_S,
            runtime_environment={
                "NAV2_PROVIDER_MODE": "mock",
                "NAV2_MICROPHONE_ENABLED": "false",
                "NAV2_CAPTURE_ENABLED": "false",
                "WAKE_WORD_ENABLED": "false",
                "CONTINUOUS_PREFLIGHT_ENABLED": "false",
                "CONTINUOUS_MONITOR_ENABLED": "false",
                "CONTINUOUS_READINESS_ENABLED": "false",
                "VAD_PROVIDER": "energy",
                # mock 链路通常数秒即就绪。若 Lifecycle 响应丢失，45 秒内失败并
                # 输出缺失组件，比留下“界面已开但机器人不动”的假运行状态更安全。
                "SYSTEM_READINESS_TIMEOUT": "45",
            },
        )

    @classmethod
    def live_voice(cls, agent_mode: str) -> "UnknownWorldRunProfile":
        """构造真人麦克风配置；Agent 只能明确选择在线或离线实现。"""

        if agent_mode not in {"offline", "online"}:
            raise ValueError("agent mode must be offline or online")
        return cls(
            agent_mode=agent_mode,
            trigger_source="live_voice",
            artifact_directory_name="voice_unknown_world_slam_nav",
            report_filename="voice_unknown_world_slam_e2e_report.json",
            # 子脚本的系统 readiness 最长允许 180 秒；父状态机必须留出更大
            # 预算，否则冷启动时会出现“子进程仍在准备、父进程先判失败”。
            mapping_startup_timeout_s=240.0,
            runtime_environment={
                "NAV2_PROVIDER_MODE": agent_mode,
                "NAV2_MICROPHONE_ENABLED": "true",
                "NAV2_CAPTURE_ENABLED": "true",
                "WAKE_WORD_ENABLED": "true",
                "CONTINUOUS_PREFLIGHT_ENABLED": "true",
                "CONTINUOUS_MONITOR_ENABLED": "true",
                # 启动 readiness 会在 MAPPING 就绪前抢占麦克风；真人口令应由
                # 联合探针在 MAPPING 后采集，才能证明它触发了本次严格门禁。
                "CONTINUOUS_READINESS_ENABLED": "false",
                "CONTINUOUS_READINESS_REQUIRED": "false",
                "PULSE_CAPTURE_BRIDGE": "auto",
                "SPEAKER_ENABLED": "false",
                "SYSTEM_READINESS_TIMEOUT": "180",
            },
        )


def resolve_profile_runtime_environment(
    profile: UnknownWorldRunProfile,
    *,
    inherited_environment: Mapping[str, str] | None = None,
    pulse_socket: Path = Path("/mnt/wslg/PulseServer"),
) -> dict[str, str]:
    """解析现场音频环境，并把最终选择写入 session manifest。"""

    resolved = dict(profile.runtime_environment)
    if profile.trigger_source != "live_voice":
        return resolved

    parent = os.environ if inherited_environment is None else inherited_environment
    runtime_root = str(parent.get("EMBODIED_RUNTIME_ROOT", "")).strip()
    if runtime_root:
        # shell handler 已解析 linked worktree 的共享资产根；写入受控环境后，
        # manifest 与真正启动的 llama/ASR/VAD 路径使用同一来源。
        resolved["EMBODIED_RUNTIME_ROOT"] = runtime_root
    explicit_server = str(parent.get("PULSE_SERVER", "")).strip()
    if explicit_server:
        resolved["PULSE_SERVER"] = explicit_server
    elif pulse_socket.exists():
        # Codex/非交互 shell 不一定继承 Windows Terminal 的 PULSE_SERVER。
        # WSLg socket 存在时显式选择它，避免 auto 模式静默回退到近静音设备。
        resolved["PULSE_SERVER"] = f"unix:{pulse_socket}"
    return resolved


def verify_report(
    report: Mapping[str, Any],
    *,
    expected_session_id: str,
) -> dict[str, float | int]:
    """二次校验 schema v4，拒绝陈旧、降级或缺字段的伪 PASS。"""

    if report.get("schema_version") != 4 or report.get("passed") is not True:
        raise ValueError("unknown-world report did not pass schema v4")
    if report.get("evidence_kind") != EVIDENCE_KIND:
        raise ValueError("unknown-world report has the wrong evidence kind")
    if report.get("session_id") != expected_session_id:
        raise ValueError("unknown-world report belongs to another session")
    checks = report.get("checks") or {}
    if not isinstance(checks, Mapping) or not REQUIRED_CORE_CHECKS.issubset(checks):
        raise ValueError("unknown-world report is missing required checks")
    if not all(checks[name] is True for name in REQUIRED_CORE_CHECKS):
        raise ValueError(f"unknown-world evidence checks failed: {checks}")
    for section_name in (
        "map_quality",
        "return_to_start",
        "localization",
        "sampled_navigation",
        "dynamic_navigation",
    ):
        section = report.get(section_name)
        if not isinstance(section, Mapping) or section.get("passed") is not True:
            raise ValueError(f"unknown-world section failed: {section_name}")
    # strict frontier 和 bounded-saturation 是互斥的建图完成路径；报告必须至少
    # 有一条通过，但两条路径都受上面的真实返航门禁约束。
    frontier = report.get("frontier")
    approximate = report.get("approximate_completion")
    frontier_passed = isinstance(frontier, Mapping) and (
        frontier.get("passed") is True
    )
    approximate_passed = isinstance(approximate, Mapping) and (
        approximate.get("passed") is True
    )
    if not (frontier_passed or approximate_passed):
        raise ValueError("unknown-world completion section failed")
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


def verify_profile_report(
    report: Mapping[str, Any],
    *,
    profile: UnknownWorldRunProfile,
    expected_session_id: str,
) -> dict[str, float | int]:
    """按触发来源校验报告，并始终复用 strict schema v4 核心校验。"""

    if profile.trigger_source == "synthetic":
        return verify_report(report, expected_session_id=expected_session_id)
    if (
        report.get("schema_version") != 1
        or report.get("evidence_kind") != LIVE_VOICE_EVIDENCE_KIND
        or report.get("passed") is not True
    ):
        raise ValueError("live-voice evidence envelope is invalid")
    if (
        report.get("session_id") != expected_session_id
        or report.get("agent_mode") != profile.agent_mode
        or report.get("trigger_source") != "live_voice"
    ):
        raise ValueError("live-voice evidence metadata does not match this run")
    voice_checks = report.get("checks")
    if (
        not isinstance(voice_checks, Mapping)
        or not REQUIRED_VOICE_CHECKS.issubset(voice_checks)
        or not all(voice_checks[name] is True for name in REQUIRED_VOICE_CHECKS)
    ):
        raise ValueError("live-voice evidence voice checks failed")
    voice_window = report.get("voice_window")
    if (
        not isinstance(voice_window, Mapping)
        or not voice_window.get("matched_endpoint")
        or not voice_window.get("matched_automatic_mission_final")
        or not voice_window.get("matched_wake_event")
    ):
        raise ValueError("live-voice evidence has no matched voice window")
    core_report = report.get("core_report")
    if not isinstance(core_report, Mapping):
        raise ValueError("live-voice evidence envelope has no core_report")
    # 联合报告只能包裹、不能替代 strict unknown-world 证据；因此继续执行
    # schema v4、session_id 与所有 checks 的原有硬门禁。
    return verify_report(core_report, expected_session_id=expected_session_id)


# 兼容现有测试/调用处的内部名称；公开业务入口只使用无下划线函数。
_resolve_profile_runtime_environment = resolve_profile_runtime_environment
_verify_profile_report = verify_profile_report
