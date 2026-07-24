"""持久 Gazebo 演示的附加证据验证。"""

from __future__ import annotations

from typing import Mapping

from tools.acceptance.runtime_continuity import (
    RUNTIME_CONTINUITY_SCHEMA_VERSION,
    build_runtime_continuity_evidence,
)
from tools.acceptance.scenarios.unknown_world_run_profile import verify_report


def verify_persistent_runtime_report(
    report: Mapping[str, object],
    *,
    expected_session_id: str,
    require_rviz: bool,
    agent_mode: str,
) -> dict[str, float | int]:
    """先验证运行时连续性，再复用完整 strict unknown-world 判定。"""

    continuity = report.get("runtime_continuity")
    if agent_mode not in {"offline", "online"}:
        raise ValueError("persistent runtime has an invalid Agent mode")
    required = {
        "gazebo_server",
        f"{agent_mode}_agent",
        "robot_state_publisher",
    }
    if require_rviz:
        required.add("rviz")
    if not isinstance(continuity, Mapping):
        raise ValueError("persistent runtime continuity evidence is missing")
    # 附加证据也有独立、封闭的 schema。拒绝未知字段，避免 verifier 与
    # producer 版本漂移时把未审计数据误当成连续性证明。
    if (
        set(continuity)
        != {
            "schema_version",
            "passed",
            "required_roles",
            "checks",
            "checkpoints",
        }
        or continuity.get("schema_version")
        != RUNTIME_CONTINUITY_SCHEMA_VERSION
    ):
        raise ValueError("persistent runtime continuity schema is invalid")
    checkpoints = continuity.get("checkpoints")
    if (
        not isinstance(checkpoints, Mapping)
        or set(checkpoints) != {"mapping_ready", "navigation_ready"}
    ):
        raise ValueError("persistent runtime continuity checkpoints are missing")
    mapping_ready = checkpoints.get("mapping_ready")
    navigation_ready = checkpoints.get("navigation_ready")
    if not isinstance(mapping_ready, Mapping) or not isinstance(
        navigation_ready, Mapping
    ):
        raise ValueError("persistent runtime continuity checkpoints are invalid")
    # 不信任报告里的 `passed/checks` 缓存；再次比较原始身份字段，防止后处理
    # 或 schema 漂移让“进程已重启”和旧 PASS 标志同时存在。
    recomputed = build_runtime_continuity_evidence(
        mapping_ready,
        navigation_ready,
        required_roles=required,
    )
    checks = continuity.get("checks")
    observed_roles = continuity.get("required_roles")
    if (
        continuity.get("passed") is not True
        or recomputed["passed"] is not True
        or continuity.get("checks") != recomputed["checks"]
        or observed_roles != sorted(required)
        or not isinstance(checks, Mapping)
        or not checks
        or not all(value is True for value in checks.values())
    ):
        raise ValueError("persistent runtime continuity checks failed")
    report_checks = report.get("checks")
    if (
        not isinstance(report_checks, Mapping)
        or report_checks.get("runtime_continuity") is not True
    ):
        raise ValueError("persistent runtime continuity was not gated")
    return verify_report(report, expected_session_id=expected_session_id)
