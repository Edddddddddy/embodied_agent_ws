"""SLAM/Nav2 验收产物与报告 I/O。

这里集中管理地图哈希和终端摘要，避免运行时 ROS Adapter 反向依赖 CLI。
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from tools.acceptance.slam_nav_evidence import cumulative_distance
import yaml


def robot_traveled_distance(positions: list[tuple[float, float]]) -> float:
    """累计有效里程，忽略 Gazebo 重启或里程计重置造成的瞬时跳变。"""

    if not positions:
        return 0.0
    segments: list[tuple[float, float]] = [positions[0]]
    distance = 0.0
    for previous, current in zip(positions, positions[1:]):
        if math.hypot(current[0] - previous[0], current[1] - previous[1]) <= 0.5:
            segments.append(current)
            continue
        distance += cumulative_distance(segments)
        segments = [current]
    return distance + cumulative_distance(segments)


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def map_artifact_sha256(map_yaml: Path) -> tuple[str, Path]:
    """把 YAML 与其引用的栅格绑定，防止同名旧图替换本次验收产物。"""

    payload = yaml.safe_load(map_yaml.read_text(encoding="utf-8"))
    image = Path(str(payload["image"]))
    image_path = image if image.is_absolute() else map_yaml.parent / image
    digest = hashlib.sha256()
    digest.update(map_yaml.read_bytes())
    digest.update(image_path.read_bytes())
    return digest.hexdigest(), image_path


def load_survey_plan(path: Path) -> tuple[list[dict], dict]:
    payload = yaml.safe_load(path.read_text(encoding="utf-8"))
    route = payload.get("mapping_route") if isinstance(payload, dict) else None
    acceptance = payload.get("acceptance") if isinstance(payload, dict) else None
    if not isinstance(route, list) or not route:
        raise ValueError("survey plan requires a non-empty mapping_route")
    if not isinstance(acceptance, dict):
        raise ValueError("survey plan requires acceptance thresholds")
    return route, acceptance


def print_report(report: dict, *, summary_only: bool) -> None:
    """终端默认只展示决策字段；完整证据始终写入 JSON 文件。"""

    if not summary_only:
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
        return
    summary = {
        "passed": report.get("passed"),
        "session_id": report.get("session_id"),
        "final_phase": report.get("final_phase"),
        "mapping_path_m": report.get("mapping_path_m"),
        "frontier_goal_count": report.get("frontier_goal_count"),
        "checks": report.get("checks"),
        "error": report.get("error"),
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)


def build_failure_report(
    *,
    session_id: str,
    session_start_ns: int,
    evidence_kind: str,
    error: str,
    state_sequence: list[int],
    last_state_detail: str,
    map_stats: dict | None,
    frontier_goal_count: int,
    mapping_path_m: float,
    final_cmd_vel: dict,
) -> dict:
    """生成稳定的 schema v3 失败证据；中途异常也必须可机器读取。"""

    return {
        "schema_version": 3,
        "passed": False,
        "session_id": session_id,
        "session_start_ns": session_start_ns,
        "evidence_kind": evidence_kind,
        "error": error,
        "state_sequence": state_sequence,
        "last_state_detail": last_state_detail,
        "map": map_stats,
        "frontier_goal_count": frontier_goal_count,
        "mapping_path_m": round(mapping_path_m, 3),
        "final_cmd_vel": final_cmd_vel,
    }
