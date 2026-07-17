#!/usr/bin/env python3
"""Audit Nav2/TurtleBot3 voice-demo assets.

这个审计不启动 Gazebo/Nav2；它只检查演示资产是否齐全：语义目标点、Nav2 launch、
RViz 配置、项目本地 map/world、重型验收脚本。`--require-local-assets` 保留为发布门禁：
如果后续误删本地 map/world，它会把资产缺失从 warning 升级为 blocker。
"""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path
from typing import Any

import yaml


WORKSPACE = Path(__file__).resolve().parents[1]
SIM_DIR = WORKSPACE / "src" / "embodied_simulation"


def _relative(path: Path) -> str:
    try:
        return str(path.relative_to(WORKSPACE))
    except ValueError:
        return str(path)


def _audit_places(blockers: list[str]) -> dict[str, Any]:
    path = SIM_DIR / "config" / "places.yaml"
    if not path.is_file():
        blockers.append("places:missing")
        return {"status": "missing", "path": _relative(path), "place_names": []}
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    places = data.get("places", {})
    place_names = sorted(places) if isinstance(places, dict) else []
    required = {"home", "door", "desk"}
    missing = sorted(required.difference(place_names))
    if missing:
        blockers.append("places:missing_required:" + ",".join(missing))
    return {
        "status": "present" if not missing else "incomplete",
        "path": _relative(path),
        "frame_id": data.get("frame_id", ""),
        "place_count": len(place_names),
        "place_names": place_names,
    }


def _launch_arguments(text: str) -> list[str]:
    return sorted(set(re.findall(r'DeclareLaunchArgument\("([^"]+)"', text)))


def _audit_launch(blockers: list[str]) -> dict[str, Any]:
    path = SIM_DIR / "launch" / "voice_nav2_turtlebot3.launch.py"
    if not path.is_file():
        blockers.append("launch:voice_nav2_turtlebot3_missing")
        return {"status": "missing", "path": _relative(path), "arguments": []}
    text = path.read_text(encoding="utf-8")
    arguments = _launch_arguments(text)
    required = {
        "map",
        "params_file",
        "rviz_config_file",
        "world",
        "use_rviz",
        "headless",
        "slam",
        "x_pose",
        "y_pose",
        "yaw",
    }
    missing = sorted(required.difference(arguments))
    if missing:
        blockers.append("launch:missing_arguments:" + ",".join(missing))
    if "tb3_simulation_launch.py" not in text:
        blockers.append("launch:nav2_tb3_bringup_not_included")
    if "Nav2RobotExecutor" not in text:
        blockers.append("launch:nav2_executor_not_configured")
    uses_project_local_map = "maps" in text and "voice_demo.yaml" in text
    uses_project_local_world = "worlds" in text and "voice_demo.sdf.xacro" in text
    if not uses_project_local_map:
        blockers.append("launch:project_local_map_not_default")
    if not uses_project_local_world:
        blockers.append("launch:project_local_world_not_default")
    launch_ready = not missing and uses_project_local_map and uses_project_local_world
    return {
        "status": "present" if launch_ready else "incomplete",
        "path": _relative(path),
        "arguments": arguments,
        "includes_nav2_tb3": "tb3_simulation_launch.py" in text,
        "uses_nav2_executor": "Nav2RobotExecutor" in text,
        "uses_project_local_map": uses_project_local_map,
        "uses_project_local_world": uses_project_local_world,
    }


def _audit_rviz(blockers: list[str]) -> dict[str, Any]:
    path = SIM_DIR / "rviz" / "voice_nav2_demo.rviz"
    if not path.is_file():
        blockers.append("rviz_config:missing")
        return {"status": "missing", "path": _relative(path), "displays": []}
    text = path.read_text(encoding="utf-8")
    displays = [
        name
        for name in ("TF", "Map", "LaserScan", "Odometry", "GlobalPlan", "GoalPose")
        if name in text
    ]
    required = {"TF", "Map", "LaserScan", "Odometry", "GlobalPlan"}
    missing = sorted(required.difference(displays))
    if missing:
        blockers.append("rviz_config:missing_displays:" + ",".join(missing))
    return {
        "status": "present" if not missing else "incomplete",
        "path": _relative(path),
        "displays": displays,
    }


def _audit_local_assets(
    blockers: list[str], warnings: list[str], *, require_local_assets: bool
) -> dict[str, Any]:
    local_maps = sorted((SIM_DIR / "maps").glob("*.yaml")) if (SIM_DIR / "maps").is_dir() else []
    local_worlds = (
        sorted((SIM_DIR / "worlds").glob("*"))
        if (SIM_DIR / "worlds").is_dir()
        else []
    )
    if not local_maps:
        message = "map:local_asset_required"
        if require_local_assets:
            blockers.append(message)
        warnings.append("map:uses_nav2_builtin_tb3_sandbox")
    if not local_worlds:
        message = "world:local_asset_required"
        if require_local_assets:
            blockers.append(message)
        warnings.append("world:uses_nav2_builtin_tb3_sandbox")
    map_images: list[str] = []
    for map_yaml in local_maps:
        try:
            data = yaml.safe_load(map_yaml.read_text(encoding="utf-8")) or {}
        except yaml.YAMLError:
            blockers.append(f"map:invalid_yaml:{_relative(map_yaml)}")
            continue
        image = data.get("image")
        if not isinstance(image, str) or not image:
            blockers.append(f"map:image_missing:{_relative(map_yaml)}")
            continue
        image_path = (map_yaml.parent / image).resolve()
        map_images.append(_relative(image_path))
        if not image_path.is_file():
            blockers.append(f"map:image_file_missing:{_relative(image_path)}")
    return {
        "local_maps": [_relative(path) for path in local_maps],
        "local_map_images": map_images,
        "local_worlds": [_relative(path) for path in local_worlds],
        "uses_nav2_builtin_map": not local_maps,
        "uses_nav2_builtin_world": not local_worlds,
    }


def _audit_scripts(blockers: list[str]) -> dict[str, Any]:
    required = {
        "nav2_preflight": WORKSPACE / "scripts" / "smoke_test_nav2_preflight.sh",
        "nav2_turtlebot3": WORKSPACE / "scripts" / "smoke_test_nav2_turtlebot3_voice.sh",
        "nav2_live_check": WORKSPACE / "scripts" / "continuous_nav2_voice_evidence.sh",
        # 该 probe 尚在 SLAM/Nav2 领域迁移队列中；路径必须包含领域目录，
        # 否则资产审计会把一个存在的验收程序误报为缺失。
        "nav2_probe": (
            WORKSPACE
            / "tests"
            / "integration"
            / "slam_nav"
            / "test_nav2_turtlebot3_voice.py"
        ),
    }
    missing = [name for name, path in required.items() if not path.is_file()]
    for name in missing:
        blockers.append(f"script:{name}_missing")
    return {
        "status": "present" if not missing else "incomplete",
        "items": {
            name: {"path": _relative(path), "exists": path.is_file()}
            for name, path in required.items()
        },
    }


def build_audit(*, require_local_assets: bool = False) -> dict[str, Any]:
    blockers: list[str] = []
    warnings: list[str] = []
    assets = {
        "places": _audit_places(blockers),
        "launch": _audit_launch(blockers),
        "rviz_config": _audit_rviz(blockers),
        "local_assets": _audit_local_assets(
            blockers, warnings, require_local_assets=require_local_assets
        ),
        "scripts": _audit_scripts(blockers),
    }
    guidance = [
        "可以说：当前已具备语音目标点/巡航到 Nav2 action 的演示链路和语义地点配置。",
        "可以说：RViz 配置已覆盖 TF、map、scan、odom、global plan 等面试展示视图。",
    ]
    if assets["local_assets"]["uses_nav2_builtin_map"]:
        guidance.append("不要说：项目已经自带完整固定 map/world；当前仍复用 Nav2 官方 tb3_sandbox 资产。")
    else:
        guidance.append("可以说：项目已自带本地 map/world 演示资产。")
    return {
        "schema_version": 1,
        "scenario": "nav2_voice_demo_asset_audit",
        "ok": not blockers,
        "blockers": blockers,
        "warnings": warnings,
        "assets": assets,
        "claim_guidance": guidance,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", default="logs/nav2_demo_assets.json")
    parser.add_argument("--require-local-assets", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    output_path = Path(args.output).expanduser()
    if not output_path.is_absolute():
        output_path = WORKSPACE / output_path
    audit = build_audit(require_local_assets=args.require_local_assets)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(audit, ensure_ascii=False, indent=2), encoding="utf-8")
    summary = {
        "status": "PASS" if audit["ok"] else "FAIL",
        "output": str(output_path),
        "blockers": audit["blockers"],
        "warnings": audit["warnings"],
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if not audit["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
