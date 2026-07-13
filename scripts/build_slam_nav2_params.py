#!/usr/bin/env python3
"""在 Nav2 官方完整参数之上叠加项目自有导航参数。"""

from __future__ import annotations

import argparse
import copy
from pathlib import Path
from typing import Any

import yaml


DEFAULT_BASE = Path("/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml")
DEFAULT_OVERRIDE = Path("src/embodied_navigation/config/navigation_overrides.yaml")
DEFAULT_OUTPUT = Path("logs/slam_nav2_params.yaml")


def deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    """递归合并字典；列表和标量由项目配置整体替换。"""
    result = copy.deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def enable_predicted_obstacle_layer(parameters: dict[str, Any]) -> None:
    """把预测层放在 inflation 前，确保预测占用也会获得安全膨胀。"""
    try:
        costmap = parameters["global_costmap"]["global_costmap"]["ros__parameters"]
    except KeyError as exc:
        raise ValueError("base params missing global_costmap ROS parameters") from exc
    plugins = list(costmap.get("plugins", []))
    plugins = [name for name in plugins if name != "predicted_obstacle_layer"]
    insertion_index = plugins.index("inflation_layer") if "inflation_layer" in plugins else len(plugins)
    plugins.insert(insertion_index, "predicted_obstacle_layer")
    costmap["plugins"] = plugins


def build_parameters(base_path: Path, override_path: Path) -> dict[str, Any]:
    base = yaml.safe_load(base_path.read_text(encoding="utf-8")) or {}
    override = yaml.safe_load(override_path.read_text(encoding="utf-8")) or {}
    if not isinstance(base, dict) or not isinstance(override, dict):
        raise ValueError("Nav2 parameter files must contain YAML mappings")
    merged = deep_merge(base, override)
    enable_predicted_obstacle_layer(merged)
    return merged


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--base", type=Path, default=DEFAULT_BASE)
    parser.add_argument("--override", type=Path, default=DEFAULT_OVERRIDE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    parameters = build_parameters(args.base, args.override)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(parameters, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    plugins = parameters["global_costmap"]["global_costmap"]["ros__parameters"]["plugins"]
    print(f"Nav2 params: {args.output}")
    print(f"Global costmap plugins: {plugins}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
