#!/usr/bin/env python3
"""从 Nav2 官方参数生成仅用于 frontier 探索的窄目标容差配置。"""

from __future__ import annotations

import argparse
from copy import deepcopy
from pathlib import Path

import yaml


def with_frontier_goal_tolerance(payload: dict, tolerance: float) -> dict:
    """保留官方完整配置，只改探索阶段必须收紧的目标位置容差。"""

    result = deepcopy(payload)
    controller = result["controller_server"]["ros__parameters"]
    controller["general_goal_checker"]["xy_goal_tolerance"] = tolerance
    return result


def default_nav2_params() -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("nav2_bringup")) / "params/nav2_params.yaml"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--xy-goal-tolerance", type=float, default=0.08)
    args = parser.parse_args()
    if not 0.02 <= args.xy_goal_tolerance <= 0.15:
        raise ValueError("frontier xy goal tolerance must be in [0.02, 0.15] m")

    source = args.base or default_nav2_params()
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    adjusted = with_frontier_goal_tolerance(payload, args.xy_goal_tolerance)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(adjusted, sort_keys=False),
        encoding="utf-8",
    )
    print(
        f"frontier Nav2 params: {source} -> {args.output} "
        f"(xy_goal_tolerance={args.xy_goal_tolerance:.2f}m)"
    )


if __name__ == "__main__":
    main()
