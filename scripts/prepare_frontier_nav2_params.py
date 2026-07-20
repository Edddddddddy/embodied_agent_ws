#!/usr/bin/env python3
"""从 Nav2 官方参数生成 frontier 探索所需的完整运行配置。"""

from __future__ import annotations

import argparse
from copy import deepcopy
import math
from pathlib import Path

import yaml


def with_frontier_goal_tolerance(payload: dict, tolerance: float) -> dict:
    """保留官方完整配置，只改探索阶段必须收紧的目标位置容差。"""

    result = deepcopy(payload)
    controller = result["controller_server"]["ros__parameters"]
    controller["general_goal_checker"]["xy_goal_tolerance"] = tolerance
    return result


def with_frontier_progress_checker(
    payload: dict,
    *,
    movement_radius_m: float,
    movement_timeout_s: float,
) -> dict:
    """把探索阶段真正生效的进展门槛写入可留档参数文件。"""

    if not 0.05 <= movement_radius_m <= 0.25:
        raise ValueError("frontier movement radius must be in [0.05, 0.25] m")
    if not 10.0 <= movement_timeout_s <= 60.0:
        raise ValueError("frontier movement timeout must be in [10, 60] s")
    result = deepcopy(payload)
    try:
        checker = result["controller_server"]["ros__parameters"][
            "progress_checker"
        ]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "frontier Nav2 source must contain controller progress_checker"
        ) from exc
    if (
        not isinstance(checker, dict)
        or checker.get("plugin")
        != "nav2_controller::SimpleProgressChecker"
    ):
        raise ValueError("frontier progress_checker must use SimpleProgressChecker")

    # launch 的 RewrittenYaml 临时文件会在进程退出后消失；若只在那里改参数，
    # session 留下的 YAML 会错误显示官方 0.5m/10s。提前写入使验收证据自包含，
    # 同时仍由 Nav2 参数系统在运行时再次校验类型。
    checker["required_movement_radius"] = movement_radius_m
    checker["movement_time_allowance"] = movement_timeout_s
    return result


def with_known_free_frontier_planner(payload: dict) -> dict:
    """让 frontier 的目标契约和 Nav2 路径契约使用同一片已知自由域。"""

    result = deepcopy(payload)
    try:
        global_costmap = result["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]
        planner = result["planner_server"]["ros__parameters"]["GridBased"]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "frontier Nav2 source must contain global_costmap and "
            "planner_server.GridBased"
        ) from exc
    if not isinstance(global_costmap, dict) or not isinstance(planner, dict):
        raise ValueError("frontier Nav2 planner sections must be mappings")
    if global_costmap.get("track_unknown_space") is not True:
        raise ValueError(
            "frontier global costmap must track unknown space"
        )
    if planner.get("plugin") != "nav2_navfn_planner::NavfnPlanner":
        raise ValueError("frontier GridBased planner must use NavfnPlanner")
    if not isinstance(planner.get("allow_unknown"), bool):
        raise ValueError("frontier GridBased.allow_unknown must be boolean")

    # Explore Lite 已经把目标投影到“机器人可达、净空安全的已知自由域”。如果
    # Navfn 又允许穿 unknown，它会给出几何上更短却未经激光确认的捷径，局部控制器
    # 到边界才停车，最终被误诊为 frontier 不可达并形成黑名单风暴。
    planner["allow_unknown"] = False
    return result


def with_slam_toolbox_profile(nav2_payload: dict, slam_payload: dict) -> dict:
    """把独立维护的 SLAM profile 注入 Nav2 参数，而不复制或裁剪其他节点。"""

    if not isinstance(slam_payload, dict) or set(slam_payload) != {"slam_toolbox"}:
        raise ValueError("slam params source must only contain the slam_toolbox node")
    try:
        parameters = slam_payload["slam_toolbox"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise ValueError(
            "slam params source must contain slam_toolbox.ros__parameters"
        ) from exc
    if not isinstance(parameters, dict):
        raise ValueError("slam_toolbox.ros__parameters must be a mapping")

    expected_interfaces = {
        "odom_frame": "odom",
        "map_frame": "map",
        "base_frame": "base_footprint",
        "scan_topic": "/scan",
        "mode": "mapping",
    }
    for key, expected in expected_interfaces.items():
        if parameters.get(key) != expected:
            raise ValueError(f"slam_toolbox {key} must be {expected!r}")
    for key in ("use_sim_time", "check_min_dist_and_heading_precisely"):
        if parameters.get(key) is not True:
            raise ValueError(f"slam_toolbox {key} must be true")

    # SLAM profile 属于机器人策略输入，禁止以后为了“让验收通过”偷偷塞回真值地图、
    # 出生位姿或场景路线。这里做递归键检查，使配置错误在启动 Gazebo 前失败。
    forbidden_prior_keys = {
        "bootstrap_route",
        "expected_targets",
        "map_file_name",
        "map_start_at_dock",
        "map_start_pose",
        "mapping_route",
        "places",
        "reference_map",
        "static_map",
    }

    def nested_keys(value: object):
        if isinstance(value, dict):
            for nested_key, nested_value in value.items():
                yield str(nested_key)
                yield from nested_keys(nested_value)
        elif isinstance(value, list):
            for item in value:
                yield from nested_keys(item)

    forbidden = sorted(forbidden_prior_keys.intersection(nested_keys(parameters)))
    if forbidden:
        raise ValueError(f"slam_toolbox profile contains scenario prior: {forbidden[0]}")

    # 该上限是探索器和 SLAM 前端之间的接口契约：再回到官方 0.50 m，
    # 安全前沿短移动就可能不产出新地图，外部却只看到 Nav2 goal succeeded。
    for key in ("minimum_travel_distance", "minimum_travel_heading"):
        value = parameters.get(key)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or not 0.0 < float(value) <= 0.15
        ):
            raise ValueError(f"slam_toolbox {key} must be in (0.0, 0.15]")

    # 重复房间中的错误回环会在一次 Ceres 优化后改变整张地图拓扑。这里固定
    # “鲁棒核 + 保守候选门槛”接口，让配置退化在启动 Gazebo 前就被拒绝；
    # evaluator 仍只负责事后评分，绝不会把真值地图反馈给机器人策略。
    if parameters.get("do_loop_closing") is not True:
        raise ValueError("slam_toolbox do_loop_closing must be true")
    if parameters.get("ceres_loss_function") not in {"HuberLoss", "CauchyLoss"}:
        raise ValueError("slam_toolbox ceres_loss_function must be robust")

    loop_minimums = {
        "loop_match_minimum_chain_size": 10.0,
        "loop_match_minimum_response_coarse": 0.35,
        "loop_match_minimum_response_fine": 0.45,
    }
    for key, minimum in loop_minimums.items():
        value = parameters.get(key)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or float(value) < minimum
        ):
            raise ValueError(f"slam_toolbox {key} must be >= {minimum}")

    loop_maximums = {
        "loop_search_maximum_distance": 2.5,
        "loop_match_maximum_variance_coarse": 2.0,
    }
    for key, maximum in loop_maximums.items():
        value = parameters.get(key)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(float(value))
            or not 0.0 < float(value) <= maximum
        ):
            raise ValueError(f"slam_toolbox {key} must be in (0.0, {maximum}]")

    result = deepcopy(nav2_payload)
    result["slam_toolbox"] = deepcopy(slam_payload["slam_toolbox"])
    return result


def default_nav2_params() -> Path:
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("nav2_bringup")) / "params/nav2_params.yaml"


def default_slam_params() -> Path:
    return (
        Path(__file__).resolve().parents[1]
        / "src/embodied_simulation/config/frontier_slam_toolbox.yaml"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base", type=Path)
    parser.add_argument("--slam-params-source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--xy-goal-tolerance", type=float, default=0.08)
    parser.add_argument("--progress-radius", type=float, default=0.10)
    parser.add_argument("--progress-timeout", type=float, default=30.0)
    args = parser.parse_args()
    # unknown-world 需要 0.20 m 消化在线栅格误差；0.30 m 上限仍防止把“到附近”无限放宽。
    if not 0.02 <= args.xy_goal_tolerance <= 0.30:
        raise ValueError("frontier xy goal tolerance must be in [0.02, 0.30] m")

    source = args.base or default_nav2_params()
    slam_source = args.slam_params_source or default_slam_params()
    payload = yaml.safe_load(source.read_text(encoding="utf-8"))
    slam_payload = yaml.safe_load(slam_source.read_text(encoding="utf-8"))
    adjusted = with_frontier_goal_tolerance(payload, args.xy_goal_tolerance)
    adjusted = with_frontier_progress_checker(
        adjusted,
        movement_radius_m=args.progress_radius,
        movement_timeout_s=args.progress_timeout,
    )
    adjusted = with_known_free_frontier_planner(adjusted)
    adjusted = with_slam_toolbox_profile(adjusted, slam_payload)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        yaml.safe_dump(adjusted, sort_keys=False),
        encoding="utf-8",
    )
    print(
        f"frontier Nav2 params: {source} -> {args.output} "
        f"(xy_goal_tolerance={args.xy_goal_tolerance:.2f}m, "
        f"progress={args.progress_radius:.2f}m/{args.progress_timeout:.1f}s, "
        f"slam_profile={slam_source})"
    )


if __name__ == "__main__":
    main()
