"""自动建图任务配置：集中解析 YAML、默认值和运行时策略。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .exploration_saturation import SaturationPolicy
from .frontier_monitor import FrontierMonitorConfig
from .mapping_return import ReturnToStartSpec
from .mission_executor import (
    AutomaticMissionSpec,
    RecoveryBackUpSpec,
    UnknownWorldMissionSpec,
)
from .showcase_session import parse_mapping_bootstrap_route


def _mapping(value: Any, name: str) -> dict:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a YAML mapping")
    return value


def _required_text(values: dict, key: str, section: str) -> str:
    text = str(values.get(key, "")).strip()
    if not text:
        raise ValueError(f"{section}.{key} must be non-empty")
    return text


def _reject_unexpected_keys(values: dict, allowed: set[str], section: str) -> None:
    unexpected = sorted(set(values) - allowed)
    if unexpected:
        raise ValueError(
            f"{section} contains unsupported keys: {', '.join(unexpected)}"
        )


@dataclass(frozen=True, slots=True)
class MissionConfiguration:
    """Node 可直接装配的任务值对象，不向调用者泄漏 YAML key。"""

    profile: str
    evidence_min_growth_cells: int
    frontier: FrontierMonitorConfig
    automatic: AutomaticMissionSpec | UnknownWorldMissionSpec

    @classmethod
    def load(
        cls,
        *,
        workspace: Path,
        mission_plan_path: Path,
        scan_startup_timeout_s: float,
        status_available: bool,
        dry_run: bool,
        dry_run_delay_s: float,
    ) -> "MissionConfiguration":
        """读取并验证一次；运行期间不再查询松散字典或隐式默认值。"""

        if scan_startup_timeout_s <= 0.0:
            raise ValueError("scan_startup_timeout_s must be positive")
        plan = yaml.safe_load(mission_plan_path.read_text(encoding="utf-8"))
        if not isinstance(plan, dict):
            # 保留既有启动错误文本，避免 launch/验收脚本因重构失去可搜索线索。
            raise ValueError("mission_plan must contain a YAML mapping")
        profile = str(plan.get("profile", "known_world")).strip()
        if profile == "unknown_world":
            return cls._load_unknown_world(
                workspace=workspace,
                plan=plan,
                scan_startup_timeout_s=scan_startup_timeout_s,
                status_available=status_available,
                dry_run=dry_run,
                dry_run_delay_s=dry_run_delay_s,
            )
        if profile != "known_world":
            raise ValueError(f"unsupported mission profile: {profile}")
        automatic = _mapping(
            plan.get("automatic_exploration", {}),
            "automatic_exploration",
        )
        navigation = _mapping(
            plan.get("navigation_mission", {}),
            "navigation_mission",
        )
        acceptance = _mapping(plan.get("acceptance", {}), "acceptance")

        # 路径只在配置层推导一次；任务执行器只接收已验证的绝对路径和值对象。
        explorer_config_path = (
            workspace
            / "src"
            / "embodied_simulation"
            / "config"
            / str(automatic.get("config", "frontier_exploration.yaml"))
        )
        min_growth_cells = int(automatic.get("min_growth_cells", 40))
        frontier = FrontierMonitorConfig(
            timeout_s=float(automatic.get("timeout_s", 300.0)),
            min_runtime_s=float(automatic.get("min_runtime_s", 15.0)),
            stable_map_s=float(automatic.get("stable_map_s", 20.0)),
            completion_status=str(
                automatic.get("completion_status", "exploration_complete")
            ),
            min_known_cells=int(acceptance.get("min_known_map_cells", 0)),
            min_occupied_cells=int(
                acceptance.get("min_occupied_map_cells", 0)
            ),
            min_mapping_path_m=float(
                acceptance.get("min_mapping_path_m", 0.0)
            ),
            frontier_idle_grace_s=float(
                automatic.get("frontier_idle_grace_s", 0.0)
            ),
            status_available=status_available,
            dry_run=dry_run,
            dry_run_delay_s=dry_run_delay_s,
        )
        mission = AutomaticMissionSpec(
            explorer_config_path=explorer_config_path,
            bootstrap_route=tuple(parse_mapping_bootstrap_route(automatic)),
            scan_startup_timeout_s=scan_startup_timeout_s,
            bootstrap_action_timeout_s=float(
                automatic.get("bootstrap_action_timeout_s", 45.0)
            ),
            navigation_timeout_s=float(
                automatic.get("navigation_timeout_s", 330.0)
            ),
            navigate_text=_required_text(
                navigation, "navigate_text", "navigation_mission"
            ),
            patrol_text=_required_text(
                navigation, "patrol_text", "navigation_mission"
            ),
        )
        return cls(
            profile="known_world",
            evidence_min_growth_cells=min_growth_cells,
            frontier=frontier,
            automatic=mission,
        )

    @classmethod
    def _load_unknown_world(
        cls,
        *,
        workspace: Path,
        plan: dict,
        scan_startup_timeout_s: float,
        status_available: bool,
        dry_run: bool,
        dry_run_delay_s: float,
    ) -> "MissionConfiguration":
        """解析 unknown-world 白名单配置；未声明 key 一律拒绝。"""

        _reject_unexpected_keys(
            plan,
            {"schema_version", "profile", "exploration", "navigation", "sanity"},
            "mission",
        )
        exploration = _mapping(plan.get("exploration", {}), "exploration")
        navigation = _mapping(plan.get("navigation", {}), "navigation")
        sanity = _mapping(plan.get("sanity", {}), "sanity")
        _reject_unexpected_keys(
            exploration,
            {
                "provider",
                "config",
                "timeout_s",
                "final_confirmation_timeout_s",
                "min_runtime_s",
                "stable_map_s",
                "frontier_idle_grace_s",
                "min_growth_cells",
                "min_growth_ratio",
                "completion_status",
                "max_recovery_attempts",
                "action_timeout_s",
                "initial_scan_text",
                "recovery_scan_text",
                "recovery_backup",
                "saturation",
                "return_to_start",
            },
            "exploration",
        )
        recovery_backup = _mapping(
            exploration.get("recovery_backup", {}),
            "exploration.recovery_backup",
        )
        saturation = _mapping(
            exploration.get("saturation", {}),
            "exploration.saturation",
        )
        return_to_start = _mapping(
            exploration.get("return_to_start", {}),
            "exploration.return_to_start",
        )
        if "final_confirmation_timeout_s" not in exploration:
            # 最终确认轮会进入外层 acceptance 总预算，因此不能在运行时悄悄
            # 使用代码默认值；YAML 与验收预算必须共享同一个显式事实源。
            raise ValueError(
                "exploration.final_confirmation_timeout_s is required"
            )
        _reject_unexpected_keys(
            recovery_backup,
            {
                "distance_m",
                "speed_mps",
                "timeout_s",
                "minimum_displacement_m",
            },
            "exploration.recovery_backup",
        )
        _reject_unexpected_keys(
            saturation,
            {
                "minimum_low_yield_epochs",
                "minimum_terminal_goals_per_epoch",
                "minimum_mapping_path_m",
                "maximum_residual_available_frontiers",
                "required_map_quiet_s",
                "maximum_final_stop_age_s",
            },
            "exploration.saturation",
        )
        _reject_unexpected_keys(
            return_to_start,
            {
                "timeout_s",
                "map_settle_s",
                "max_xy_error_m",
                "max_yaw_error_rad",
                "zero_velocity_tolerance",
                "max_cmd_vel_age_s",
            },
            "exploration.return_to_start",
        )
        _reject_unexpected_keys(
            navigation,
            {
                "timeout_s",
                "goal_count",
                "goal_seed",
                "minimum_goal_separation_m",
                "goal_clearance_m",
            },
            "navigation",
        )
        _reject_unexpected_keys(
            sanity,
            {
                "min_known_map_cells",
                "min_occupied_map_cells",
                "min_exploration_path_m",
            },
            "sanity",
        )
        provider = str(exploration.get("provider", "explore_lite"))
        if provider != "explore_lite":
            raise ValueError("unknown-world exploration.provider must be explore_lite")
        explorer_config_path = (
            workspace
            / "src"
            / "embodied_simulation"
            / "config"
            / str(exploration.get("config", "frontier_exploration.yaml"))
        )
        min_growth_cells = int(exploration.get("min_growth_cells", 40))
        min_growth_ratio = float(
            exploration.get("min_growth_ratio", 0.002)
        )
        # 这两个值同时进入 recovery 与 saturation；先在配置边界保留稳定的
        # 错误语义，避免后创建的任一值对象抢先抛出更模糊的异常。
        if min_growth_cells < 0:
            raise ValueError(
                "minimum epoch map gain cells must be non-negative"
            )
        frontier = FrontierMonitorConfig(
            timeout_s=float(exploration.get("timeout_s", 600.0)),
            min_runtime_s=float(exploration.get("min_runtime_s", 60.0)),
            stable_map_s=float(exploration.get("stable_map_s", 15.0)),
            completion_status=str(
                exploration.get("completion_status", "exploration_complete")
            ),
            min_known_cells=int(sanity.get("min_known_map_cells", 0)),
            min_occupied_cells=int(sanity.get("min_occupied_map_cells", 0)),
            min_mapping_path_m=float(sanity.get("min_exploration_path_m", 0.0)),
            frontier_idle_grace_s=float(
                exploration.get("frontier_idle_grace_s", 20.0)
            ),
            policy_mode="unknown_world",
            status_available=status_available,
            dry_run=dry_run,
            dry_run_delay_s=dry_run_delay_s,
        )
        mission = UnknownWorldMissionSpec(
            explorer_config_path=explorer_config_path,
            scan_startup_timeout_s=scan_startup_timeout_s,
            exploration_timeout_s=float(exploration.get("timeout_s", 600.0)),
            final_confirmation_timeout_s=float(
                exploration["final_confirmation_timeout_s"]
            ),
            action_timeout_s=float(exploration.get("action_timeout_s", 60.0)),
            navigation_timeout_s=float(navigation.get("timeout_s", 330.0)),
            initial_scan_text=_required_text(
                exploration, "initial_scan_text", "exploration"
            ),
            recovery_scan_text=_required_text(
                exploration, "recovery_scan_text", "exploration"
            ),
            recovery_backup=RecoveryBackUpSpec(
                distance_m=float(recovery_backup.get("distance_m", 0.30)),
                speed_mps=float(recovery_backup.get("speed_mps", 0.08)),
                timeout_s=float(recovery_backup.get("timeout_s", 10.0)),
                minimum_displacement_m=float(
                    recovery_backup.get("minimum_displacement_m", 0.20)
                ),
            ),
            max_recovery_attempts=int(
                exploration.get("max_recovery_attempts", 2)
            ),
            # 与 MappingEvidenceTracker 复用同一个最小增长口径，避免 monitor
            # 和跨 epoch 收敛各自发明一套“有进展”阈值。
            minimum_epoch_map_gain_cells=min_growth_cells,
            minimum_epoch_map_gain_ratio=min_growth_ratio,
            # 恢复扫描结束后沿用 frontier 的地图静默窗；一个 profile 只保留
            # 一套 settle 语义，避免两个超时参数随配置演进发生漂移。
            map_settle_s=frontier.stable_map_s,
            navigation_goal_count=int(navigation.get("goal_count", 3)),
            navigation_goal_seed=int(navigation.get("goal_seed", 0)),
            navigation_goal_minimum_separation_m=float(
                navigation.get("minimum_goal_separation_m", 1.5)
            ),
            navigation_goal_clearance_m=float(
                navigation.get("goal_clearance_m", 0.25)
            ),
            saturation_policy=SaturationPolicy(
                minimum_low_yield_epochs=int(
                    saturation.get("minimum_low_yield_epochs", 2)
                ),
                minimum_terminal_goals_per_epoch=int(
                    saturation.get("minimum_terminal_goals_per_epoch", 3)
                ),
                minimum_mapping_path_m=float(
                    saturation.get("minimum_mapping_path_m", 20.0)
                ),
                maximum_residual_available_frontiers=int(
                    saturation.get(
                        "maximum_residual_available_frontiers", 4
                    )
                ),
                # 复用 recovery 的双增益阈值，避免同一 profile 出现两套互相
                # 冲突的“有效扩图”定义。
                maximum_low_yield_gain_cells=min_growth_cells,
                maximum_low_yield_gain_ratio=min_growth_ratio,
                required_map_quiet_s=float(
                    saturation.get("required_map_quiet_s", 15.0)
                ),
                maximum_final_stop_age_s=float(
                    saturation.get("maximum_final_stop_age_s", 5.0)
                ),
            ),
            return_to_start_spec=ReturnToStartSpec(
                max_xy_error_m=float(
                    return_to_start.get("max_xy_error_m", 0.35)
                ),
                max_yaw_error_rad=float(
                    return_to_start.get("max_yaw_error_rad", 0.35)
                ),
                zero_velocity_tolerance=float(
                    return_to_start.get("zero_velocity_tolerance", 0.001)
                ),
                max_cmd_vel_age_s=float(
                    return_to_start.get("max_cmd_vel_age_s", 1.0)
                ),
            ),
            return_to_start_timeout_s=float(
                return_to_start.get("timeout_s", 180.0)
            ),
            return_map_settle_s=float(
                return_to_start.get("map_settle_s", 15.0)
            ),
        )
        return cls(
            profile="unknown_world",
            evidence_min_growth_cells=min_growth_cells,
            frontier=frontier,
            automatic=mission,
        )
