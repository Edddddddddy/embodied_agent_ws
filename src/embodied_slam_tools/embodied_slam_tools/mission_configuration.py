"""自动建图任务配置：集中解析 YAML、默认值和运行时策略。"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .frontier_monitor import FrontierMonitorConfig
from .mission_executor import AutomaticMissionSpec
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


@dataclass(frozen=True, slots=True)
class MissionConfiguration:
    """Node 可直接装配的任务值对象，不向调用者泄漏 YAML key。"""

    evidence_min_growth_cells: int
    frontier: FrontierMonitorConfig
    automatic: AutomaticMissionSpec

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
            evidence_min_growth_cells=min_growth_cells,
            frontier=frontier,
            automatic=mission,
        )
