"""Unknown-world 场景装配契约：隔离仿真、机器人策略与验收真值。"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any


_FORBIDDEN_RUNTIME_PRIORS = frozenset(
    {
        "bootstrap_route",
        "mapping_route",
        "static_map",
        "reference_map",
        "places",
        "expected_targets",
        "navigate_text",
        "patrol_text",
    }
)

_FORBIDDEN_POLICY_ENVIRONMENT_KEYS = frozenset(
    {
        "EMBODIED_NAV2_PLACES_FILE",
        "NAV2_MAP",
        "NAV2_PLACES_FILE",
        "SLAM_MAPPING_ROUTE",
        "SLAM_BOOTSTRAP_ROUTE",
        "SHOWCASE_MAPPING_ROUTE",
        "SHOWCASE_BOOTSTRAP_ROUTE",
    }
)

# world/spawn 只负责 StageProcessManager 内的 Gazebo 装配。当前 supervisor 与
# robot policy 共用一个进程环境，所以这里明确把它们列为唯一允许的场景装配项；
# 真正做到 OS 级不可见，需要以后在 StageProcessManager 的子进程边界拆分环境。
UNKNOWN_WORLD_SIMULATOR_ASSEMBLY_ENVIRONMENT_KEYS = frozenset(
    {
        "NAV2_WORLD",
        "NAV2_SPAWN_X",
        "NAV2_SPAWN_Y",
        "NAV2_SPAWN_YAW",
    }
)

# 这些变量来自旧的 known-world/手工 Nav2 演示。AcceptanceSession 默认继承
# 宿主环境，如果不先移除，用户终端里一次 export 就可能让 unknown-world 偷用
# 静态地图、地点表或错误出生位姿，而且同一 commit 两次验收会得到不同结果。
UNKNOWN_WORLD_CLEARED_ENVIRONMENT_KEYS = (
    "EMBODIED_NAV2_PLACES_FILE",
    "NAV2_MAP",
    "NAV2_PLACES_FILE",
    "NAV2_WORLD",
    "NAV2_PARAMS_FILE",
    "NAV2_INITIAL_X",
    "NAV2_INITIAL_Y",
    "NAV2_INITIAL_YAW",
    "NAV2_SPAWN_X",
    "NAV2_SPAWN_Y",
    "NAV2_SPAWN_YAW",
    "X_POSE",
    "Y_POSE",
    "YAW",
    "SHOWCASE_MAP_PREFIX",
    "SHOWCASE_SESSION_DIR",
    "SLAM_MAPPING_ROUTE",
    "SLAM_BOOTSTRAP_ROUTE",
    "SHOWCASE_MAPPING_ROUTE",
    "SHOWCASE_BOOTSTRAP_ROUTE",
)


class UnknownWorldTimeoutBudgetError(ValueError):
    """外层门禁无法覆盖已声明的内层最坏执行时间。"""


@dataclass(frozen=True, slots=True)
class UnknownWorldTimeoutBudget:
    """从任务声明推导的端到端预算，而不是另一组易漂移的魔数。"""

    mapping_startup_s: float
    probe_subscription_s: float
    scan_startup_s: float
    mapping_start_pose_s: float
    initial_action_s: float
    exploration_s: float
    recovery_actions_s: float
    recovery_backup_s: float
    recovery_confirmation_s: float
    final_confirmation_s: float
    saturation_assessment_s: float
    return_to_start_s: float
    return_map_settle_s: float
    stage_switch_s: float
    sampled_navigation_s: float
    terminal_evidence_s: float
    dynamic_navigation_s: float
    reserve_s: float

    @property
    def mission_transition_s(self) -> float:
        """发送自动任务意图后，任务到达 terminal 所需的保守上限。"""

        return (
            self.scan_startup_s
            + self.mapping_start_pose_s
            + self.initial_action_s
            + self.exploration_s
            + self.recovery_actions_s
            + self.recovery_backup_s
            + self.recovery_confirmation_s
            + self.final_confirmation_s
            + self.saturation_assessment_s
            + self.return_to_start_s
            + self.return_map_settle_s
            + self.stage_switch_s
            + self.sampled_navigation_s
        )

    @property
    def gate_s(self) -> float:
        """包含启动、任务、动态障碍证据与调度余量的外层门禁上限。"""

        return (
            self.mapping_startup_s
            + self.probe_subscription_s
            + self.mission_transition_s
            + self.terminal_evidence_s
            + self.dynamic_navigation_s
            + self.reserve_s
        )

    def validate_outer_timeouts(
        self,
        *,
        transition_timeout_s: float,
        gate_timeout_s: float,
    ) -> None:
        """在启动重型进程前拒绝必然抢跑的外层 timeout。"""

        if transition_timeout_s < self.mission_transition_s:
            raise UnknownWorldTimeoutBudgetError(
                "UNKNOWN_WORLD_TRANSITION_TIMEOUT_S is smaller than the "
                "declared mission budget: "
                f"{transition_timeout_s:.1f}s < {self.mission_transition_s:.1f}s"
            )
        if gate_timeout_s < self.gate_s:
            raise UnknownWorldTimeoutBudgetError(
                "UNKNOWN_WORLD_GATE_TIMEOUT_S is smaller than the declared "
                f"end-to-end budget: {gate_timeout_s:.1f}s < {self.gate_s:.1f}s"
            )

    def audit_environment(self) -> dict[str, str]:
        """把预算组成写入 session manifest 的安全白名单。"""

        return {
            "UNKNOWN_WORLD_REQUIRED_TRANSITION_TIMEOUT_S": (
                f"{self.mission_transition_s:.3f}"
            ),
            "UNKNOWN_WORLD_REQUIRED_GATE_TIMEOUT_S": f"{self.gate_s:.3f}",
            "UNKNOWN_WORLD_EXPLORATION_TIMEOUT_S": f"{self.exploration_s:.3f}",
            "UNKNOWN_WORLD_MAPPING_START_POSE_BUDGET_S": (
                f"{self.mapping_start_pose_s:.3f}"
            ),
            "UNKNOWN_WORLD_NAVIGATION_BUDGET_S": (
                f"{self.sampled_navigation_s:.3f}"
            ),
            "UNKNOWN_WORLD_RECOVERY_ACTION_BUDGET_S": (
                f"{self.recovery_actions_s:.3f}"
            ),
            "UNKNOWN_WORLD_RECOVERY_BACKUP_BUDGET_S": (
                f"{self.recovery_backup_s:.3f}"
            ),
            "UNKNOWN_WORLD_RECOVERY_CONFIRMATION_BUDGET_S": (
                f"{self.recovery_confirmation_s:.3f}"
            ),
            "UNKNOWN_WORLD_FINAL_CONFIRMATION_BUDGET_S": (
                f"{self.final_confirmation_s:.3f}"
            ),
            "UNKNOWN_WORLD_SATURATION_ASSESSMENT_BUDGET_S": (
                f"{self.saturation_assessment_s:.3f}"
            ),
            "UNKNOWN_WORLD_RETURN_TO_START_BUDGET_S": (
                f"{self.return_to_start_s:.3f}"
            ),
            "UNKNOWN_WORLD_RETURN_MAP_SETTLE_BUDGET_S": (
                f"{self.return_map_settle_s:.3f}"
            ),
            "UNKNOWN_WORLD_DYNAMIC_NAVIGATION_TIMEOUT_S": (
                f"{self.dynamic_navigation_s:.3f}"
            ),
        }


def _positive_number(value: Any, name: str) -> float:
    try:
        rendered = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a number") from error
    if not math.isfinite(rendered) or rendered <= 0.0:
        raise ValueError(f"{name} must be positive")
    return rendered


def _nonnegative_number(value: Any, name: str) -> float:
    try:
        rendered = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"{name} must be a number") from error
    if not math.isfinite(rendered) or rendered < 0.0:
        raise ValueError(f"{name} must be non-negative")
    return rendered


def build_unknown_world_timeout_budget(
    mission: Mapping[str, Any],
    *,
    mapping_startup_s: float,
    scan_startup_s: float,
    stage_stop_s: float,
    map_save_s: float,
    dynamic_navigation_s: float,
    navigation_readiness_wait_count: int = 3,
    probe_subscription_s: float = 5.0,
    terminal_evidence_s: float = 50.0,
    reserve_s: float = 30.0,
) -> UnknownWorldTimeoutBudget:
    """由同一份 mission YAML 计算外层 timeout 的最低安全值。"""

    validate_unknown_world_mission(mission)
    exploration = mission.get("exploration")
    navigation = mission.get("navigation")
    if not isinstance(exploration, Mapping) or not isinstance(
        navigation, Mapping
    ):
        raise ValueError("unknown-world mission requires exploration/navigation")
    recovery_attempts = int(exploration.get("max_recovery_attempts", 0))
    goal_count = int(navigation.get("goal_count", 0))
    if recovery_attempts < 0 or goal_count <= 0:
        raise ValueError("recovery attempts and navigation goal count are invalid")
    if navigation_readiness_wait_count <= 0:
        raise ValueError("navigation readiness wait count must be positive")
    startup = _positive_number(mapping_startup_s, "mapping_startup_s")
    action = _positive_number(
        exploration.get("action_timeout_s"),
        "exploration.action_timeout_s",
    )
    map_settle = _nonnegative_number(
        exploration.get("stable_map_s"),
        "exploration.stable_map_s",
    )
    exploration_timeout = _positive_number(
        exploration.get("timeout_s"), "exploration.timeout_s"
    )
    final_confirmation_timeout = _positive_number(
        exploration.get("final_confirmation_timeout_s"),
        "exploration.final_confirmation_timeout_s",
    )
    saturation = exploration.get("saturation")
    if not isinstance(saturation, Mapping):
        raise ValueError(
            "unknown-world mission requires exploration.saturation"
        )
    saturation_map_quiet = _nonnegative_number(
        saturation.get("required_map_quiet_s"),
        "exploration.saturation.required_map_quiet_s",
    )
    return_to_start = exploration.get("return_to_start")
    if not isinstance(return_to_start, Mapping):
        raise ValueError(
            "unknown-world mission requires exploration.return_to_start"
        )
    return_timeout = _positive_number(
        return_to_start.get("timeout_s"),
        "exploration.return_to_start.timeout_s",
    )
    return_map_settle = _nonnegative_number(
        return_to_start.get("map_settle_s"),
        "exploration.return_to_start.map_settle_s",
    )
    recovery_backup = exploration.get("recovery_backup")
    if not isinstance(recovery_backup, Mapping):
        raise ValueError(
            "unknown-world mission requires exploration.recovery_backup"
        )
    recovery_backup_timeout = _positive_number(
        recovery_backup.get("timeout_s"),
        "exploration.recovery_backup.timeout_s",
    )
    navigation_timeout = _positive_number(
        navigation.get("timeout_s"), "navigation.timeout_s"
    )
    # 一次恢复包含 typed STOP 和一次传感器驱动原地扫描，两者都可能使用完整
    # action timeout；这里不按“通常很快”折扣，否则外层 probe 仍可能先杀内层。
    recovery_actions = recovery_attempts * 2.0 * action
    # BackUp 会在 no-clearance 或 approach-attempts-exhausted 这两个 typed
    # 恢复分支触发；外层门禁按每次恢复的最坏路径计费，不能让
    # probe 比合法的内层 Action 更早超时。
    recovery_backup_budget = recovery_attempts * recovery_backup_timeout
    # mission executor 在每次恢复扫描后还会等待地图静稳，并用下面两项的较大值
    # 作为硬 deadline。预算必须从同一份 mission YAML 复刻这条语义，不能漏掉
    # confirmation 后让外层 transition probe 提前终止一个仍合法运行的任务。
    recovery_confirmation = recovery_attempts * max(map_settle, action)
    # 探索硬预算耗尽后，任务不会立刻宣布完成：先在一个 action timeout 内
    # 暂停 Explorer 并排空 UUID 账本，再执行 STOP -> 360° scan -> map quiet
    # -> STOP。这里逐段计费，防止外层 probe 在“最后停车”前误杀合法事务。
    saturation_assessment = (
        action
        + 3.0 * action
        + max(map_settle, saturation_map_quiet, action)
    )
    # 返航 NavigateToPose、TF/零速度验证共享 return_to_start.timeout_s；返航后
    # 仍要等回环与 SLAM 尾帧静稳，二者必须分账，不能把 settle 藏进 stage switch。
    return_map_settle_budget = max(return_map_settle, action)
    stage_switch = (
        _positive_number(map_save_s, "map_save_s")
        + _positive_number(stage_stop_s, "stage_stop_s")
        + navigation_readiness_wait_count * startup
    )
    return UnknownWorldTimeoutBudget(
        mapping_startup_s=startup,
        probe_subscription_s=_positive_number(
            probe_subscription_s, "probe_subscription_s"
        ),
        scan_startup_s=_positive_number(scan_startup_s, "scan_startup_s"),
        # 第一次运动前捕获 map->base_link TF，同样使用 action timeout。
        mapping_start_pose_s=action,
        initial_action_s=action,
        # Executor 与 monitor 共享一个绝对 deadline，因此 recovery 不会把
        # exploration.timeout_s 重置成 (attempts + 1) 倍。
        exploration_s=exploration_timeout,
        recovery_actions_s=recovery_actions,
        recovery_backup_s=recovery_backup_budget,
        recovery_confirmation_s=recovery_confirmation,
        # 只在预算边界的 post-scan 地图仍显著增长时使用一次；它与普通
        # exploration deadline 分账，但仍计入外层最坏时间，不能隐式续期。
        final_confirmation_s=final_confirmation_timeout,
        saturation_assessment_s=saturation_assessment,
        return_to_start_s=return_timeout,
        return_map_settle_s=return_map_settle_budget,
        stage_switch_s=stage_switch,
        sampled_navigation_s=goal_count * navigation_timeout,
        terminal_evidence_s=_positive_number(
            terminal_evidence_s, "terminal_evidence_s"
        ),
        dynamic_navigation_s=_positive_number(
            dynamic_navigation_s, "dynamic_navigation_s"
        ),
        reserve_s=_positive_number(reserve_s, "reserve_s"),
    )


def build_unknown_world_runtime_environment(
    *,
    world_path: Path,
    spawn: Mapping[str, Any],
    headless: str,
    use_rviz: str,
    budget: UnknownWorldTimeoutBudget,
    gate_timeout_s: float,
    transition_timeout_s: float,
) -> dict[str, str]:
    """返回 deterministic unknown-world 覆盖值；地点/静态图仍保持被清除。"""

    spawn_x = float(spawn["x"])
    spawn_y = float(spawn["y"])
    spawn_yaw = float(spawn.get("yaw", 0.0))
    environment = {
        "SLAM_MISSION_PROFILE": "unknown_world",
        "SHOWCASE_ORCHESTRATOR_DRY_RUN": "false",
        "NAV2_WORLD": str(world_path.resolve()),
        "NAV2_MAP": "",
        "NAV2_PLACES_FILE": "",
        # spawn 只装配 Gazebo；SLAM map 坐标仍从 (0,0,0) 开始，不能复用
        # world 坐标作为 AMCL/策略先验。
        "NAV2_SPAWN_X": str(spawn_x),
        "NAV2_SPAWN_Y": str(spawn_y),
        "NAV2_SPAWN_YAW": str(spawn_yaw),
        "NAV2_INITIAL_X": "0.0",
        "NAV2_INITIAL_Y": "0.0",
        "NAV2_INITIAL_YAW": "0.0",
        # 在线栅格中的 approach pose 有离散误差；固定 0.20 m 可减少可达 frontier 被误拒。
        "FRONTIER_XY_GOAL_TOLERANCE": "0.20",
        "HEADLESS": str(headless),
        "USE_RVIZ": str(use_rviz),
        "UNKNOWN_WORLD_GATE_TIMEOUT_S": f"{gate_timeout_s:.3f}",
        "UNKNOWN_WORLD_TRANSITION_TIMEOUT_S": f"{transition_timeout_s:.3f}",
    }
    environment.update(budget.audit_environment())
    return environment


class ScenePriorLeakError(ValueError):
    """Unknown-world 机器人配置中出现了场景先验。"""


def _reject_scene_priors(value: Any, path: tuple[str, ...]) -> None:
    if isinstance(value, Mapping):
        for raw_key, child in value.items():
            key = str(raw_key)
            current_path = (*path, key)
            if key in _FORBIDDEN_RUNTIME_PRIORS:
                raise ScenePriorLeakError(
                    "scene prior is forbidden in unknown-world mission: "
                    + ".".join(current_path)
                )
            # 必须递归检查，避免把固定路线藏进 recovery/options 等子字典绕过边界。
            _reject_scene_priors(child, current_path)
        return
    if isinstance(value, Sequence) and not isinstance(
        value, (str, bytes, bytearray)
    ):
        for index, child in enumerate(value):
            _reject_scene_priors(child, (*path, str(index)))


def validate_unknown_world_mission(mission: Mapping[str, Any]) -> None:
    """拒绝任何会让机器人预知地图、路线或语义坐标的运行时配置。"""

    if not isinstance(mission, Mapping):
        raise TypeError("unknown-world mission must be a mapping")
    _reject_scene_priors(mission, ("mission",))


def _argument_declares_scene_prior(token: str) -> bool:
    normalized = token.strip().lower().replace("_", "-")
    forbidden_names = {
        *(name.replace("_", "-") for name in _FORBIDDEN_RUNTIME_PRIORS),
        "truth-map",
        "scene-spec",
        "world-file",
    }
    return any(
        normalized == f"--{name}"
        or normalized.startswith(f"--{name}=")
        or normalized.startswith(f"{name}:=")
        or normalized.startswith(f"{name}=")
        for name in forbidden_names
    )


def audit_unknown_world_policy_spawn(
    *,
    argv: Sequence[str],
    environment: Mapping[str, str],
    world_path: Path,
    scene_spec_path: Path,
    truth_map_path: Path,
) -> None:
    """审计即将交给 AcceptanceSession.spawn 的真实命令和环境。"""

    command = tuple(str(token) for token in argv)
    if not command:
        raise ScenePriorLeakError("unknown-world policy command is empty")

    resolved_paths = {
        "world": str(world_path.resolve()),
        "scene spec": str(scene_spec_path.resolve()),
        "truth map": str(truth_map_path.resolve()),
    }
    for token in command:
        if _argument_declares_scene_prior(token):
            raise ScenePriorLeakError(
                f"scene-prior argument is forbidden in robot policy argv: {token}"
            )
        for label, sensitive_path in resolved_paths.items():
            if sensitive_path and sensitive_path in token:
                raise ScenePriorLeakError(
                    f"{label} is forbidden in robot policy argv: {token}"
                )

    for raw_key, raw_value in environment.items():
        key = str(raw_key)
        value = str(raw_value).strip()
        if not value:
            continue
        normalized_key = key.upper()
        if normalized_key in _FORBIDDEN_POLICY_ENVIRONMENT_KEYS:
            raise ScenePriorLeakError(
                f"scene-prior environment variable is not empty: {key}"
            )
        canonical_key = key.lower().replace("_", "-")
        if any(
            prior.replace("_", "-") in canonical_key
            for prior in _FORBIDDEN_RUNTIME_PRIORS
        ):
            raise ScenePriorLeakError(
                f"scene-prior environment variable is forbidden: {key}"
            )
        for label in ("scene spec", "truth map"):
            if resolved_paths[label] in value:
                raise ScenePriorLeakError(
                    f"{label} leaked through environment variable: {key}"
                )
        if (
            resolved_paths["world"] in value
            and normalized_key
            not in UNKNOWN_WORLD_SIMULATOR_ASSEMBLY_ENVIRONMENT_KEYS
        ):
            raise ScenePriorLeakError(
                f"world path may only be used for simulator assembly: {key}"
            )
