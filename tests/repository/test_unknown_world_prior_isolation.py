"""Unknown-world 验收必须把场景真值和机器人运行时严格隔离。"""

from __future__ import annotations

import importlib
from pathlib import Path

import pytest
import yaml


ROOT = Path(__file__).resolve().parents[2]


FORBIDDEN_RUNTIME_PRIORS = (
    "bootstrap_route",
    "mapping_route",
    "static_map",
    "reference_map",
    "places",
    "expected_targets",
    "navigate_text",
    "patrol_text",
)


def _contract_module():
    """延迟导入让每条红测都能独立说明缺失的公共契约。"""

    return importlib.import_module(
        "tools.acceptance.scenarios.unknown_world_contract"
    )


@pytest.mark.parametrize("prior_name", FORBIDDEN_RUNTIME_PRIORS)
def test_unknown_world_mission_rejects_scene_specific_runtime_prior(
    prior_name: str,
):
    contract = _contract_module()
    mission = {
        "schema_version": 1,
        "exploration": {
            "provider": "explore_lite",
            "timeout_s": 600.0,
            "nested_scene_prior": {prior_name: "must-not-reach-robot-policy"},
        },
    }

    # 禁止项需要递归检查；把真值藏进子配置同样会让策略“预知”环境。
    with pytest.raises(contract.ScenePriorLeakError, match=prior_name):
        contract.validate_unknown_world_mission(mission)


def _actual_orchestrator_command(
    *, workspace: Path, map_prefix: Path, mission_path: Path
) -> tuple[str, ...]:
    """复刻生产入口传给 AcceptanceSession.spawn 的命令形状。"""

    return (
        "ros2",
        "run",
        "embodied_slam_tools",
        "voice_slam_session_orchestrator",
        "--ros-args",
        "-p",
        f"workspace:={workspace}",
        "-p",
        "mode:=offline",
        "-p",
        f"map_prefix:={map_prefix}",
        "-p",
        f"mission_plan:={mission_path}",
        "-p",
        "startup_timeout_s:=150.0",
        "-p",
        "scan_startup_timeout_s:=20.0",
        "-p",
        "stop_timeout_s:=15.0",
    )


def test_actual_policy_spawn_audit_accepts_only_simulator_assembly_prior(
    tmp_path: Path,
):
    contract = _contract_module()
    workspace = tmp_path / "workspace"
    artifact_dir = tmp_path / "artifacts"
    mission_path = workspace / "config" / "unknown_world_mission.yaml"
    world_path = workspace / "worlds" / "workplace.sdf.xacro"
    scene_spec_path = workspace / "evaluator" / "workplace_scene.yaml"
    truth_map_path = workspace / "evaluator" / "workplace_truth.yaml"
    for path in (mission_path, world_path, scene_spec_path, truth_map_path):
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("fixture\n", encoding="utf-8")
    artifact_dir.mkdir()
    budget = contract.build_unknown_world_timeout_budget(
        _mission_document(),
        mapping_startup_s=150.0,
        scan_startup_s=20.0,
        stage_stop_s=15.0,
        map_save_s=35.0,
        dynamic_navigation_s=180.0,
    )
    environment = contract.build_unknown_world_runtime_environment(
        world_path=world_path,
        spawn={"x": -4.15, "y": -3.15, "yaw": 0.0},
        headless="true",
        use_rviz="false",
        budget=budget,
        gate_timeout_s=budget.gate_s,
        transition_timeout_s=budget.mission_transition_s,
    )
    command = _actual_orchestrator_command(
        workspace=workspace,
        map_prefix=artifact_dir / "unknown_world_map",
        mission_path=mission_path,
    )

    contract.audit_unknown_world_policy_spawn(
        argv=command,
        environment=environment,
        world_path=world_path,
        scene_spec_path=scene_spec_path,
        truth_map_path=truth_map_path,
    )

    # world/spawn 是 StageProcessManager 装配 Gazebo 的输入，不是机器人策略输入；
    # 它们暂时共用 supervisor 环境，但绝不能被塞进策略进程命令行。
    assert environment["NAV2_WORLD"] == str(world_path.resolve())
    assert environment["NAV2_SPAWN_X"] == "-4.15"
    assert all(str(world_path.resolve()) not in token for token in command)


@pytest.mark.parametrize(
    ("command_suffix", "environment_leak"),
    [
        (("--truth-map", "{truth_map}"), {}),
        (("-p", "places:=/tmp/known_places.yaml"), {}),
        ((), {"NAV2_MAP": "/tmp/static_map.yaml"}),
        ((), {"SLAM_BOOTSTRAP_ROUTE": "0,0;1,1"}),
        ((), {"LEAKED_SCENE_SPEC": "{scene_spec}"}),
    ],
)
def test_actual_policy_spawn_audit_rejects_runtime_scene_prior(
    tmp_path: Path,
    command_suffix: tuple[str, ...],
    environment_leak: dict[str, str],
):
    contract = _contract_module()
    workspace = tmp_path / "workspace"
    mission_path = workspace / "config" / "unknown_world_mission.yaml"
    world_path = workspace / "worlds" / "workplace.sdf.xacro"
    scene_spec_path = workspace / "evaluator" / "workplace_scene.yaml"
    truth_map_path = workspace / "evaluator" / "workplace_truth.yaml"
    replacements = {
        "{truth_map}": str(truth_map_path),
        "{scene_spec}": str(scene_spec_path),
    }
    rendered_suffix = tuple(replacements.get(token, token) for token in command_suffix)
    rendered_environment = {
        key: replacements.get(value, value) for key, value in environment_leak.items()
    }
    command = _actual_orchestrator_command(
        workspace=workspace,
        map_prefix=tmp_path / "artifacts" / "unknown_world_map",
        mission_path=mission_path,
    ) + rendered_suffix
    environment = {
        "NAV2_WORLD": str(world_path),
        "NAV2_SPAWN_X": "0.0",
        "NAV2_SPAWN_Y": "0.0",
        "NAV2_SPAWN_YAW": "0.0",
        **rendered_environment,
    }

    with pytest.raises(contract.ScenePriorLeakError):
        contract.audit_unknown_world_policy_spawn(
            argv=command,
            environment=environment,
            world_path=world_path,
            scene_spec_path=scene_spec_path,
            truth_map_path=truth_map_path,
        )


def _mission_document() -> dict:
    return yaml.safe_load(
        (
            ROOT
            / "src/embodied_simulation/config/unknown_world_slam_mission.yaml"
        ).read_text(encoding="utf-8")
    )


def test_timeout_budget_is_derived_from_the_same_mission_document():
    contract = _contract_module()
    budget = contract.build_unknown_world_timeout_budget(
        _mission_document(),
        mapping_startup_s=150.0,
        scan_startup_s=20.0,
        stage_stop_s=15.0,
        map_save_s=35.0,
        dynamic_navigation_s=180.0,
    )

    # 3 个目标各 330s；2 次恢复各含 STOP + scan 两个 60s typed action，
    # 并各自保留 max(stable_map_s, action_timeout_s)=60s 的地图静稳确认硬预算。
    assert budget.sampled_navigation_s == 990.0
    assert budget.recovery_actions_s == 240.0
    assert budget.recovery_confirmation_s == 120.0
    assert budget.exploration_s == 900.0
    assert budget.mission_transition_s == 2830.0
    assert budget.gate_s == 3245.0
    budget.validate_outer_timeouts(
        transition_timeout_s=budget.mission_transition_s,
        gate_timeout_s=budget.gate_s,
    )


def test_recovery_confirmation_uses_larger_mission_declared_hard_budget():
    contract = _contract_module()
    mission = _mission_document()
    mission["exploration"]["stable_map_s"] = 75.0
    budget = contract.build_unknown_world_timeout_budget(
        mission,
        mapping_startup_s=150.0,
        scan_startup_s=20.0,
        stage_stop_s=15.0,
        map_save_s=35.0,
        dynamic_navigation_s=180.0,
    )

    # 两次 recovery confirmation 各取 max(75s, 60s)，与 executor 语义一致。
    assert budget.recovery_confirmation_s == 150.0


def test_timeout_budget_rejects_outer_probe_that_would_expire_first():
    contract = _contract_module()
    budget = contract.build_unknown_world_timeout_budget(
        _mission_document(),
        mapping_startup_s=150.0,
        scan_startup_s=20.0,
        stage_stop_s=15.0,
        map_save_s=35.0,
        dynamic_navigation_s=180.0,
    )

    with pytest.raises(
        contract.UnknownWorldTimeoutBudgetError,
        match="TRANSITION_TIMEOUT",
    ):
        budget.validate_outer_timeouts(
            transition_timeout_s=budget.mission_transition_s - 0.1,
            gate_timeout_s=budget.gate_s,
        )
    with pytest.raises(
        contract.UnknownWorldTimeoutBudgetError,
        match="GATE_TIMEOUT",
    ):
        budget.validate_outer_timeouts(
            transition_timeout_s=budget.mission_transition_s,
            gate_timeout_s=budget.gate_s - 0.1,
        )


def test_unknown_world_environment_overrides_spawn_and_clears_policy_priors(
    tmp_path: Path,
):
    contract = _contract_module()
    budget = contract.build_unknown_world_timeout_budget(
        _mission_document(),
        mapping_startup_s=150.0,
        scan_startup_s=20.0,
        stage_stop_s=15.0,
        map_save_s=35.0,
        dynamic_navigation_s=180.0,
    )
    world = tmp_path / "world.sdf.xacro"
    world.write_text("fixture", encoding="utf-8")

    environment = contract.build_unknown_world_runtime_environment(
        world_path=world,
        spawn={"x": -4.15, "y": -3.15, "yaw": 0.0},
        headless="true",
        use_rviz="false",
        budget=budget,
        gate_timeout_s=budget.gate_s,
        transition_timeout_s=budget.mission_transition_s,
    )

    assert environment["NAV2_SPAWN_X"] == "-4.15"
    assert environment["NAV2_INITIAL_X"] == "0.0"
    assert environment["NAV2_MAP"] == ""
    assert environment["NAV2_PLACES_FILE"] == ""
    assert environment["SLAM_MISSION_PROFILE"] == "unknown_world"
    assert environment["FRONTIER_XY_GOAL_TOLERANCE"] == "0.20"
    assert environment[
        "UNKNOWN_WORLD_RECOVERY_CONFIRMATION_BUDGET_S"
    ] == "120.000"
    assert "EMBODIED_NAV2_PLACES_FILE" in (
        contract.UNKNOWN_WORLD_CLEARED_ENVIRONMENT_KEYS
    )
