"""自动建图任务配置深模块的行为契约。"""

from pathlib import Path

import pytest

from embodied_slam_tools.mission_configuration import MissionConfiguration
from embodied_slam_tools.mission_executor import UnknownWorldMissionSpec


WORKSPACE = Path(__file__).resolve().parents[3]
MISSION_PLAN = (
    WORKSPACE
    / "src"
    / "embodied_simulation"
    / "config"
    / "showcase_workplace_mission.yaml"
)
UNKNOWN_WORLD_MISSION_PLAN = (
    WORKSPACE
    / "src"
    / "embodied_simulation"
    / "config"
    / "unknown_world_slam_mission.yaml"
)


def test_loads_showcase_plan_into_runtime_ready_values():
    configuration = MissionConfiguration.load(
        workspace=WORKSPACE,
        mission_plan_path=MISSION_PLAN,
        scan_startup_timeout_s=20.0,
        status_available=True,
        dry_run=False,
        dry_run_delay_s=0.0,
    )

    assert configuration.evidence_min_growth_cells == 40
    assert configuration.frontier.timeout_s == 300.0
    assert configuration.frontier.min_runtime_s == 45.0
    assert configuration.frontier.min_known_cells == 6000
    assert configuration.frontier.min_occupied_cells == 150
    assert configuration.frontier.min_mapping_path_m == 10.0
    assert configuration.frontier.status_available is True
    assert configuration.automatic.explorer_config_path == (
        WORKSPACE
        / "src"
        / "embodied_simulation"
        / "config"
        / "frontier_exploration.yaml"
    )
    assert len(configuration.automatic.bootstrap_route) == 9
    assert configuration.automatic.scan_startup_timeout_s == 20.0
    assert configuration.automatic.navigate_text == "去入口"
    assert configuration.automatic.patrol_text == "依次去厨房、办公室"


def test_applies_existing_defaults_without_leaking_yaml_keys(tmp_path):
    plan = tmp_path / "minimal.yaml"
    plan.write_text(
        """
automatic_exploration: {}
navigation_mission:
  navigate_text: 去入口
  patrol_text: 依次去厨房、办公室
""".strip(),
        encoding="utf-8",
    )

    configuration = MissionConfiguration.load(
        workspace=tmp_path,
        mission_plan_path=plan,
        scan_startup_timeout_s=8.0,
        status_available=False,
        dry_run=True,
        dry_run_delay_s=0.25,
    )

    assert configuration.evidence_min_growth_cells == 40
    assert configuration.frontier.timeout_s == 300.0
    assert configuration.frontier.min_runtime_s == 15.0
    assert configuration.frontier.stable_map_s == 20.0
    assert configuration.frontier.frontier_idle_grace_s == 0.0
    assert configuration.frontier.completion_status == "exploration_complete"
    assert configuration.frontier.status_available is False
    assert configuration.frontier.dry_run is True
    assert configuration.frontier.dry_run_delay_s == 0.25
    assert configuration.automatic.bootstrap_route == ()
    assert configuration.automatic.bootstrap_action_timeout_s == 45.0
    assert configuration.automatic.navigation_timeout_s == 330.0


@pytest.mark.parametrize(
    ("contents", "message"),
    [
        ("- not\n- a\n- mapping\n", "mission_plan must contain a YAML mapping"),
        (
            "automatic_exploration: []\nnavigation_mission: {}\n",
            "automatic_exploration must be a YAML mapping",
        ),
        (
            "automatic_exploration: {}\nnavigation_mission: []\n",
            "navigation_mission must be a YAML mapping",
        ),
        (
            "automatic_exploration: {}\nnavigation_mission: {}\nacceptance: []\n",
            "acceptance must be a YAML mapping",
        ),
    ],
)
def test_rejects_structurally_invalid_plans(tmp_path, contents, message):
    plan = tmp_path / "invalid.yaml"
    plan.write_text(contents, encoding="utf-8")

    with pytest.raises(ValueError, match=message):
        MissionConfiguration.load(
            workspace=tmp_path,
            mission_plan_path=plan,
            scan_startup_timeout_s=20.0,
            status_available=True,
            dry_run=False,
            dry_run_delay_s=0.0,
        )


def test_rejects_non_positive_scan_startup_timeout(tmp_path):
    plan = tmp_path / "mission.yaml"
    plan.write_text(
        "automatic_exploration: {}\n"
        "navigation_mission:\n"
        "  navigate_text: 去入口\n"
        "  patrol_text: 依次去厨房、办公室\n",
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="scan_startup_timeout_s must be positive"):
        MissionConfiguration.load(
            workspace=tmp_path,
            mission_plan_path=plan,
            scan_startup_timeout_s=0.0,
            status_available=True,
            dry_run=False,
            dry_run_delay_s=0.0,
        )


def test_loads_unknown_world_plan_without_scene_route_or_named_places():
    configuration = MissionConfiguration.load(
        workspace=WORKSPACE,
        mission_plan_path=UNKNOWN_WORLD_MISSION_PLAN,
        scan_startup_timeout_s=20.0,
        status_available=True,
        dry_run=False,
        dry_run_delay_s=0.0,
    )

    assert configuration.profile == "unknown_world"
    assert configuration.frontier.policy_mode == "unknown_world"
    assert configuration.frontier.timeout_s == 900.0
    assert configuration.frontier.frontier_idle_grace_s == 20.0
    assert isinstance(configuration.automatic, UnknownWorldMissionSpec)
    assert configuration.automatic.max_recovery_attempts == 2
    assert configuration.automatic.minimum_epoch_map_gain_cells == 40
    assert (
        configuration.automatic.minimum_epoch_map_gain_cells
        == configuration.evidence_min_growth_cells
    )
    assert (
        configuration.automatic.map_settle_s
        == configuration.frontier.stable_map_s
    )
    assert configuration.automatic.navigation_goal_count == 3
    assert configuration.automatic.navigation_goal_seed == 20260719
    assert not hasattr(configuration.automatic, "bootstrap_route")
    assert not hasattr(configuration.automatic, "navigate_text")


@pytest.mark.parametrize("invalid", ["-0.1", ".nan", ".inf", "-.inf"])
def test_rejects_invalid_frontier_idle_grace(tmp_path, invalid):
    plan = tmp_path / "unknown-world-invalid-grace.yaml"
    contents = UNKNOWN_WORLD_MISSION_PLAN.read_text(encoding="utf-8")
    plan.write_text(
        contents.replace("frontier_idle_grace_s: 20.0", f"frontier_idle_grace_s: {invalid}"),
        encoding="utf-8",
    )

    with pytest.raises(
        ValueError,
        match="frontier_idle_grace_s must be finite and non-negative",
    ):
        MissionConfiguration.load(
            workspace=WORKSPACE,
            mission_plan_path=plan,
            scan_startup_timeout_s=20.0,
            status_available=True,
            dry_run=False,
            dry_run_delay_s=0.0,
        )


def test_rejects_negative_unknown_world_map_gain_threshold(tmp_path):
    plan = tmp_path / "unknown-world-negative-map-gain.yaml"
    contents = UNKNOWN_WORLD_MISSION_PLAN.read_text(encoding="utf-8")
    plan.write_text(
        contents.replace("min_growth_cells: 40", "min_growth_cells: -1"),
        encoding="utf-8",
    )

    with pytest.raises(
        ValueError,
        match="minimum epoch map gain cells must be non-negative",
    ):
        MissionConfiguration.load(
            workspace=WORKSPACE,
            mission_plan_path=plan,
            scan_startup_timeout_s=20.0,
            status_available=True,
            dry_run=False,
            dry_run_delay_s=0.0,
        )
