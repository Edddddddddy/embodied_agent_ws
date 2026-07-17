"""自动建图任务配置深模块的行为契约。"""

from pathlib import Path

import pytest

from embodied_slam_tools.mission_configuration import MissionConfiguration


WORKSPACE = Path(__file__).resolve().parents[3]
MISSION_PLAN = (
    WORKSPACE
    / "src"
    / "embodied_simulation"
    / "config"
    / "showcase_workplace_mission.yaml"
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
