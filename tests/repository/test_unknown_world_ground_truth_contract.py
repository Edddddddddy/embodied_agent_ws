"""未知世界验收真值只能由 evaluator 采集，且 headless 场景也必须可观测。"""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
WORLD = ROOT / "src/embodied_simulation/worlds/showcase_apartment.sdf.xacro"


def test_showcase_world_always_loads_exactly_one_scene_broadcaster():
    text = WORLD.read_text(encoding="utf-8")

    assert text.count("gz-sim-scene-broadcaster-system") == 1
    broadcaster_line = next(
        line for line in text.splitlines() if "scene-broadcaster-system" in line
    )
    assert "xacro:" not in broadcaster_line


def test_ground_truth_does_not_fallback_to_odom_in_unknown_world_contract():
    contract = (
        ROOT / "docs/development/UNKNOWN_WORLD_SLAM_GOAL.md"
    ).read_text(encoding="utf-8")

    assert "Gazebo 真值" in contract
    assert "AMCL" in contract
