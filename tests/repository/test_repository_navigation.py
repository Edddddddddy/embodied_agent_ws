"""动态障碍预测与 Nav2 参数装配的仓库级护栏。"""

import importlib.util

import yaml

from repository_test_support import ROOT


def _load_parameter_builder():
    path = ROOT / "scripts" / "build_slam_nav2_params.py"
    spec = importlib.util.spec_from_file_location("build_slam_nav2_params", path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_dynamic_navigation_package_has_typed_tracking_and_predictive_plugin():
    package = ROOT / "src" / "embodied_navigation"
    required = (
        package / "costmap_plugins.xml",
        package / "src" / "dynamic_obstacle_tracker.cpp",
        package / "src" / "constant_velocity_predictor.cpp",
        package / "src" / "predicted_obstacle_layer.cpp",
        package / "test" / "test_dynamic_obstacle_tracker.cpp",
        package / "test" / "test_constant_velocity_predictor.cpp",
        ROOT / "src" / "embodied_agent_interfaces" / "msg" / "DynamicObstacle.msg",
        ROOT / "src" / "embodied_agent_interfaces" / "msg" / "DynamicObstacleArray.msg",
    )
    assert all(path.is_file() for path in required)


def test_parameter_builder_preserves_nav2_and_inserts_prediction_before_inflation(tmp_path):
    module = _load_parameter_builder()
    base = tmp_path / "base.yaml"
    override = tmp_path / "override.yaml"
    base.write_text(
        yaml.safe_dump(
            {
                "global_costmap": {
                    "global_costmap": {
                        "ros__parameters": {
                            "plugins": ["static_layer", "obstacle_layer", "inflation_layer"],
                            "resolution": 0.05,
                        }
                    }
                },
                "planner_server": {"ros__parameters": {"expected_planner_frequency": 20.0}},
            }
        ),
        encoding="utf-8",
    )
    override.write_text(
        "global_costmap:\n  global_costmap:\n    ros__parameters:\n"
        "      update_frequency: 5.0\n      predicted_obstacle_layer:\n"
        "        plugin: embodied_navigation::PredictedObstacleLayer\n",
        encoding="utf-8",
    )
    result = module.build_parameters(base, override)
    costmap = result["global_costmap"]["global_costmap"]["ros__parameters"]
    assert costmap["plugins"] == [
        "static_layer",
        "obstacle_layer",
        "predicted_obstacle_layer",
        "inflation_layer",
    ]
    assert costmap["resolution"] == 0.05
    assert result["planner_server"]["ros__parameters"]["expected_planner_frequency"] == 20.0


def test_ci_builds_navigation_package_but_skips_heavy_gazebo_gate():
    workflow = (ROOT / ".github" / "workflows" / "ros2-ci.yml").read_text(
        encoding="utf-8"
    )
    assert "embodied_navigation" in workflow
    assert "smoke_test_predicted_dynamic_obstacle_navigation.sh" not in workflow
