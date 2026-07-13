"""SLAM 基线的仓库级结构护栏。"""

from repository_test_support import ROOT


def test_slam_mapping_baseline_has_reproducible_inputs_and_evidence_entrypoints():
    package = ROOT / "src" / "embodied_slam"
    required = (
        package / "package.xml",
        package / "config" / "slam_mapping_ceres.yaml",
        package / "launch" / "mapping_baseline.launch.py",
        package / "src" / "odom_drift_injector_node.cpp",
        package / "src" / "closed_loop_driver_node.cpp",
        ROOT / "src" / "embodied_simulation" / "worlds" / "slam_loop_demo.sdf.xacro",
        ROOT / "scripts" / "audit_slam_mapping_assets.py",
        ROOT / "scripts" / "smoke_test_slam_mapping_baseline.sh",
        ROOT / "tests" / "integration" / "test_slam_mapping_baseline.py",
        package / "launch" / "localization_navigation.launch.py",
        ROOT / "scripts" / "smoke_test_slam_localization_navigation.sh",
        ROOT / "tests" / "integration" / "test_slam_localization_navigation.py",
    )
    assert all(path.is_file() for path in required)

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(encoding="utf-8")
    assert "mapping-stage" in acceptance
    assert "slam-benchmark" in acceptance
    assert "slam-navigation" in acceptance


def test_slam_baseline_exposes_drift_and_loop_closure_as_measurable_variables():
    config = (
        ROOT / "src" / "embodied_slam" / "config" / "slam_mapping_ceres.yaml"
    ).read_text(encoding="utf-8")
    drift_source = (
        ROOT / "src" / "embodied_slam" / "src" / "odom_drift_injector_node.cpp"
    ).read_text(encoding="utf-8")
    probe = (
        ROOT / "tests" / "integration" / "test_slam_mapping_baseline.py"
    ).read_text(encoding="utf-8")

    assert "solver_plugins::CeresSolver" in config
    assert "do_loop_closing: true" in config
    assert "resolution: 0.05" in config
    for parameter in ("linear_scale", "yaw_bias_per_meter", "random_seed"):
        assert parameter in config
        assert parameter in drift_source
    for metric in ("raw_closure_error_m", "raw_ate_rmse_m", "known_area_m2"):
        assert metric in probe


def test_ci_builds_the_slam_package_without_running_the_heavy_gazebo_benchmark():
    workflow = (ROOT / ".github" / "workflows" / "ros2-ci.yml").read_text(
        encoding="utf-8"
    )
    assert "embodied_slam" in workflow
    assert "smoke_test_slam_mapping_baseline.sh" not in workflow
