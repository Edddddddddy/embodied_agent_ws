"""SLAM 基线的仓库级结构护栏。"""

from repository_test_support import ROOT


def test_slam_mapping_baseline_has_reproducible_inputs_and_evidence_entrypoints():
    package = ROOT / "src" / "embodied_slam"
    required = (
        package / "package.xml",
        package / "config" / "slam_mapping_ceres.yaml",
        package / "launch" / "mapping_baseline.launch.py",
        package / "launch" / "lidar_loop_candidate.launch.py",
        package / "include" / "embodied_slam" / "lidar_loop_runtime.hpp",
        package / "src" / "lidar_loop_runtime.cpp",
        package / "src" / "lidar_loop_candidate_node.cpp",
        package / "src" / "odom_drift_injector_node.cpp",
        package / "src" / "closed_loop_driver_node.cpp",
        ROOT / "src" / "embodied_simulation" / "worlds" / "slam_loop_demo.sdf.xacro",
        ROOT / "scripts" / "audit_slam_mapping_assets.py",
        ROOT / "scripts" / "smoke_test_slam_mapping_baseline.sh",
        ROOT / "tests" / "integration" / "test_slam_mapping_baseline.py",
        ROOT / "tests" / "integration" / "test_lidar_loop_runtime.py",
        ROOT / "scripts" / "smoke_test_lidar_loop_runtime.sh",
        package / "launch" / "localization_navigation.launch.py",
        ROOT / "scripts" / "smoke_test_slam_localization_navigation.sh",
        ROOT / "tests" / "integration" / "test_slam_localization_navigation.py",
        ROOT / "scripts" / "evaluate_slam_trajectory.py",
        ROOT / "scripts" / "extract_rosbag_trajectory.py",
        ROOT / "scripts" / "setup_openloris_groundtruth.py",
        ROOT / "tests" / "repository" / "test_slam_trajectory_evaluation.py",
    )
    assert all(path.is_file() for path in required)

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(encoding="utf-8")
    assert "mapping-stage" in acceptance
    assert "slam-benchmark" in acceptance
    assert "slam-navigation" in acceptance
    assert "slam-evaluation-stage" in acceptance
    assert "openloris-evaluate" in acceptance
    assert "lidar-loop-runtime" in acceptance


def test_live_lidar_loop_frontend_is_typed_lifecycle_and_shadow_only():
    interfaces = ROOT / "src" / "embodied_agent_interfaces"
    package = ROOT / "src" / "embodied_slam"
    node = (package / "src" / "lidar_loop_candidate_node.cpp").read_text(
        encoding="utf-8"
    )
    launch = (package / "launch" / "mapping_baseline.launch.py").read_text(
        encoding="utf-8"
    )

    assert (interfaces / "msg" / "LidarLoopCandidate.msg").is_file()
    assert (interfaces / "msg" / "LidarLoopCandidateArray.msg").is_file()
    assert "rclcpp_lifecycle::LifecycleNode" in node
    assert "RCLCPP_COMPONENTS_REGISTER_NODE" in node
    assert "output.shadow_only = true" in node
    assert "pose_graph" not in node.lower()
    assert "enable_loop_candidate_shadow" in launch


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


def test_openloris_runner_uses_a_fastdds_safe_domain_id():
    runner = (ROOT / "scripts" / "run_openloris_slam_replay.sh").read_text(
        encoding="utf-8"
    )
    assert "120 + $$ % 80" in runner
    assert "ROS_DOMAIN_ID > 232" in runner
    assert 'ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-241}"' not in runner
