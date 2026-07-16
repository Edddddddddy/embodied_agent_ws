"""SLAM 基线的仓库级结构护栏。"""

import importlib.util
import yaml

from repository_test_support import ROOT, assert_acceptance_modes


def test_frontier_nav2_params_only_narrow_the_mapping_goal_tolerance():
    script = ROOT / "scripts" / "prepare_frontier_nav2_params.py"
    spec = importlib.util.spec_from_file_location("frontier_nav2_params", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    source = {
        "controller_server": {
            "ros__parameters": {
                "general_goal_checker": {"xy_goal_tolerance": 0.25},
                "FollowPath": {"plugin": "nav2_mppi_controller::MPPIController"},
            }
        },
        "planner_server": {"ros__parameters": {"expected_planner_frequency": 20.0}},
    }

    adjusted = module.with_frontier_goal_tolerance(source, 0.08)

    assert source["controller_server"]["ros__parameters"][
        "general_goal_checker"
    ]["xy_goal_tolerance"] == 0.25
    assert adjusted["controller_server"]["ros__parameters"][
        "general_goal_checker"
    ]["xy_goal_tolerance"] == 0.08
    assert adjusted["planner_server"] == source["planner_server"]


def test_autonomous_frontier_mission_has_pinned_runtime_and_typed_contracts():
    repos_path = ROOT / "config" / "frontier_exploration.repos"
    setup_script = ROOT / "scripts" / "setup_frontier_exploration.sh"
    exploration_config = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "config"
        / "frontier_exploration.yaml"
    )
    mission_plan = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "config"
        / "showcase_workplace_mission.yaml"
    )
    smoke = ROOT / "scripts" / "smoke_test_voice_slam_automatic_mission.sh"
    assert all(
        path.is_file()
        for path in (repos_path, setup_script, exploration_config, mission_plan, smoke)
    )

    repos = yaml.safe_load(repos_path.read_text(encoding="utf-8"))
    dependency = repos["repositories"]["third_party/m-explore-ros2"]
    assert dependency["type"] == "git"
    assert dependency["url"] == "https://github.com/robo-friends/m-explore-ros2.git"
    assert len(dependency["version"]) == 40  # 固定提交，避免上游变化破坏演示。

    action_contract = (
        ROOT
        / "src"
        / "embodied_agent_interfaces"
        / "action"
        / "ManageSlamSession.action"
    ).read_text(encoding="utf-8")
    state_contract = (
        ROOT
        / "src"
        / "embodied_agent_interfaces"
        / "msg"
        / "SlamSessionState.msg"
    ).read_text(encoding="utf-8")
    assert "RUN_AUTOMATIC_MISSION=5" in action_contract
    for phase in ("AUTOMATIC_MAPPING=10", "AUTOMATIC_NAVIGATING=11", "MISSION_COMPLETED=12"):
        assert phase in state_contract

    plan = yaml.safe_load(mission_plan.read_text(encoding="utf-8"))
    assert plan["automatic_exploration"]["provider"] == "explore_lite"
    assert plan["automatic_exploration"]["timeout_s"] > 0
    assert plan["navigation_mission"]["expected_targets"] == [
        "entrance",
        "kitchen",
        "office",
    ]


def test_slam_mapping_baseline_has_reproducible_inputs_and_evidence_entrypoints():
    package = ROOT / "src" / "embodied_slam"
    required = (
        package / "package.xml",
        package / "config" / "slam_mapping_ceres.yaml",
        package / "launch" / "mapping_baseline.launch.py",
        package / "launch" / "lidar_loop_candidate.launch.py",
        package / "launch" / "lidar_loop_verifier.launch.py",
        package / "launch" / "lidar_loop_constraint_gate.launch.py",
        package / "include" / "embodied_slam" / "lidar_loop_runtime.hpp",
        package / "include" / "embodied_slam" / "lidar_loop_verifier.hpp",
        package / "include" / "embodied_slam" / "lidar_submap_builder.hpp",
        package / "include" / "embodied_slam" / "lidar_loop_constraint_gate.hpp",
        package / "include" / "embodied_slam" / "lidar_loop_sequence_consistency.hpp",
        package / "include" / "embodied_slam" / "loop_constraint_adapter.hpp",
        package / "src" / "lidar_loop_runtime.cpp",
        package / "src" / "lidar_loop_verifier.cpp",
        package / "src" / "lidar_submap_builder.cpp",
        package / "src" / "lidar_loop_constraint_gate.cpp",
        package / "src" / "lidar_loop_sequence_consistency.cpp",
        package / "src" / "loop_constraint_adapter.cpp",
        package / "src" / "lidar_loop_candidate_node.cpp",
        package / "src" / "lidar_loop_verifier_node.cpp",
        package / "src" / "odom_drift_injector_node.cpp",
        package / "src" / "closed_loop_driver_node.cpp",
        ROOT / "src" / "embodied_simulation" / "worlds" / "slam_loop_demo.sdf.xacro",
        ROOT / "scripts" / "audit_slam_mapping_assets.py",
        ROOT / "scripts" / "smoke_test_slam_mapping_baseline.sh",
        ROOT / "tests" / "integration" / "slam_nav" / "test_slam_mapping_baseline.py",
        ROOT / "tests" / "integration" / "slam_nav" / "test_lidar_loop_runtime.py",
        ROOT / "scripts" / "smoke_test_lidar_loop_runtime.sh",
        package / "launch" / "localization_navigation.launch.py",
        ROOT / "scripts" / "smoke_test_slam_localization_navigation.sh",
        ROOT
        / "tests"
        / "integration"
        / "slam_nav"
        / "test_slam_localization_navigation.py",
        ROOT / "tools" / "evaluation" / "evaluate_slam_trajectory.py",
        ROOT / "tools" / "evaluation" / "extract_rosbag_trajectory.py",
        ROOT / "tools" / "evaluation" / "setup_openloris_groundtruth.py",
        ROOT / "tools" / "evaluation" / "compare_gtsam_switchable_sequences.py",
        ROOT / "tools" / "evaluation" / "compare_lidar_sequence_ablation.py",
        ROOT / "docs" / "evidence" / "gtsam_switchable_multisequence.json",
        ROOT / "docs" / "evidence" / "lidar_sequence_ablation_multisequence.json",
        ROOT / "tests" / "evaluation" / "test_slam_trajectory_evaluation.py",
    )
    assert all(path.is_file() for path in required)

    assert_acceptance_modes(
        "mapping-stage",
        "slam-benchmark",
        "slam-navigation",
        "slam-evaluation-stage",
        "openloris-evaluate",
        "lidar-loop-runtime",
    )


def test_live_lidar_loop_frontend_is_typed_lifecycle_and_commit_is_default_off():
    interfaces = ROOT / "src" / "embodied_agent_interfaces"
    package = ROOT / "src" / "embodied_slam"
    node = (package / "src" / "lidar_loop_candidate_node.cpp").read_text(
        encoding="utf-8"
    )
    verifier = (package / "src" / "lidar_loop_verifier_node.cpp").read_text(
        encoding="utf-8"
    )
    gate = (package / "src" / "lidar_loop_constraint_gate_node.cpp").read_text(
        encoding="utf-8"
    )
    backend = (
        package / "src" / "instrumented_async_slam_toolbox_node.cpp"
    ).read_text(encoding="utf-8")
    launch = (package / "launch" / "mapping_baseline.launch.py").read_text(
        encoding="utf-8"
    )

    assert (interfaces / "msg" / "LidarLoopCandidate.msg").is_file()
    assert (interfaces / "msg" / "LidarLoopCandidateArray.msg").is_file()
    assert (interfaces / "msg" / "LidarLoopVerification.msg").is_file()
    assert (interfaces / "msg" / "LidarLoopVerificationArray.msg").is_file()
    assert (interfaces / "msg" / "LidarLoopConstraintDecision.msg").is_file()
    assert (interfaces / "msg" / "LidarLoopConstraintResult.msg").is_file()
    assert "rclcpp_lifecycle::LifecycleNode" in node
    assert "RCLCPP_COMPONENTS_REGISTER_NODE" in node
    assert "output.shadow_only = true" in node
    assert "pose_graph" not in node.lower()
    assert "rclcpp_lifecycle::LifecycleNode" in verifier
    assert "RCLCPP_COMPONENTS_REGISTER_NODE" in verifier
    assert "output.shadow_only = true" in verifier
    assert "matchLidarScans" not in verifier  # 几何算法封装在可单测的领域对象中。
    assert "rclcpp_lifecycle::LifecycleNode" in gate
    assert "RCLCPP_COMPONENTS_REGISTER_NODE" in gate
    assert 'declare_parameter<bool>("commit_enabled", false)' in gate
    assert 'declare_parameter<bool>("enable_multi_hypothesis_sequence", true)' in gate
    assert 'declare_parameter<int>("minimum_sequence_confirmations", 3)' in gate
    assert "policy_approved" in gate and "commit_requested" in gate
    assert 'declare_parameter<bool>("external_loop_constraint_enabled", false)' in backend
    assert "commit_not_requested" in backend
    assert "query_scan_not_resolved" not in backend  # 原因由纯 C++ adapter 统一生成。
    verification_contract = (
        interfaces / "msg" / "LidarLoopVerification.msg"
    ).read_text(encoding="utf-8")
    assert "query_submap_scans" in verification_contract
    assert "candidate_submap_scans" in verification_contract
    assert "create_subscription<Odometry>" in verifier
    assert "hasGeometry" in verifier
    assert "lidar_loop_verifier.launch.py" in launch
    assert "lidar_loop_constraint_gate.launch.py" in launch
    assert "enable_loop_candidate_shadow" in launch
    assert '"enable_loop_constraint_commit", default_value="false"' in launch
    assert '"matching_mode": "scan_to_submap"' in launch
    assert '"odometry_topic": "/slam/odom"' in launch


def test_slam_baseline_exposes_drift_and_loop_closure_as_measurable_variables():
    config = (
        ROOT / "src" / "embodied_slam" / "config" / "slam_mapping_ceres.yaml"
    ).read_text(encoding="utf-8")
    drift_source = (
        ROOT / "src" / "embodied_slam" / "src" / "odom_drift_injector_node.cpp"
    ).read_text(encoding="utf-8")
    probe = (
        ROOT / "tests" / "integration" / "slam_nav" / "test_slam_mapping_baseline.py"
    ).read_text(encoding="utf-8")

    assert "solver_plugins::CeresSolver" in config
    assert "do_loop_closing: true" in config
    assert "resolution: 0.05" in config
    for parameter in ("linear_scale", "yaw_bias_per_meter", "random_seed"):
        assert parameter in config
        assert parameter in drift_source
    for metric in ("raw_closure_error_m", "raw_ate_rmse_m", "known_area_m2"):
        assert metric in probe


def test_gtsam_switchable_constraints_are_exposed_but_default_off():
    package = ROOT / "src" / "embodied_slam"
    for name in ("slam_mapping_gtsam.yaml", "openloris_mapping_gtsam.yaml"):
        config = (package / "config" / name).read_text(encoding="utf-8")
        assert "gtsam_enable_switchable_loop_constraints: false" in config
        assert "gtsam_switch_prior_sigma: 1.0" in config
        assert "gtsam_switch_suppression_threshold: 0.5" in config

    optimizer = (package / "src" / "gtsam_pose_graph.cpp").read_text(encoding="utf-8")
    assert "SwitchableBetweenFactor" in optimizer
    assert "PriorFactor<double>" in optimizer


def test_ci_builds_the_slam_package_without_running_the_heavy_gazebo_benchmark():
    workflow = (ROOT / ".github" / "workflows" / "ros2-ci.yml").read_text(
        encoding="utf-8"
    )
    assert "embodied_slam" in workflow
    assert "smoke_test_slam_mapping_baseline.sh" not in workflow


def test_openloris_runner_uses_a_fastdds_safe_domain_id():
    runner = (ROOT / "tools" / "evaluation" / "run_openloris_slam_replay.sh").read_text(
        encoding="utf-8"
    )
    assert "120 + $$ % 80" in runner
    assert "ROS_DOMAIN_ID > 232" in runner
    assert 'ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-241}"' not in runner
