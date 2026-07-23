"""SLAM 基线的仓库级结构护栏。"""

import importlib.util
import subprocess
import sys

import pytest
import yaml

from repository_test_support import ROOT, assert_acceptance_modes


def _frontier_nav2_source(*, allow_unknown=True):
    return {
        "controller_server": {
            "ros__parameters": {
                "general_goal_checker": {"xy_goal_tolerance": 0.25},
                "progress_checker": {
                    "plugin": "nav2_controller::SimpleProgressChecker",
                    "required_movement_radius": 0.5,
                    "movement_time_allowance": 10.0,
                },
            }
        },
        "global_costmap": {
            "global_costmap": {
                "ros__parameters": {"track_unknown_space": True}
            }
        },
        "planner_server": {
            "ros__parameters": {
                "GridBased": {
                    "plugin": "nav2_navfn_planner::NavfnPlanner",
                    "allow_unknown": allow_unknown,
                }
            }
        },
    }


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


def test_frontier_progress_checker_is_written_into_auditable_params():
    script = ROOT / "scripts" / "prepare_frontier_nav2_params.py"
    spec = importlib.util.spec_from_file_location("frontier_nav2_params", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    source = _frontier_nav2_source()

    adjusted = module.with_frontier_progress_checker(
        source,
        movement_radius_m=0.10,
        movement_timeout_s=30.0,
    )
    source_checker = source["controller_server"]["ros__parameters"][
        "progress_checker"
    ]
    adjusted_checker = adjusted["controller_server"]["ros__parameters"][
        "progress_checker"
    ]

    assert source_checker["required_movement_radius"] == 0.5
    assert source_checker["movement_time_allowance"] == 10.0
    assert adjusted_checker["required_movement_radius"] == 0.10
    assert adjusted_checker["movement_time_allowance"] == 30.0


@pytest.mark.parametrize(
    ("radius", "timeout"),
    [(0.0, 30.0), (0.30, 30.0), (0.10, 5.0), (0.10, 90.0)],
)
def test_frontier_progress_checker_rejects_unsafe_contract(radius, timeout):
    script = ROOT / "scripts" / "prepare_frontier_nav2_params.py"
    spec = importlib.util.spec_from_file_location("frontier_nav2_params", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    with pytest.raises(ValueError, match="frontier movement"):
        module.with_frontier_progress_checker(
            _frontier_nav2_source(),
            movement_radius_m=radius,
            movement_timeout_s=timeout,
        )


def test_frontier_planner_uses_known_free_space_without_mutating_source():
    script = ROOT / "scripts" / "prepare_frontier_nav2_params.py"
    spec = importlib.util.spec_from_file_location("frontier_nav2_params", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    source = _frontier_nav2_source(allow_unknown=True)

    adjusted = module.with_known_free_frontier_planner(source)

    assert source["planner_server"]["ros__parameters"]["GridBased"][
        "allow_unknown"
    ] is True
    assert adjusted["global_costmap"]["global_costmap"]["ros__parameters"][
        "track_unknown_space"
    ] is True
    assert adjusted["planner_server"]["ros__parameters"]["GridBased"][
        "allow_unknown"
    ] is False


@pytest.mark.parametrize(
    "source",
    [
        {},
        {
            "global_costmap": {
                "global_costmap": {
                    "ros__parameters": {"track_unknown_space": False}
                }
            },
            "planner_server": {
                "ros__parameters": {
                    "GridBased": {
                        "plugin": "nav2_navfn_planner::NavfnPlanner",
                        "allow_unknown": True,
                    }
                }
            },
        },
    ],
)
def test_frontier_planner_rejects_missing_or_inconsistent_unknown_contract(source):
    script = ROOT / "scripts" / "prepare_frontier_nav2_params.py"
    spec = importlib.util.spec_from_file_location("frontier_nav2_params", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)

    with pytest.raises(ValueError, match="frontier"):
        module.with_known_free_frontier_planner(source)


def test_frontier_slam_profile_is_complete_and_accepts_safe_short_motion():
    script = ROOT / "scripts" / "prepare_frontier_nav2_params.py"
    spec = importlib.util.spec_from_file_location("frontier_nav2_params", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    slam_source = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "config"
        / "frontier_slam_toolbox.yaml"
    )
    slam_payload = yaml.safe_load(slam_source.read_text(encoding="utf-8"))

    adjusted = module.with_slam_toolbox_profile({"keep": {"value": 1}}, slam_payload)
    parameters = adjusted["slam_toolbox"]["ros__parameters"]

    assert adjusted["keep"] == {"value": 1}
    assert adjusted["slam_toolbox"] == slam_payload["slam_toolbox"]
    assert parameters["solver_plugin"] == "solver_plugins::CeresSolver"
    assert parameters["use_map_saver"] is True
    assert parameters["use_sim_time"] is True
    assert parameters["odom_frame"] == "odom"
    assert parameters["map_frame"] == "map"
    assert parameters["base_frame"] == "base_footprint"
    assert parameters["scan_topic"] == "/scan"
    assert parameters["mode"] == "mapping"
    assert parameters["minimum_travel_distance"] <= 0.15
    assert parameters["minimum_travel_heading"] <= 0.15
    assert parameters["check_min_dist_and_heading_precisely"] is True
    assert parameters["do_loop_closing"] is True
    assert parameters["ceres_loss_function"] == "CauchyLoss"
    assert parameters["loop_match_minimum_chain_size"] >= 10
    assert parameters["loop_match_minimum_response_coarse"] >= 0.35
    assert parameters["loop_match_minimum_response_fine"] >= 0.45
    assert parameters["loop_search_maximum_distance"] <= 2.5
    assert parameters["loop_match_maximum_variance_coarse"] <= 2.0
    assert parameters["map_update_interval"] == pytest.approx(2.0)
    assert parameters["minimum_time_interval"] == pytest.approx(0.20)
    assert parameters["max_laser_range"] == pytest.approx(3.5)


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("minimum_travel_distance", 0.50),
        ("minimum_travel_heading", 0.50),
        ("minimum_travel_distance", 0.0),
        ("minimum_travel_distance", True),
        ("minimum_travel_heading", float("nan")),
        ("minimum_travel_heading", float("inf")),
    ],
)
def test_frontier_slam_profile_rejects_motion_threshold_regression(key, value):
    script = ROOT / "scripts" / "prepare_frontier_nav2_params.py"
    spec = importlib.util.spec_from_file_location("frontier_nav2_params", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    parameters = {
        "odom_frame": "odom",
        "map_frame": "map",
        "base_frame": "base_footprint",
        "scan_topic": "/scan",
        "mode": "mapping",
        "use_sim_time": True,
        "check_min_dist_and_heading_precisely": True,
        "minimum_travel_distance": 0.15,
        "minimum_travel_heading": 0.15,
        "do_loop_closing": True,
        "ceres_loss_function": "CauchyLoss",
        "loop_match_minimum_chain_size": 12,
        "loop_match_minimum_response_coarse": 0.45,
        "loop_match_minimum_response_fine": 0.55,
        "loop_search_maximum_distance": 2.5,
        "loop_match_maximum_variance_coarse": 2.0,
    }
    parameters[key] = value

    with pytest.raises(ValueError, match=key):
        module.with_slam_toolbox_profile(
            {}, {"slam_toolbox": {"ros__parameters": parameters}}
        )


@pytest.mark.parametrize("prior", ["map_start_pose", "map_file_name", "places"])
def test_frontier_slam_profile_rejects_scenario_priors(prior):
    script = ROOT / "scripts" / "prepare_frontier_nav2_params.py"
    spec = importlib.util.spec_from_file_location("frontier_nav2_params", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    parameters = {
        "odom_frame": "odom",
        "map_frame": "map",
        "base_frame": "base_footprint",
        "scan_topic": "/scan",
        "mode": "mapping",
        "use_sim_time": True,
        "check_min_dist_and_heading_precisely": True,
        "minimum_travel_distance": 0.15,
        "minimum_travel_heading": 0.15,
        "do_loop_closing": True,
        "ceres_loss_function": "CauchyLoss",
        "loop_match_minimum_chain_size": 12,
        "loop_match_minimum_response_coarse": 0.45,
        "loop_match_minimum_response_fine": 0.55,
        "loop_search_maximum_distance": 2.5,
        "loop_match_maximum_variance_coarse": 2.0,
        prior: [1.0, 2.0],
    }

    with pytest.raises(ValueError, match="scenario prior"):
        module.with_slam_toolbox_profile(
            {}, {"slam_toolbox": {"ros__parameters": parameters}}
        )


@pytest.mark.parametrize(
    ("key", "value"),
    [
        ("ceres_loss_function", "None"),
        ("do_loop_closing", False),
        ("loop_match_minimum_chain_size", 8),
        ("loop_match_minimum_response_coarse", 0.30),
        ("loop_match_minimum_response_fine", 0.40),
        ("loop_search_maximum_distance", 3.0),
        ("loop_match_maximum_variance_coarse", 3.0),
    ],
)
def test_frontier_slam_profile_rejects_non_robust_loop_contract(key, value):
    script = ROOT / "scripts" / "prepare_frontier_nav2_params.py"
    spec = importlib.util.spec_from_file_location("frontier_nav2_params", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    parameters = {
        "odom_frame": "odom",
        "map_frame": "map",
        "base_frame": "base_footprint",
        "scan_topic": "/scan",
        "mode": "mapping",
        "use_sim_time": True,
        "check_min_dist_and_heading_precisely": True,
        "minimum_travel_distance": 0.15,
        "minimum_travel_heading": 0.15,
        "do_loop_closing": True,
        "ceres_loss_function": "CauchyLoss",
        "loop_match_minimum_chain_size": 12,
        "loop_match_minimum_response_coarse": 0.45,
        "loop_match_minimum_response_fine": 0.55,
        "loop_search_maximum_distance": 2.5,
        "loop_match_maximum_variance_coarse": 2.0,
    }
    parameters[key] = value

    with pytest.raises(ValueError, match=key):
        module.with_slam_toolbox_profile(
            {}, {"slam_toolbox": {"ros__parameters": parameters}}
        )


@pytest.mark.parametrize(
    ("arguments", "expected"),
    [
        ((), 0.08),
        (("--xy-goal-tolerance", "0.20"), 0.20),
        (("--xy-goal-tolerance", "0.30"), 0.30),
    ],
)
def test_frontier_nav2_params_cli_preserves_known_default_and_allows_unknown_profile(
    tmp_path,
    arguments,
    expected,
):
    script = ROOT / "scripts" / "prepare_frontier_nav2_params.py"
    base = tmp_path / "nav2.yaml"
    output = tmp_path / "frontier.yaml"
    base.write_text(
        yaml.safe_dump(_frontier_nav2_source()),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(script),
            "--base",
            str(base),
            "--output",
            str(output),
            *arguments,
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode == 0, completed.stderr
    payload = yaml.safe_load(output.read_text(encoding="utf-8"))
    assert payload["controller_server"]["ros__parameters"][
        "general_goal_checker"
    ]["xy_goal_tolerance"] == expected
    assert payload["planner_server"]["ros__parameters"]["GridBased"][
        "allow_unknown"
    ] is False
    slam = payload["slam_toolbox"]["ros__parameters"]
    assert slam["minimum_travel_distance"] == pytest.approx(0.15)
    assert slam["minimum_travel_heading"] == pytest.approx(0.15)


def test_frontier_nav2_params_cli_rejects_tolerance_above_safety_bound(tmp_path):
    script = ROOT / "scripts" / "prepare_frontier_nav2_params.py"
    base = tmp_path / "nav2.yaml"
    output = tmp_path / "frontier.yaml"
    base.write_text(
        yaml.safe_dump(_frontier_nav2_source()),
        encoding="utf-8",
    )

    completed = subprocess.run(
        [
            sys.executable,
            str(script),
            "--base",
            str(base),
            "--output",
            str(output),
            "--xy-goal-tolerance",
            "0.31",
        ],
        capture_output=True,
        text=True,
        check=False,
    )

    assert completed.returncode != 0
    assert "[0.02, 0.30]" in completed.stderr


def test_showcase_passes_profile_frontier_tolerance_to_param_generator():
    showcase = (ROOT / "scripts" / "voice_slam_nav_showcase.sh").read_text(
        encoding="utf-8"
    )

    assert (
        'FRONTIER_XY_GOAL_TOLERANCE="${FRONTIER_XY_GOAL_TOLERANCE:-0.20}"'
        in showcase
    )
    assert (
        'FRONTIER_XY_GOAL_TOLERANCE="${FRONTIER_XY_GOAL_TOLERANCE:-0.08}"'
        in showcase
    )
    assert '--xy-goal-tolerance "$FRONTIER_XY_GOAL_TOLERANCE"' in showcase
    assert '--slam-params-source "$FRONTIER_SLAM_PARAMS"' in showcase


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
    frontier = yaml.safe_load(exploration_config.read_text(encoding="utf-8"))
    frontier_parameters = frontier["/**"]["ros__parameters"]
    assert frontier_parameters["progress_timeout"] == 45.0
    # frontier clearance 侵蚀整条 traversal 连通域；navigation 的 0.40m
    # goal clearance 只约束终点，两者不能共用同一个阈值。
    assert frontier_parameters["frontier_approach_clearance"] == 0.33
    assert frontier_parameters["frontier_approach_max_distance"] == 1.5
    assert frontier_parameters["frontier_robot_anchor_max_distance"] == 0.5
    assert frontier_parameters["frontier_approach_reached_tolerance"] == 0.40
    # unknown-world profile 的“接近即完成”必须仍在本机 costmap 激光清障证据内；
    # 它不能被日后调成跨房间的捷径。
    assert (
        frontier_parameters["frontier_approach_max_distance"]
        + frontier_parameters["frontier_approach_reached_tolerance"]
        <= 3.0
    )
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
        ROOT / "tools" / "acceptance" / "probes" / "slam_nav" / "slam_mapping_baseline.py",
        ROOT / "tools" / "acceptance" / "probes" / "slam_nav" / "lidar_loop_runtime.py",
        ROOT / "scripts" / "smoke_test_lidar_loop_runtime.sh",
        package / "launch" / "localization_navigation.launch.py",
        ROOT / "scripts" / "smoke_test_slam_localization_navigation.sh",
        ROOT
        / "tools"
        / "acceptance"
        / "probes"
        / "slam_nav"
        / "slam_localization_navigation.py",
        ROOT / "tools" / "evaluation" / "evaluate_slam_trajectory.py",
        ROOT / "tools" / "evaluation" / "extract_rosbag_trajectory.py",
        ROOT / "tools" / "evaluation" / "setup_openloris_groundtruth.py",
        ROOT / "tools" / "evaluation" / "compare_gtsam_switchable_sequences.py",
        ROOT / "tools" / "evaluation" / "compare_lidar_sequence_ablation.py",
        ROOT / "docs" / "evidence" / "slam" / "gtsam_switchable_multisequence.json",
        ROOT / "docs" / "evidence" / "slam" / "lidar_sequence_ablation_multisequence.json",
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
        ROOT / "tools" / "acceptance" / "probes" / "slam_nav" / "slam_mapping_baseline.py"
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
