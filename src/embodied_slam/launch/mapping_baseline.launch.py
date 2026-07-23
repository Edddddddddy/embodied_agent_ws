import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, NotSubstitution, OrSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory("embodied_slam")
    simulation_share = get_package_share_directory("embodied_simulation")
    tb3_share = get_package_share_directory("nav2_minimal_tb3_sim")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")
    default_world = os.path.join(simulation_share, "worlds", "slam_loop_demo.sdf.xacro")
    default_params = os.path.join(slam_share, "config", "slam_mapping_ceres.yaml")

    world = LaunchConfiguration("world")
    headless = LaunchConfiguration("headless")
    params_file = LaunchConfiguration("params_file")
    auto_drive = LaunchConfiguration("auto_drive")
    use_rviz = LaunchConfiguration("use_rviz")
    robot_name = LaunchConfiguration("robot_name")
    robot_sdf = LaunchConfiguration("robot_sdf")

    world_sdf = tempfile.mktemp(prefix="embodied_slam_", suffix=".sdf")
    render_world = ExecuteProcess(
        cmd=["xacro", "-o", world_sdf, ["headless:=", headless], world],
        output="screen",
    )
    gazebo_server = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_sdf], output="screen"
    )
    remove_temp_world = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                OpaqueFunction(
                    function=lambda _: os.path.exists(world_sdf) and os.remove(world_sdf)
                )
            ]
        )
    )

    robot_description_path = os.path.join(tb3_share, "urdf", "turtlebot3_waffle.urdf")
    with open(robot_description_path, "r", encoding="utf-8") as stream:
        robot_description = stream.read()

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tb3_share, "launch", "spawn_tb3.launch.py")
        ),
        launch_arguments={
            "namespace": "",
            "robot_name": robot_name,
            "robot_sdf": robot_sdf,
            "x_pose": LaunchConfiguration("x_pose"),
            "y_pose": LaunchConfiguration("y_pose"),
            "z_pose": "0.01",
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True, "robot_description": robot_description}],
    )
    drift_injector = Node(
        package="embodied_slam",
        executable="odom_drift_injector_node",
        name="odom_drift_injector",
        output="screen",
        parameters=[params_file],
    )
    closed_loop_driver = Node(
        package="embodied_slam",
        executable="closed_loop_driver_node",
        name="slam_closed_loop_driver",
        output="screen",
        parameters=[params_file],
        condition=IfCondition(auto_drive),
    )
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_toolbox_share, "launch", "online_async_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "slam_params_file": params_file,
        }.items(),
        condition=IfCondition(
            NotSubstitution(LaunchConfiguration("enable_loop_constraint_commit"))
        ),
    )
    instrumented_slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "instrumented_online_async.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "slam_params_file": params_file,
            "external_loop_constraint_enabled": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_loop_constraint_commit")),
    )
    # 默认链路只产生 shadow 决策，不修改位姿图；只有显式开启实验 flag 时，
    # gate 才请求 instrumented slam_toolbox 写入，避免低 precision 前端污染基线。
    lidar_loop_candidates = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "lidar_loop_candidate.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
        condition=IfCondition(
            OrSubstitution(
                LaunchConfiguration("enable_loop_candidate_shadow"),
                LaunchConfiguration("enable_loop_constraint_commit"),
            )
        ),
    )
    lidar_loop_verifier = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "lidar_loop_verifier.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            # 使用 SLAM 实际消费的漂移里程计，只借助短时相邻帧运动构建局部子图。
            "odometry_topic": "/slam/odom",
            "matching_mode": "scan_to_submap",
        }.items(),
        condition=IfCondition(
            OrSubstitution(
                LaunchConfiguration("enable_loop_candidate_shadow"),
                LaunchConfiguration("enable_loop_constraint_commit"),
            )
        ),
    )
    lidar_loop_constraint_gate = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "lidar_loop_constraint_gate.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
            "commit_enabled": LaunchConfiguration("enable_loop_constraint_commit"),
        }.items(),
        condition=IfCondition(
            OrSubstitution(
                LaunchConfiguration("enable_loop_candidate_shadow"),
                LaunchConfiguration("enable_loop_constraint_commit"),
            )
        ),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(slam_toolbox_share, "config", "slam_toolbox_default.rviz")],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("headless", default_value="true"),
            DeclareLaunchArgument("auto_drive", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument(
                "enable_loop_candidate_shadow", default_value="true"
            ),
            # 实验写图必须显式开启；默认只运行 typed shadow 审计链路。
            DeclareLaunchArgument(
                "enable_loop_constraint_commit", default_value="false"
            ),
            DeclareLaunchArgument("robot_name", default_value="turtlebot3_waffle"),
            DeclareLaunchArgument(
                "robot_sdf",
                default_value=os.path.join(tb3_share, "urdf", "gz_waffle.sdf.xacro"),
            ),
            DeclareLaunchArgument("x_pose", default_value="-1.40"),
            DeclareLaunchArgument("y_pose", default_value="-1.30"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            render_world,
            remove_temp_world,
            gazebo_server,
            spawn_robot,
            robot_state_publisher,
            drift_injector,
            slam_toolbox,
            instrumented_slam_toolbox,
            lidar_loop_candidates,
            lidar_loop_verifier,
            lidar_loop_constraint_gate,
            closed_loop_driver,
            rviz,
        ]
    )
