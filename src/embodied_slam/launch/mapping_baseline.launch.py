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
from launch.substitutions import LaunchConfiguration
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
    )
    # 候选检索与几何验证都作为 shadow 旁路运行；即使 ICP 通过也不直接修改
    # Ceres/GTSAM 位姿图，因此不会改变现有建图基线或污染基准实验。
    lidar_loop_candidates = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(slam_share, "launch", "lidar_loop_candidate.launch.py")
        ),
        launch_arguments={
            "use_sim_time": "true",
            "autostart": "true",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_loop_candidate_shadow")),
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
        condition=IfCondition(LaunchConfiguration("enable_loop_candidate_shadow")),
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
            lidar_loop_candidates,
            lidar_loop_verifier,
            closed_loop_driver,
            rviz,
        ]
    )
