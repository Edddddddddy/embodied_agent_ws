import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    simulation_share = get_package_share_directory("embodied_simulation")
    navigation_share = get_package_share_directory("embodied_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")
    default_map = os.path.join(os.path.expanduser("~"), "embodied_agent_ws", "logs", "slam_ceres_map.yaml")
    default_world = os.path.join(simulation_share, "worlds", "slam_loop_demo.sdf.xacro")
    default_params = os.path.join(nav2_share, "params", "nav2_params.yaml")
    tracker_params = os.path.join(navigation_share, "config", "navigation_overrides.yaml")

    # 跟踪器独立于 costmap 插件运行：感知输入可以替换，Nav2 只消费稳定的 typed tracks。
    dynamic_obstacle_tracker = Node(
        package="embodied_navigation",
        executable="dynamic_obstacle_tracker_node",
        name="dynamic_obstacle_tracker",
        output="screen",
        parameters=[tracker_params, {"use_sim_time": True}],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, "launch", "tb3_simulation_launch.py")),
        launch_arguments={
            "slam": "False",
            "map": LaunchConfiguration("map"),
            "params_file": LaunchConfiguration("params_file"),
            "use_sim_time": "true",
            "autostart": "true",
            "use_composition": "True",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "headless": LaunchConfiguration("headless"),
            "world": LaunchConfiguration("world"),
            "x_pose": LaunchConfiguration("x_pose"),
            "y_pose": LaunchConfiguration("y_pose"),
            "yaw": LaunchConfiguration("yaw"),
        }.items(),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("world", default_value=default_world),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_rviz", default_value="False"),
            DeclareLaunchArgument("headless", default_value="True"),
            DeclareLaunchArgument("x_pose", default_value="-1.40"),
            DeclareLaunchArgument("y_pose", default_value="-1.30"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            dynamic_obstacle_tracker,
            nav2,
        ]
    )
