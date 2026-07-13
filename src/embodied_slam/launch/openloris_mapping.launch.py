import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    slam_share = get_package_share_directory("embodied_slam")
    slam_toolbox_share = get_package_share_directory("slam_toolbox")
    default_params = os.path.join(
        slam_share, "config", "openloris_mapping_ceres.yaml"
    )

    bag_path = LaunchConfiguration("bag_path")
    params_file = LaunchConfiguration("params_file")
    output_path = LaunchConfiguration("output_path")
    replay_rate = LaunchConfiguration("replay_rate")
    startup_delay_s = LaunchConfiguration("startup_delay_s")
    use_rviz = LaunchConfiguration("use_rviz")

    recorder = Node(
        package="embodied_slam_tools",
        executable="slam_trajectory_recorder",
        name="openloris_trajectory_recorder",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "output_path": output_path,
                "map_frame": "map",
                "odom_frame": "dataset_odom",
                "odom_topic": "/openloris/odom",
            }
        ],
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
    replay = Node(
        package="embodied_slam_tools",
        executable="openloris_rosbag_replay",
        name="openloris_rosbag_replay",
        output="screen",
        parameters=[
            {
                "bag_path": bag_path,
                "rate": replay_rate,
                "startup_delay_s": startup_delay_s,
                "frame_prefix": "dataset",
            }
        ],
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=[
            "-d",
            os.path.join(slam_toolbox_share, "config", "slam_toolbox_default.rviz"),
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
        output="screen",
    )
    stop_after_replay = RegisterEventHandler(
        OnProcessExit(
            target_action=replay,
            # recorder 每个样本都会 flush；replay 正常退出后统一关闭其余长驻节点。
            on_exit=[EmitEvent(event=Shutdown(reason="OpenLORIS replay completed"))],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("bag_path", description="ROS 1 .bag or ROS 2 bag directory"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument(
                "output_path", default_value="logs/openloris_slam_estimate.tum"
            ),
            DeclareLaunchArgument("replay_rate", default_value="1.0"),
            DeclareLaunchArgument("startup_delay_s", default_value="5.0"),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            recorder,
            slam_toolbox,
            replay,
            rviz,
            stop_after_replay,
        ]
    )
