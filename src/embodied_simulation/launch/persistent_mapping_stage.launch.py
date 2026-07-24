"""持久会话的建图阶段：只装载 SLAM provider 与 Gazebo executor."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _include_launch(package: str, filename: str, arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
    )


def generate_launch_description():
    simulation_share = get_package_share_directory("embodied_simulation")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=os.path.join(
                    simulation_share,
                    "config",
                    "frontier_slam_toolbox.yaml",
                ),
            ),
            DeclareLaunchArgument("use_respawn", default_value="false"),
            DeclareLaunchArgument("action_timeout_s", default_value="180.0"),
            DeclareLaunchArgument(
                "readiness_stale_timeout_s",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "readiness_required_components",
                default_value=(
                    "audio_frontend,agent,action_guard,"
                    "typed_action_bridge,simulation_control"
                ),
                description=(
                    "Set to action_guard,typed_action_bridge,"
                    "simulation_control when base launch_agent=false."
                ),
            ),
            DeclareLaunchArgument(
                "simulation_control_config",
                default_value=os.path.join(
                    simulation_share,
                    "config",
                    "simulation_control.yaml",
                ),
            ),
            # 官方 slam_launch 同时提供 slam_toolbox 与 map_saver。base 中的
            # Nav2 common 已经运行，因此这里不得再次启动 bringup/navigation。
            _include_launch(
                "nav2_bringup",
                "slam_launch.py",
                {
                    "namespace": "",
                    "params_file": params_file,
                    "use_sim_time": "true",
                    "autostart": "true",
                    "use_respawn": LaunchConfiguration("use_respawn"),
                },
            ),
            _include_launch(
                "embodied_simulation",
                "simulation_control.launch.py",
                {
                    "config": LaunchConfiguration(
                        "simulation_control_config"
                    ),
                    "use_sim_time": "true",
                    # typed bridge 常驻在 base；stage 只替换具体 executor。
                    "use_typed_actions": "false",
                    "use_behavior_tree": "true",
                    "executor_plugin": (
                        "embodied_simulation/GazeboRobotExecutor"
                    ),
                    # lifecycle 由常驻 SessionOrchestrator 闭环推进，禁用
                    # stage 内第二个 manager，避免 service 响应超时后永久卡住。
                    "autostart": "false",
                    "lifecycle_manager_enabled": "false",
                    "action_timeout_s": LaunchConfiguration(
                        "action_timeout_s"
                    ),
                    "cmd_vel_topic": "/control/voice/cmd_vel",
                },
            ),
            # readiness 聚合器与 stage 共生共灭。这样切换后必须重新收到
            # navigation executor 的健康心跳，不能复用 mapping 的旧 ready。
            Node(
                package="embodied_agent_middleware",
                executable="system_readiness_node",
                name="system_readiness",
                output="screen",
                parameters=[
                    {
                        "profile": "persistent_mapping_stage",
                        "required_components_csv": LaunchConfiguration(
                            "readiness_required_components"
                        ),
                        "stale_timeout_s": ParameterValue(
                            LaunchConfiguration(
                                "readiness_stale_timeout_s"
                            ),
                            value_type=float,
                        ),
                    }
                ],
            ),
        ]
    )
