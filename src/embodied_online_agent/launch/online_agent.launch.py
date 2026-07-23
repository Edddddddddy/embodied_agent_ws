import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from embodied_agent_bringup.agent_deployment_launch_contract import (
    agent_deployment_nodes,
    declare_agent_deployment_arguments,
)
from embodied_agent_bringup.agent_launch_contract import (
    agent_control_configurations,
    agent_control_parameter_overrides,
    declare_agent_control_arguments,
)
from embodied_agent_bringup.voice_frontend_launch_contract import (
    declare_voice_frontend_arguments,
    voice_frontend_nodes,
)


def generate_launch_description():
    config = LaunchConfiguration("config")
    agent_config = agent_control_configurations()
    microphone_enabled = agent_config["microphone_enabled"]
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=os.path.join(
                    get_package_share_directory("embodied_online_agent"),
                    "config",
                    "online_agent.yaml",
                ),
                description="Online provider YAML profile.",
            ),
            # 节点 schema 是默认值的单一权威来源；launch 只负责部署期显式覆盖。
            *declare_agent_control_arguments("online"),
            *declare_voice_frontend_arguments(microphone_enabled),
            *declare_agent_deployment_arguments(),
            LifecycleNode(
                package="embodied_online_agent",
                executable="online_agent",
                name="online_agent",
                namespace="",
                output="screen",
                parameters=[
                    config,
                    agent_control_parameter_overrides(),
                    {"agent_lifecycle_autostart": False},
                ],
            ),
            *voice_frontend_nodes(config),
            *agent_deployment_nodes("online_agent"),
        ]
    )
