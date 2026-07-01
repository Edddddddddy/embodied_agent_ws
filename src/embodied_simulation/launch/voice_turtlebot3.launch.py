import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def include_launch(package, filename, arguments=None, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", filename)
        ),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    os.environ["TURTLEBOT3_MODEL"] = "burger"
    simulation_share = get_package_share_directory("embodied_simulation")
    turtlebot_share = get_package_share_directory("turtlebot3_gazebo")
    world = os.path.join(turtlebot_share, "worlds", "turtlebot3_world.world")
    model = os.path.join(
        turtlebot_share, "models", "turtlebot3_burger", "model.sdf"
    )
    bridge = os.path.join(simulation_share, "config", "turtlebot3_bridge.yaml")
    control_config = os.path.join(
        simulation_share, "config", "simulation_control.yaml"
    )

    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    launch_agent = LaunchConfiguration("launch_agent")
    agent_type = LaunchConfiguration("agent_type")
    provider_mode = LaunchConfiguration("provider_mode")
    microphone = LaunchConfiguration("microphone_enabled")
    capture = LaunchConfiguration("capture_enabled")
    speaker = LaunchConfiguration("speaker_enabled")
    wake_word = LaunchConfiguration("wake_word_enabled")

    online_condition = IfCondition(
        PythonExpression([
            "'", launch_agent, "' == 'true' and '", agent_type, "' == 'online'"
        ])
    )
    offline_condition = IfCondition(
        PythonExpression([
            "'", launch_agent, "' == 'true' and '", agent_type, "' == 'offline'"
        ])
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("launch_agent", default_value="true"),
        DeclareLaunchArgument("agent_type", default_value="online"),
        DeclareLaunchArgument("provider_mode", default_value="mock"),
        DeclareLaunchArgument("microphone_enabled", default_value="false"),
        DeclareLaunchArgument("capture_enabled", default_value=microphone),
        DeclareLaunchArgument("speaker_enabled", default_value="false"),
        DeclareLaunchArgument("wake_word_enabled", default_value="true"),
        DeclareLaunchArgument("x_pose", default_value="-2.0"),
        DeclareLaunchArgument("y_pose", default_value="-0.5"),
        SetEnvironmentVariable("TURTLEBOT3_MODEL", "burger"),
        AppendEnvironmentVariable(
            "GZ_SIM_RESOURCE_PATH", os.path.join(turtlebot_share, "models")
        ),
        include_launch(
            "ros_gz_sim",
            "gz_sim.launch.py",
            {"gz_args": ["-r -s -v2 ", world], "on_exit_shutdown": "true"},
        ),
        include_launch(
            "ros_gz_sim",
            "gz_sim.launch.py",
            {"gz_args": "-g -v2 ", "on_exit_shutdown": "true"},
            IfCondition(gui),
        ),
        include_launch(
            "turtlebot3_gazebo",
            "robot_state_publisher.launch.py",
            {"use_sim_time": "true"},
        ),
        Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name", "burger", "-file", model,
                "-x", LaunchConfiguration("x_pose"),
                "-y", LaunchConfiguration("y_pose"), "-z", "0.01",
            ],
            output="screen",
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=["--ros-args", "-p", f"config_file:={bridge}"],
            output="screen",
        ),
        Node(
            package="embodied_simulation",
            executable="simulation_control_node",
            name="simulation_control",
            output="screen",
            parameters=[control_config, {"use_sim_time": True}],
        ),
        include_launch(
            "embodied_online_agent",
            "online_agent.launch.py",
            {
                "mode": provider_mode,
                "microphone_enabled": microphone,
                "capture_enabled": capture,
                "speaker_enabled": speaker,
                "hardware_enabled": "false",
            },
            online_condition,
        ),
        include_launch(
            "embodied_offline_agent",
            "offline_agent.launch.py",
            {
                "mode": provider_mode,
                "microphone_enabled": microphone,
                "capture_enabled": capture,
                "speaker_enabled": speaker,
                "wake_word_enabled": wake_word,
                "hardware_enabled": "false",
            },
            offline_condition,
        ),
        Node(
            package="embodied_agent_cpp",
            executable="action_guard",
            name="action_guard",
            output="screen",
            condition=UnlessCondition(launch_agent),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            output="screen",
            arguments=[
                "-d", os.path.join(turtlebot_share, "rviz", "tb3_gazebo.rviz")
            ],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(rviz),
        ),
    ])
