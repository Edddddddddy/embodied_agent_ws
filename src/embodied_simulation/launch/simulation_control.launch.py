import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("embodied_simulation"),
        "config",
        "simulation_control.yaml",
    )
    use_typed_actions = LaunchConfiguration("use_typed_actions")
    use_behavior_tree = LaunchConfiguration("use_behavior_tree")
    executor_plugin = LaunchConfiguration("executor_plugin")
    autostart = LaunchConfiguration("autostart")
    lifecycle_manager_enabled = LaunchConfiguration(
        "lifecycle_manager_enabled"
    )
    use_composition = LaunchConfiguration("use_composition")
    namespace = LaunchConfiguration("namespace")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")
    readiness_required_components = LaunchConfiguration(
        "readiness_required_components"
    )
    node_parameters = [
        LaunchConfiguration("config"),
        {
            "use_sim_time": ParameterValue(
                LaunchConfiguration("use_sim_time"), value_type=bool
            ),
            "use_behavior_tree": ParameterValue(
                use_behavior_tree, value_type=bool
            ),
            "executor_plugin": executor_plugin,
            "action_timeout_s": ParameterValue(
                LaunchConfiguration("action_timeout_s"), value_type=float
            ),
        },
    ]
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("use_typed_actions", default_value="true"),
        DeclareLaunchArgument("use_behavior_tree", default_value="true"),
        DeclareLaunchArgument(
            "executor_plugin",
            default_value="embodied_simulation/GazeboRobotExecutor",
        ),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument(
            "lifecycle_manager_enabled",
            default_value="true",
            description=(
                "Disable when an external session orchestrator owns lifecycle "
                "transitions."
            ),
        ),
        DeclareLaunchArgument("action_timeout_s", default_value="12.0"),
        DeclareLaunchArgument("use_composition", default_value="false"),
        DeclareLaunchArgument("namespace", default_value=""),
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="cmd_vel",
            description=(
                "Gazebo/manual executor velocity output. Keep the publisher "
                "relative in C++ and remap it at the composition boundary."
            ),
        ),
        DeclareLaunchArgument("readiness_profile", default_value="execution"),
        DeclareLaunchArgument("readiness_stale_timeout_s", default_value="3.0"),
        DeclareLaunchArgument(
            "readiness_required_components",
            default_value="simulation_control,typed_action_bridge",
        ),
        LifecycleNode(
            package="embodied_simulation",
            executable="simulation_control_node",
            name="simulation_control",
            namespace=namespace,
            output="screen",
            parameters=node_parameters,
            # 只重映射手动 executor 的相对速度出口；其它状态/Action 接口保持稳定。
            remappings=[("cmd_vel", cmd_vel_topic)],
            condition=UnlessCondition(use_composition),
        ),
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            name="simulation_container",
            namespace=namespace,
            output="screen",
            composable_node_descriptions=[
                ComposableNode(
                    package="embodied_simulation",
                    plugin="embodied_simulation::SimulationControlNode",
                    name="simulation_control",
                    namespace=namespace,
                    parameters=node_parameters,
                    remappings=[("cmd_vel", cmd_vel_topic)],
                ),
            ],
            condition=IfCondition(use_composition),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="simulation_lifecycle_manager",
            namespace=namespace,
            output="screen",
            parameters=[{
                "autostart": ParameterValue(autostart, value_type=bool),
                "node_names": ["simulation_control"],
                "bond_timeout": 0.0,
            }],
            condition=IfCondition(lifecycle_manager_enabled),
        ),
        LifecycleNode(
            package="embodied_agent_cpp",
            executable="typed_action_bridge",
            name="typed_action_bridge",
            namespace=namespace,
            output="screen",
            condition=IfCondition(use_typed_actions),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="typed_action_bridge_lifecycle_manager",
            namespace=namespace,
            output="screen",
            parameters=[{
                "autostart": ParameterValue(autostart, value_type=bool),
                "node_names": ["typed_action_bridge"],
                "bond_timeout": 0.0,
            }],
            condition=IfCondition(use_typed_actions),
        ),
        Node(
            package="embodied_agent_middleware",
            executable="system_readiness_node",
            name="system_readiness",
            namespace=namespace,
            output="screen",
            parameters=[{
                "profile": LaunchConfiguration("readiness_profile"),
                "required_components_csv": readiness_required_components,
                "stale_timeout_s": ParameterValue(
                    LaunchConfiguration("readiness_stale_timeout_s"),
                    value_type=float,
                ),
            }],
            condition=IfCondition(use_typed_actions),
        ),
    ])
