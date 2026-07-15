from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """启动候选几何验证旁路；通过结果仍不直接写入位姿图。"""
    autostart = LaunchConfiguration("autostart")
    node = LifecycleNode(
        package="embodied_slam",
        executable="lidar_loop_verifier_node",
        name="lidar_loop_verifier",
        namespace="",
        output="screen",
        parameters=[
            {
                "use_sim_time": ParameterValue(
                    LaunchConfiguration("use_sim_time"), value_type=bool
                ),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "candidate_topic": LaunchConfiguration("candidate_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "maximum_cached_scans": ParameterValue(
                    LaunchConfiguration("maximum_cached_scans"), value_type=int
                ),
                "point_stride": ParameterValue(
                    LaunchConfiguration("point_stride"), value_type=int
                ),
                "minimum_points": ParameterValue(
                    LaunchConfiguration("minimum_points"), value_type=int
                ),
            }
        ],
    )
    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(autostart),
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="Activating shadow LiDAR loop geometry verifier"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(autostart),
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("scan_topic", default_value="/scan"),
            DeclareLaunchArgument(
                "candidate_topic", default_value="/slam/loop_candidates"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/slam/loop_verifications"
            ),
            DeclareLaunchArgument("maximum_cached_scans", default_value="12000"),
            DeclareLaunchArgument("point_stride", default_value="2"),
            DeclareLaunchArgument("minimum_points", default_value="30"),
            node,
            activate,
            configure,
        ]
    )
