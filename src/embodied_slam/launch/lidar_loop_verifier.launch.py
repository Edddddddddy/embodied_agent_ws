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
                "odometry_topic": LaunchConfiguration("odometry_topic"),
                "candidate_topic": LaunchConfiguration("candidate_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "maximum_cached_scans": ParameterValue(
                    LaunchConfiguration("maximum_cached_scans"), value_type=int
                ),
                "maximum_cached_odometry": ParameterValue(
                    LaunchConfiguration("maximum_cached_odometry"), value_type=int
                ),
                "maximum_odometry_time_delta_ms": ParameterValue(
                    LaunchConfiguration("maximum_odometry_time_delta_ms"),
                    value_type=float,
                ),
                "matching_mode": LaunchConfiguration("matching_mode"),
                "pending_timeout_ms": ParameterValue(
                    LaunchConfiguration("pending_timeout_ms"), value_type=float
                ),
                "point_stride": ParameterValue(
                    LaunchConfiguration("point_stride"), value_type=int
                ),
                "minimum_points": ParameterValue(
                    LaunchConfiguration("minimum_points"), value_type=int
                ),
                "submap_half_window_scans": ParameterValue(
                    LaunchConfiguration("submap_half_window_scans"), value_type=int
                ),
                "submap_maximum_time_delta_s": ParameterValue(
                    LaunchConfiguration("submap_maximum_time_delta_s"),
                    value_type=float,
                ),
                "submap_point_stride": ParameterValue(
                    LaunchConfiguration("submap_point_stride"), value_type=int
                ),
                "submap_minimum_contributing_scans": ParameterValue(
                    LaunchConfiguration("submap_minimum_contributing_scans"),
                    value_type=int,
                ),
                "submap_minimum_points": ParameterValue(
                    LaunchConfiguration("submap_minimum_points"), value_type=int
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
            DeclareLaunchArgument("odometry_topic", default_value="/odom"),
            DeclareLaunchArgument(
                "candidate_topic", default_value="/slam/loop_candidates"
            ),
            DeclareLaunchArgument(
                "output_topic", default_value="/slam/loop_verifications"
            ),
            DeclareLaunchArgument("maximum_cached_scans", default_value="12000"),
            DeclareLaunchArgument("maximum_cached_odometry", default_value="12000"),
            DeclareLaunchArgument(
                "maximum_odometry_time_delta_ms", default_value="50.0"
            ),
            DeclareLaunchArgument("matching_mode", default_value="scan_to_submap"),
            DeclareLaunchArgument("pending_timeout_ms", default_value="500.0"),
            DeclareLaunchArgument("point_stride", default_value="2"),
            DeclareLaunchArgument("minimum_points", default_value="30"),
            DeclareLaunchArgument("submap_half_window_scans", default_value="1"),
            DeclareLaunchArgument(
                "submap_maximum_time_delta_s", default_value="0.75"
            ),
            DeclareLaunchArgument("submap_point_stride", default_value="2"),
            DeclareLaunchArgument(
                "submap_minimum_contributing_scans", default_value="2"
            ),
            DeclareLaunchArgument("submap_minimum_points", default_value="60"),
            node,
            activate,
            configure,
        ]
    )
