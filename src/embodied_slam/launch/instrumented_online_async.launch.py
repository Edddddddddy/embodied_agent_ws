import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import AndSubstitution, LaunchConfiguration, NotSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """启动带回环前端诊断的 slam_toolbox，公开接口与上游 async launch 保持一致。"""
    slam_share = get_package_share_directory("embodied_slam")
    default_params = os.path.join(
        slam_share, "config", "openloris_mapping_ceres.yaml"
    )

    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("slam_params_file")

    slam_node = LifecycleNode(
        package="embodied_slam",
        executable="instrumented_async_slam_toolbox_node",
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            params_file,
            {
                "use_lifecycle_manager": use_lifecycle_manager,
                "use_sim_time": use_sim_time,
            },
        ],
    )

    configure = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(slam_node),
            transition_id=Transition.TRANSITION_CONFIGURE,
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )
    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=slam_node,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                LogInfo(msg="Activating instrumented slam_toolbox"),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(slam_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        ),
        condition=IfCondition(
            AndSubstitution(autostart, NotSubstitution(use_lifecycle_manager))
        ),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_lifecycle_manager", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("slam_params_file", default_value=default_params),
            slam_node,
            configure,
            activate,
        ]
    )
