"""Agent 安全控制与硬件 Adapter 的共享 launch 契约。"""

from __future__ import annotations

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue

from .agent_launch_contract import LaunchArgumentSpec, launch_default


AGENT_DEPLOYMENT_ARGUMENTS = (
    LaunchArgumentSpec(
        "authority_gate_enabled",
        bool,
        "Require fresh AUTONOMY authority before ActionGuard admits motion.",
    ),
    LaunchArgumentSpec("hardware_backend", str, "Hardware adapter backend."),
    LaunchArgumentSpec("hardware_enabled", bool, "Start hardware adapter."),
    LaunchArgumentSpec("lifecycle_autostart", bool, "Autostart managed nodes."),
    LaunchArgumentSpec("uart_device", str, "UART device path."),
    LaunchArgumentSpec("uart_baud_rate", int, "UART baud rate."),
    LaunchArgumentSpec("spi_device", str, "SPI device path."),
    LaunchArgumentSpec("spi_speed_hz", int, "SPI clock speed."),
)

AGENT_DEPLOYMENT_DEFAULTS = {
    "authority_gate_enabled": False,
    "hardware_backend": "mock",
    "hardware_enabled": True,
    "lifecycle_autostart": True,
    "uart_device": "/dev/ttyUSB0",
    "uart_baud_rate": 115200,
    "spi_device": "/dev/spidev0.0",
    "spi_speed_hz": 1000000,
}


def declare_agent_deployment_arguments():
    return [
        DeclareLaunchArgument(
            spec.name,
            default_value=launch_default(AGENT_DEPLOYMENT_DEFAULTS[spec.name]),
            description=spec.description,
        )
        for spec in AGENT_DEPLOYMENT_ARGUMENTS
    ]


def agent_deployment_nodes(agent_name: str):
    """构造安全边界、唯一 Lifecycle manager 与可选硬件 Adapter。"""
    c = {spec.name: LaunchConfiguration(spec.name) for spec in AGENT_DEPLOYMENT_ARGUMENTS}
    # node_names 的顺序就是启动顺序；manager 会按逆序停机，确保 Agent 先停止产生命令。
    managed_nodes = ["action_guard", agent_name]
    return [
        LifecycleNode(
            package="embodied_agent_cpp", executable="action_guard",
            name="action_guard", namespace="", output="screen",
            parameters=[{
                # 旧部署默认关闭；统一多控制源演示显式启用，避免在没有
                # ControlAuthorityManager 的场景中把现有动作链路全部拒绝。
                "authority_gate_enabled": ParameterValue(
                    c["authority_gate_enabled"], value_type=bool
                ),
            }],
        ),
        Node(
            package="nav2_lifecycle_manager", executable="lifecycle_manager",
            name="agent_control_lifecycle_manager", output="screen",
            parameters=[{
                "autostart": ParameterValue(c["lifecycle_autostart"], value_type=bool),
                "node_names": managed_nodes,
                "bond_timeout": 0.0,
            }],
        ),
        Node(
            package="embodied_agent_cpp", executable="hardware_controller",
            name="hardware_controller", output="screen",
            condition=IfCondition(c["hardware_enabled"]),
            parameters=[{
                "backend": c["hardware_backend"],
                "uart_device": c["uart_device"],
                "uart_baud_rate": ParameterValue(c["uart_baud_rate"], value_type=int),
                "spi_device": c["spi_device"],
                "spi_speed_hz": ParameterValue(c["spi_speed_hz"], value_type=int),
            }],
        ),
    ]


def control_authority_manager_node(
    *,
    condition,
    state_heartbeat_ms,
    use_sim_time=True,
    bootstrap_quiescence_acknowledged=False,
):
    """构造会话级控制权 manager，具体所有权由顶层 launch 决定。

    manager 不属于 online/offline Agent，也不属于 mapping/navigation 任一阶段。
    把节点定义集中在 bringup 契约中，可让独立启动保持自包含，同时允许完整
    showcase 把 manager 提升到会话根，避免阶段切换时生成新的 manager_epoch。
    """

    return Node(
        package="embodied_agent_cpp",
        executable="control_authority",
        name="control_authority",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "state_heartbeat_ms": ParameterValue(
                    state_heartbeat_ms, value_type=int
                ),
                # 默认 fail-closed：只有会话编排器收齐旧自动任务 terminal、
                # priority STOP 与新鲜零速证据后，才发送同代 typed ACK。
                "bootstrap_quiescence_acknowledged": (
                    bootstrap_quiescence_acknowledged
                ),
            }
        ],
        condition=condition,
    )
