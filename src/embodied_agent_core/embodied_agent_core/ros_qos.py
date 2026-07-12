"""项目级 ROS 2 QoS 语义，避免各节点散落魔法数字 depth=10。"""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def command_event_qos(depth: int = 50) -> QoSProfile:
    """命令生命周期事件不可静默丢失，使用 reliable FIFO 历史。"""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, int(depth)),
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def latched_state_qos(depth: int = 1) -> QoSProfile:
    """晚加入的监控应立即获得当前状态，而不必等待下一次状态变化。"""
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, int(depth)),
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def audio_stream_qos(depth: int = 20) -> QoSProfile:
    """实时 PCM 优先低延迟；允许丢弃过期帧，禁止可靠传输反压音频线程。"""

    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, int(depth)),
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )
