"""与 C++ middleware 对齐的项目级 ROS 2 QoS 语义。"""

from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _profile(
    depth: int,
    *,
    reliability: ReliabilityPolicy,
    durability: DurabilityPolicy = DurabilityPolicy.VOLATILE,
) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=max(1, int(depth)),
        reliability=reliability,
        durability=durability,
    )


def command_qos(depth: int = 50) -> QoSProfile:
    """控制命令可靠传输但不重放，避免节点重启后执行陈旧动作。"""
    return _profile(depth, reliability=ReliabilityPolicy.RELIABLE)


def event_qos(depth: int = 50) -> QoSProfile:
    """生命周期事件可靠、有序、volatile；事件不是可覆盖的当前状态。"""
    return _profile(depth, reliability=ReliabilityPolicy.RELIABLE)


def state_qos(depth: int = 1) -> QoSProfile:
    """晚加入的监控立即获得最新状态，但只保留很浅的历史。"""
    return _profile(
        depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


def sensor_qos(depth: int = 5) -> QoSProfile:
    """高频遥测允许丢旧帧，以最新数据和低延迟优先。"""
    return _profile(depth, reliability=ReliabilityPolicy.BEST_EFFORT)


def audio_qos(depth: int = 5) -> QoSProfile:
    """PCM 使用传感器语义，消费跟不上时丢旧帧而不是积压音频。"""
    return sensor_qos(depth)


def diagnostics_qos(depth: int = 10) -> QoSProfile:
    """低频诊断可靠传输，不把历史诊断误当成当前状态。"""
    return _profile(depth, reliability=ReliabilityPolicy.RELIABLE)
