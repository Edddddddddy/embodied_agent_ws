"""Replay OpenLORIS ROS 1/2 bags as isolated ROS 2 SLAM input topics."""

from __future__ import annotations

import time
from pathlib import Path

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .bag_source import inspect_bag, iter_events
from .replay_core import FrameMapper, ReplayTimeline, TimestampDeduplicator, stamp_to_ns


def _copy_stamp(target, source) -> None:
    target.sec = int(getattr(source, "sec", getattr(source, "secs", 0)))
    target.nanosec = int(getattr(source, "nanosec", getattr(source, "nsecs", 0)))


def _copy_vector(target, source) -> None:
    target.x = float(source.x)
    target.y = float(source.y)
    target.z = float(source.z)


def _copy_quaternion(target, source) -> None:
    target.x = float(source.x)
    target.y = float(source.y)
    target.z = float(source.z)
    target.w = float(source.w)


class OpenLorisReplayNode(Node):
    def __init__(self) -> None:
        super().__init__("openloris_rosbag_replay")
        self._bag_path = Path(self.declare_parameter("bag_path", "").value)
        self._rate = float(self.declare_parameter("rate", 1.0).value)
        self._startup_delay_s = float(
            self.declare_parameter("startup_delay_s", 5.0).value
        )
        self._max_wall_gap_s = float(
            self.declare_parameter("max_wall_gap_s", 1.0).value
        )
        self._odom_topic = str(
            self.declare_parameter("output_odom_topic", "/openloris/odom").value
        )
        self._scan_topic = str(
            self.declare_parameter("output_scan_topic", "/openloris/scan").value
        )
        self._done_topic = str(
            self.declare_parameter("done_topic", "/openloris/replay_done").value
        )
        frame_prefix = str(self.declare_parameter("frame_prefix", "dataset").value)
        self._frames = FrameMapper(frame_prefix)
        if not self._bag_path.exists():
            raise ValueError(f"bag_path does not exist: {self._bag_path}")

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, sensor_qos)
        self._scan_pub = self.create_publisher(LaserScan, self._scan_topic, sensor_qos)
        self._clock_pub = self.create_publisher(Clock, "/clock", clock_qos)
        self._done_pub = self.create_publisher(Bool, self._done_topic, latched_qos)
        self._diagnostics_pub = self.create_publisher(
            DiagnosticArray, "/diagnostics", QoSProfile(depth=10)
        )
        self._tf = TransformBroadcaster(self)
        self._static_tf = StaticTransformBroadcaster(self)
        self._deduplicator = TimestampDeduplicator()
        self._counts = {"odom": 0, "scan": 0, "tf_static": 0, "duplicates": 0}

    def replay(self) -> None:
        contract = inspect_bag(self._bag_path)
        if not contract["passed"]:
            raise ValueError(f"bag contract failed: {contract['checks']}")
        self.get_logger().info(
            f"bag ready: duration={contract['duration_s']:.2f}s "
            f"rate={self._rate:.2f}x; waiting {self._startup_delay_s:.1f}s "
            "for subscribers"
        )
        # 用墙钟等待 discovery；此时尚未发布 /clock，不能使用 ROS timer。
        time.sleep(max(0.0, self._startup_delay_s))
        timeline = ReplayTimeline(self._rate, self._max_wall_gap_s)
        selected_topics = {"/odom", "/scan", "/tf_static"}
        for event in iter_events(self._bag_path, selected_topics):
            if not rclpy.ok():
                break
            clock_ns, delay_s = timeline.advance(event.timestamp_ns)
            if delay_s > 0.0:
                time.sleep(delay_s)
            self._publish_clock(clock_ns)
            if event.topic == "/odom":
                self._publish_odometry(event.message, event.timestamp_ns)
            elif event.topic == "/scan":
                self._publish_scan(event.message, event.timestamp_ns)
            elif event.topic == "/tf_static":
                self._publish_static_transforms(event.message)

        done = Bool()
        done.data = True
        self._done_pub.publish(done)
        self._publish_diagnostics(final=True)
        # transient-local done 已缓存，但保留短暂 discovery 窗口让 recorder 完成 flush。
        time.sleep(1.0)
        self.get_logger().info(f"replay complete: {self._counts}")

    def _publish_clock(self, stamp_ns: int) -> None:
        message = Clock()
        message.clock.sec = stamp_ns // 1_000_000_000
        message.clock.nanosec = stamp_ns % 1_000_000_000
        self._clock_pub.publish(message)

    def _publish_odometry(self, source, fallback_ns: int) -> None:
        stamp_ns = stamp_to_ns(source.header.stamp, fallback_ns)
        # OpenLORIS 已知旧 office bag 会重复 /odom；相同或倒退 stamp 只保留首帧。
        if not self._deduplicator.accept("odom", stamp_ns):
            self._counts["duplicates"] += 1
            return
        message = Odometry()
        _copy_stamp(message.header.stamp, source.header.stamp)
        message.header.frame_id = self._frames.map(source.header.frame_id or "base_odom")
        message.child_frame_id = self._frames.map(source.child_frame_id or "base_link")
        _copy_vector(message.pose.pose.position, source.pose.pose.position)
        _copy_quaternion(message.pose.pose.orientation, source.pose.pose.orientation)
        message.pose.covariance = list(source.pose.covariance)
        _copy_vector(message.twist.twist.linear, source.twist.twist.linear)
        _copy_vector(message.twist.twist.angular, source.twist.twist.angular)
        message.twist.covariance = list(source.twist.covariance)
        self._odom_pub.publish(message)

        # slam_toolbox 只消费 TF 与 LaserScan；不能假设发布 Odometry 就会自动出现 odom→base。
        transform = TransformStamped()
        transform.header = message.header
        transform.child_frame_id = message.child_frame_id
        _copy_vector(transform.transform.translation, message.pose.pose.position)
        _copy_quaternion(transform.transform.rotation, message.pose.pose.orientation)
        self._tf.sendTransform(transform)
        self._counts["odom"] += 1

    def _publish_scan(self, source, fallback_ns: int) -> None:
        stamp_ns = stamp_to_ns(source.header.stamp, fallback_ns)
        if not self._deduplicator.accept("scan", stamp_ns):
            self._counts["duplicates"] += 1
            return
        message = LaserScan()
        _copy_stamp(message.header.stamp, source.header.stamp)
        message.header.frame_id = self._frames.map(source.header.frame_id or "laser")
        for field in (
            "angle_min",
            "angle_max",
            "angle_increment",
            "time_increment",
            "scan_time",
            "range_min",
            "range_max",
        ):
            setattr(message, field, float(getattr(source, field)))
        message.ranges = list(source.ranges)
        message.intensities = list(source.intensities)
        self._scan_pub.publish(message)
        self._counts["scan"] += 1
        if self._counts["scan"] % 500 == 0:
            self._publish_diagnostics(final=False)

    def _publish_static_transforms(self, source) -> None:
        transforms: list[TransformStamped] = []
        for item in source.transforms:
            message = TransformStamped()
            _copy_stamp(message.header.stamp, item.header.stamp)
            message.header.frame_id = self._frames.map(item.header.frame_id)
            message.child_frame_id = self._frames.map(item.child_frame_id)
            _copy_vector(message.transform.translation, item.transform.translation)
            _copy_quaternion(message.transform.rotation, item.transform.rotation)
            if message.header.frame_id and message.child_frame_id:
                transforms.append(message)
        if transforms:
            self._static_tf.sendTransform(transforms)
            self._counts["tf_static"] += len(transforms)

    def _publish_diagnostics(self, final: bool) -> None:
        status = DiagnosticStatus()
        status.name = "embodied_slam/openloris_replay"
        status.hardware_id = str(self._bag_path)
        status.level = DiagnosticStatus.OK
        status.message = "completed" if final else "replaying"
        status.values = [
            KeyValue(key=name, value=str(value)) for name, value in self._counts.items()
        ]
        array = DiagnosticArray()
        array.status = [status]
        self._diagnostics_pub.publish(array)


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = OpenLorisReplayNode()
        node.replay()
    except Exception as exc:  # noqa: BLE001 - console entry must expose a non-zero failure.
        if node is not None:
            node.get_logger().fatal(f"replay failed: {exc}")
        raise
    finally:
        if node is not None:
            node.destroy_node()
        # SIGINT 可能已经由 rclpy signal handler 关闭 context，二次 shutdown 会抛 RCLError。
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
