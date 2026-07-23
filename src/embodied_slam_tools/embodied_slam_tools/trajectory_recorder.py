"""Record the corrected map-frame trajectory emitted during dataset replay."""

from __future__ import annotations

import math
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage

from .replay_core import Pose2, compose_pose, yaw_from_quaternion


def _stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _pose_from_odometry(message: Odometry) -> Pose2:
    pose = message.pose.pose
    return Pose2(
        pose.position.x,
        pose.position.y,
        yaw_from_quaternion(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ),
    )


class SlamTrajectoryRecorder(Node):
    def __init__(self) -> None:
        super().__init__("slam_trajectory_recorder")
        output_path = Path(
            self.declare_parameter(
                "output_path", "logs/openloris_slam_estimate.tum"
            ).value
        )
        self._map_frame = str(self.declare_parameter("map_frame", "map").value)
        self._odom_frame = str(
            self.declare_parameter("odom_frame", "dataset_odom").value
        )
        self._min_period_s = float(
            self.declare_parameter("min_period_s", 0.05).value
        )
        odom_topic = str(
            self.declare_parameter("odom_topic", "/openloris/odom").value
        )
        done_topic = str(
            self.declare_parameter("done_topic", "/openloris/replay_done").value
        )
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self._stream = output_path.open("w", encoding="utf-8")
        self._stream.write("# timestamp tx ty tz qx qy qz qw\n")
        self._latest_odom: tuple[float, Pose2] | None = None
        self._last_written_stamp = float("-inf")
        self._sample_count = 0

        sensor_qos = QoSProfile(depth=30, reliability=ReliabilityPolicy.BEST_EFFORT)
        tf_qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        done_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Odometry, odom_topic, self._on_odometry, sensor_qos)
        self.create_subscription(TFMessage, "/tf", self._on_tf, tf_qos)
        self.create_subscription(Bool, done_topic, self._on_done, done_qos)
        self.get_logger().info(f"trajectory recorder -> {output_path}")

    def _on_odometry(self, message: Odometry) -> None:
        self._latest_odom = (_stamp_seconds(message.header.stamp), _pose_from_odometry(message))

    def _on_tf(self, message: TFMessage) -> None:
        if self._latest_odom is None:
            return
        for transform in message.transforms:
            if (
                transform.header.frame_id == self._map_frame
                and transform.child_frame_id == self._odom_frame
            ):
                rotation = transform.transform.rotation
                map_to_odom = Pose2(
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    yaw_from_quaternion(
                        rotation.x, rotation.y, rotation.z, rotation.w
                    ),
                )
                stamp, odom_to_base = self._latest_odom
                # map→base 必须显式组合 map→odom 与 odom→base；只记录 map→odom 会漏掉机器人运动。
                self._record(stamp, compose_pose(map_to_odom, odom_to_base))
                break

    def _record(self, stamp: float, pose: Pose2) -> None:
        if stamp - self._last_written_stamp < self._min_period_s:
            return
        qz = math.sin(pose.yaw * 0.5)
        qw = math.cos(pose.yaw * 0.5)
        self._stream.write(
            f"{stamp:.9f} {pose.x:.9f} {pose.y:.9f} 0 0 0 {qz:.9f} {qw:.9f}\n"
        )
        self._stream.flush()
        self._last_written_stamp = stamp
        self._sample_count += 1

    def _on_done(self, message: Bool) -> None:
        if message.data:
            self._stream.flush()
            self.get_logger().info(
                f"replay done; recorded {self._sample_count} poses"
            )

    def destroy_node(self) -> bool:
        # launch 在回放结束后会向全部进程发送 SIGINT。这里保持资源释放幂等，
        # 避免“正常收尾”因重复 close/shutdown 被误报为节点崩溃。
        if not self._stream.closed:
            self._stream.flush()
            self._stream.close()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SlamTrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # launch 的 OnProcessExit 会用 SIGINT 结束 recorder；这是预期控制流，不应打印 traceback。
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
