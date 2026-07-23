#!/usr/bin/env python3
"""Generate a small ROS 2 bag that follows the OpenLORIS SLAM topic contract."""

from __future__ import annotations

import argparse
import math
from pathlib import Path


def generate(output: Path, sample_count: int = 60) -> None:
    try:
        import numpy as np
        from rosbags.rosbag2 import Writer
        from rosbags.typesys import Stores, get_typestore
    except ImportError as exc:
        raise RuntimeError(
            "run: pip install -r requirements-slam-eval.txt"
        ) from exc
    if output.exists():
        raise FileExistsError(f"refusing to overwrite existing bag: {output}")
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    types = typestore.types

    def header(stamp_ns: int, frame: str):
        stamp = types["builtin_interfaces/msg/Time"](
            sec=stamp_ns // 1_000_000_000,
            nanosec=stamp_ns % 1_000_000_000,
        )
        return types["std_msgs/msg/Header"](stamp=stamp, frame_id=frame)

    def vector(x=0.0, y=0.0, z=0.0):
        return types["geometry_msgs/msg/Vector3"](x=x, y=y, z=z)

    def quaternion(yaw=0.0):
        return types["geometry_msgs/msg/Quaternion"](
            x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5)
        )

    def odometry(stamp_ns: int, index: int):
        point = types["geometry_msgs/msg/Point"](
            x=0.025 * index, y=0.08 * math.sin(index * 0.08), z=0.0
        )
        pose = types["geometry_msgs/msg/Pose"](
            position=point, orientation=quaternion(0.02 * math.sin(index * 0.05))
        )
        pose_covariance = types["geometry_msgs/msg/PoseWithCovariance"](
            pose=pose, covariance=np.zeros(36, dtype=np.float64)
        )
        twist = types["geometry_msgs/msg/Twist"](
            linear=vector(0.25, 0.0, 0.0), angular=vector()
        )
        twist_covariance = types["geometry_msgs/msg/TwistWithCovariance"](
            twist=twist, covariance=np.zeros(36, dtype=np.float64)
        )
        return types["nav_msgs/msg/Odometry"](
            header=header(stamp_ns, "base_odom"),
            child_frame_id="base_link",
            pose=pose_covariance,
            twist=twist_covariance,
        )

    def scan(stamp_ns: int, index: int):
        beam_count = 360
        ranges = np.full(beam_count, 4.0, dtype=np.float32)
        # 稳定的非对称几何比全常量量程更容易暴露 frame/scan 接线错误。
        ranges[40:80] = 1.5 + 0.02 * math.sin(index * 0.1)
        ranges[210:245] = 2.2
        return types["sensor_msgs/msg/LaserScan"](
            header=header(stamp_ns, "laser"),
            angle_min=-math.pi,
            angle_max=math.pi,
            angle_increment=2.0 * math.pi / beam_count,
            time_increment=0.0,
            scan_time=0.1,
            range_min=0.12,
            range_max=30.0,
            ranges=ranges,
            intensities=np.zeros(beam_count, dtype=np.float32),
        )

    start_ns = 100_000_000_000
    static_transform = types["geometry_msgs/msg/TransformStamped"](
        header=header(start_ns, "base_link"),
        child_frame_id="laser",
        transform=types["geometry_msgs/msg/Transform"](
            translation=vector(0.20, 0.0, 0.18), rotation=quaternion()
        ),
    )
    static_message = types["tf2_msgs/msg/TFMessage"](
        transforms=[static_transform]
    )
    with Writer(output, version=9) as writer:
        odom_connection = writer.add_connection(
            "/odom", "nav_msgs/msg/Odometry", typestore=typestore
        )
        scan_connection = writer.add_connection(
            "/scan", "sensor_msgs/msg/LaserScan", typestore=typestore
        )
        tf_connection = writer.add_connection(
            "/tf_static", "tf2_msgs/msg/TFMessage", typestore=typestore
        )
        writer.write(
            tf_connection,
            start_ns,
            typestore.serialize_cdr(static_message, "tf2_msgs/msg/TFMessage"),
        )
        for index in range(sample_count):
            stamp_ns = start_ns + index * 100_000_000
            odom_message = odometry(stamp_ns, index)
            scan_message = scan(stamp_ns + 10_000_000, index)
            writer.write(
                odom_connection,
                stamp_ns,
                typestore.serialize_cdr(odom_message, "nav_msgs/msg/Odometry"),
            )
            if index == 10:
                # 模拟 OpenLORIS 旧 office bag 的重复 odom stamp。
                writer.write(
                    odom_connection,
                    stamp_ns + 1,
                    typestore.serialize_cdr(odom_message, "nav_msgs/msg/Odometry"),
                )
            writer.write(
                scan_connection,
                stamp_ns + 10_000_000,
                typestore.serialize_cdr(scan_message, "sensor_msgs/msg/LaserScan"),
            )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sample-count", type=int, default=60)
    args = parser.parse_args()
    generate(args.output, args.sample_count)
    print(f"PASS: generated OpenLORIS-compatible fixture -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
