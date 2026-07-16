"""Optional end-to-end check for the ROS 1/2 bag Adapter."""

from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


rosbags = pytest.importorskip("rosbags")
np = pytest.importorskip("numpy")

from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore


ROOT = Path(__file__).resolve().parents[3]
SPEC = importlib.util.spec_from_file_location(
    "extract_rosbag_trajectory", ROOT / "tools" / "evaluation" / "extract_rosbag_trajectory.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _odometry(types, index: int):
    time = types["builtin_interfaces/msg/Time"](sec=index, nanosec=0)
    header = types["std_msgs/msg/Header"](stamp=time, frame_id="odom")
    point = types["geometry_msgs/msg/Point"](x=float(index), y=0.0, z=0.0)
    quaternion = types["geometry_msgs/msg/Quaternion"](x=0.0, y=0.0, z=0.0, w=1.0)
    pose = types["geometry_msgs/msg/Pose"](position=point, orientation=quaternion)
    pose_covariance = types["geometry_msgs/msg/PoseWithCovariance"](
        pose=pose, covariance=np.zeros(36, dtype=np.float64)
    )
    vector = types["geometry_msgs/msg/Vector3"](x=0.0, y=0.0, z=0.0)
    twist = types["geometry_msgs/msg/Twist"](linear=vector, angular=vector)
    twist_covariance = types["geometry_msgs/msg/TwistWithCovariance"](
        twist=twist, covariance=np.zeros(36, dtype=np.float64)
    )
    return types["nav_msgs/msg/Odometry"](
        header=header,
        child_frame_id="base_link",
        pose=pose_covariance,
        twist=twist_covariance,
    )


def test_rosbag2_odometry_is_exported_to_tum(tmp_path):
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    bag = tmp_path / "odometry_bag"
    with Writer(bag, version=9) as writer:
        connection = writer.add_connection(
            "/odom", "nav_msgs/msg/Odometry", typestore=typestore
        )
        for index in range(1, 4):
            message = _odometry(typestore.types, index)
            writer.write(
                connection,
                index * 1_000_000_000,
                typestore.serialize_cdr(message, "nav_msgs/msg/Odometry"),
            )
    output = tmp_path / "odometry.tum"
    assert MODULE.extract(bag, "/odom", output) == 3
    rows = [line for line in output.read_text(encoding="utf-8").splitlines() if not line.startswith("#")]
    assert len(rows) == 3
    assert rows[-1].split()[1] == "3.000000000"
