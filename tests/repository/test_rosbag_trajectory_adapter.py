from __future__ import annotations

import importlib.util
from pathlib import Path
from types import SimpleNamespace as NS


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "extract_rosbag_trajectory", ROOT / "scripts" / "extract_rosbag_trajectory.py"
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _vector(x=0.0, y=0.0, z=0.0):
    return NS(x=x, y=y, z=z)


def _quaternion():
    return NS(x=0.0, y=0.0, z=0.0, w=1.0)


def _header(frame="map", sec=12, nanosec=500_000_000):
    return NS(frame_id=frame, stamp=NS(sec=sec, nanosec=nanosec))


def test_odometry_adapter_uses_message_timestamp_and_pose():
    message = NS(
        header=_header("odom"),
        pose=NS(pose=NS(position=_vector(1.0, 2.0, 0.0), orientation=_quaternion())),
    )
    rows = MODULE.rows_from_message(message, "nav_msgs/msg/Odometry", 99)
    assert rows == [MODULE.TrajectoryRow(12.5, 1.0, 2.0, 0.0, 0.0, 0.0, 0.0, 1.0)]


def test_tf_adapter_filters_parent_and_child_frames():
    transforms = [
        NS(
            header=_header("map"),
            child_frame_id="base_link",
            transform=NS(translation=_vector(1.0), rotation=_quaternion()),
        ),
        NS(
            header=_header("odom"),
            child_frame_id="camera",
            transform=NS(translation=_vector(9.0), rotation=_quaternion()),
        ),
    ]
    rows = MODULE.rows_from_message(
        NS(transforms=transforms),
        "tf2_msgs/msg/TFMessage",
        99,
        parent_frame="/map",
        child_frame="/base_link",
    )
    assert len(rows) == 1
    assert rows[0].x == 1.0
