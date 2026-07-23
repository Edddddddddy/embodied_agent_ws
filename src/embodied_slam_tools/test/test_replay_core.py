import math

import pytest

from embodied_slam_tools.replay_core import (
    FrameMapper,
    Pose2,
    ReplayTimeline,
    TimestampDeduplicator,
    compose_pose,
    stamp_to_ns,
)


def test_frame_mapper_isolates_known_and_unknown_dataset_frames():
    mapper = FrameMapper("dataset")
    assert mapper.map("/base_odom") == "dataset_odom"
    assert mapper.map("base_link") == "dataset_base_link"
    assert mapper.map("laser") == "dataset_laser"
    assert mapper.map("d400/color") == "dataset_d400_color"
    assert mapper.map("dataset_laser") == "dataset_laser"


def test_timeline_scales_wall_delay_and_never_moves_clock_backwards():
    timeline = ReplayTimeline(rate=2.0, max_wall_gap_s=0.2)
    assert timeline.advance(1_000_000_000) == (1_000_000_000, 0.0)
    assert timeline.advance(1_100_000_000) == pytest.approx((1_100_000_000, 0.05))
    clock_ns, delay_s = timeline.advance(1_050_000_000)
    assert clock_ns == 1_100_000_000
    assert delay_s == 0.0
    assert timeline.advance(3_000_000_000)[1] == 0.2


def test_timestamp_deduplicator_rejects_duplicates_and_backward_frames_per_stream():
    deduplicator = TimestampDeduplicator()
    assert deduplicator.accept("odom", 100)
    assert not deduplicator.accept("odom", 100)
    assert not deduplicator.accept("odom", 99)
    assert deduplicator.accept("scan", 99)
    assert deduplicator.accept("odom", 101)


def test_pose_composition_keeps_map_to_odom_correction_and_robot_motion():
    result = compose_pose(Pose2(1.0, 2.0, math.pi / 2.0), Pose2(2.0, 0.0, 0.2))
    assert result.x == pytest.approx(1.0)
    assert result.y == pytest.approx(4.0)
    assert result.yaw == pytest.approx(math.pi / 2.0 + 0.2)


def test_stamp_adapter_accepts_ros1_and_ros2_field_names():
    ros2_stamp = type("Stamp", (), {"sec": 3, "nanosec": 4})()
    ros1_stamp = type("Stamp", (), {"secs": 5, "nsecs": 6})()
    assert stamp_to_ns(ros2_stamp, 0) == 3_000_000_004
    assert stamp_to_ns(ros1_stamp, 0) == 5_000_000_006
