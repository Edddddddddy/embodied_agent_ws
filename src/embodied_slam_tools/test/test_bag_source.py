from types import SimpleNamespace

from embodied_slam_tools import bag_source


class FakeReader:
    start_time = 1_000_000_000
    end_time = 4_000_000_000
    connections = [
        SimpleNamespace(topic="/odom", msgtype="nav_msgs/msg/Odometry"),
        SimpleNamespace(topic="/scan", msgtype="sensor_msgs/msg/LaserScan"),
        SimpleNamespace(topic="/tf_static", msgtype="tf2_msgs/msg/TFMessage"),
    ]

    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return None


def test_inspection_checks_required_topics_and_static_extrinsics(monkeypatch, tmp_path):
    monkeypatch.setattr(bag_source, "_reader", lambda _path: FakeReader())
    report = bag_source.inspect_bag(tmp_path / "fixture")
    assert report["passed"] is True
    assert report["duration_s"] == 3.0
    assert report["checks"] == {
        "topic:/odom": True,
        "topic:/scan": True,
        "static_extrinsics": True,
    }
