import importlib.util
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "simulation_readiness_check.py"
SPEC = importlib.util.spec_from_file_location("simulation_readiness_check", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
READINESS = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = READINESS
SPEC.loader.exec_module(READINESS)


def test_frozen_gazebo_clock_is_a_readiness_blocker():
    blockers = READINESS.readiness_blockers(
        odom_count=8,
        scan_count=3,
        odom_stable=True,
        cmd_vel_publishers=1,
        cmd_vel_subscribers=1,
        action_server_seen=True,
        clock_count=8,
        clock_span_s=0.0,
        min_odom_samples=5,
        min_scan_samples=1,
        min_clock_samples=3,
        min_clock_advance_s=0.1,
    )

    assert "simulation_clock_not_advancing" in blockers


def test_advancing_gazebo_clock_allows_an_otherwise_ready_simulation():
    blockers = READINESS.readiness_blockers(
        odom_count=8,
        scan_count=3,
        odom_stable=True,
        cmd_vel_publishers=1,
        cmd_vel_subscribers=1,
        action_server_seen=True,
        clock_count=8,
        clock_span_s=0.4,
        min_odom_samples=5,
        min_scan_samples=1,
        min_clock_samples=3,
        min_clock_advance_s=0.1,
    )

    assert blockers == ()


def test_clock_span_uses_ros_clock_samples():
    assert READINESS.clock_span_seconds([1_000_000_000, 1_250_000_000]) == 0.25
    assert READINESS.clock_span_seconds([1_000_000_000]) == 0.0


def test_high_rate_clock_uses_the_full_collection_span_not_last_fifty_frames():
    node = object.__new__(READINESS.SimulationReadinessNode)
    node.clock_count = 500
    node.clock_first_ns = 1_000_000_000
    node.clock_latest_ns = 1_250_000_000

    assert node._clock_span_s() == 0.25


def test_collect_waits_for_control_graph_after_sensors_are_ready():
    """WSL 冷启动时传感器可能先到，不能在 Lifecycle 配置完成前提前返回。"""

    checker = object.__new__(READINESS.SimulationReadinessNode)
    checker.odom_count = 0
    checker.scan_count = 0
    checker.clock_count = 0
    checker.clock_first_ns = None
    checker.clock_latest_ns = None
    checker.odom_positions = []

    class FakeGraphNode:
        def __init__(self):
            self.spin_count = 0

        def count_publishers(self, topic):
            assert topic == "/cmd_vel"
            return 1 if self.spin_count >= 3 else 0

        def count_subscribers(self, topic):
            assert topic == "/cmd_vel"
            return 2

        def count_services(self, service):
            assert service == "/robot/execute_command/_action/send_goal"
            return 1 if self.spin_count >= 3 else 0

    graph = FakeGraphNode()

    class FakeRclpy:
        @staticmethod
        def spin_once(node, *, timeout_sec):
            assert node is graph
            assert timeout_sec == 0.1
            graph.spin_count += 1
            # 第一轮就满足传感器门槛；控制端点故意延迟到第三轮。
            checker.odom_count = 5
            checker.scan_count = 1
            checker.clock_count = 3
            checker.clock_first_ns = 1_000_000_000
            checker.clock_latest_ns = 1_100_000_000
            checker.odom_positions = [(0.0, 0.0), (0.0, 0.0)]

    checker.node = graph
    checker._rclpy = FakeRclpy()

    report = checker.collect(
        1.0,
        min_odom_samples=5,
        min_scan_samples=1,
        min_clock_samples=3,
        min_clock_advance_s=0.1,
        stable_window=2,
        stable_epsilon_m=0.03,
    )

    assert graph.spin_count >= 3
    assert report.ok is True
    assert report.blockers == ()
