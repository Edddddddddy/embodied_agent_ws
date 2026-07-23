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
