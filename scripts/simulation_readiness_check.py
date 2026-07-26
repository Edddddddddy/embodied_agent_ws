#!/usr/bin/env python3
"""Check that the Gazebo TurtleBot3 simulation is actually present.

`continuous_voice_control.sh` launches a full demo stack.  Audio readiness alone is
not enough: if the robot failed to spawn, the terminal can still show ASR/Agent logs
while Gazebo looks empty. This probe waits for `/odom`, `/scan` and an advancing
`/clock`; the clock gate prevents timed Actions from staying at executing 0% forever.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class SimulationReadinessReport:
    odom_seen: bool
    scan_seen: bool
    odom_count: int
    scan_count: int
    clock_seen: bool
    clock_count: int
    clock_advanced: bool
    clock_span_s: float
    odom_stable: bool
    odom_recent_drift_m: float
    cmd_vel_publishers: int
    cmd_vel_subscribers: int
    action_server_seen: bool
    ok: bool
    blockers: tuple[str, ...]


class SimulationReadinessNode:
    def __init__(self):
        import rclpy
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from rclpy.node import Node
        from rosgraph_msgs.msg import Clock
        from sensor_msgs.msg import LaserScan

        class _Node(Node):
            def __init__(self, owner: SimulationReadinessNode):
                super().__init__("simulation_readiness_check")
                self.create_subscription(Odometry, "/odom", owner._on_odom, 10)
                self.create_subscription(LaserScan, "/scan", owner._on_scan, 10)
                self.create_subscription(Clock, "/clock", owner._on_clock, 10)
                self.create_subscription(Twist, "/cmd_vel", owner._on_cmd_vel, 10)

        self.odom_count = 0
        self.scan_count = 0
        self.clock_count = 0
        self.clock_first_ns: int | None = None
        self.clock_latest_ns: int | None = None
        self.odom_positions: list[tuple[float, float]] = []
        self._rclpy = rclpy
        self.node = _Node(self)

    def _on_odom(self, message) -> None:
        self.odom_count += 1
        self.odom_positions.append(
            (message.pose.pose.position.x, message.pose.pose.position.y)
        )
        self.odom_positions = self.odom_positions[-20:]

    def _on_scan(self, _message) -> None:
        self.scan_count += 1

    def _on_clock(self, message) -> None:
        stamp = message.clock
        current_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        self.clock_count += 1
        if self.clock_first_ns is None:
            self.clock_first_ns = current_ns
        self.clock_latest_ns = current_ns

    def _clock_span_s(self) -> float:
        if self.clock_first_ns is None or self.clock_latest_ns is None:
            return 0.0
        return clock_span_seconds([self.clock_first_ns, self.clock_latest_ns])

    def _on_cmd_vel(self, _message) -> None:
        # 订阅只用于让 `ros2 topic info`/图谱更容易看懂 readiness 关注 cmd_vel；
        # readiness 本身不要求机器人已经运动。
        return

    def collect(
        self,
        timeout_s: float,
        *,
        min_odom_samples: int,
        min_scan_samples: int,
        min_clock_samples: int,
        min_clock_advance_s: float,
        stable_window: int,
        stable_epsilon_m: float,
    ) -> SimulationReadinessReport:
        deadline = time.monotonic() + max(0.1, timeout_s)
        while time.monotonic() < deadline:
            self._rclpy.spin_once(self.node, timeout_sec=0.1)
            odom_recent_drift = _recent_odom_drift(
                self.odom_positions, stable_window
            )
            # Gazebo 传感器通常比 Lifecycle configure 更早可见。必须用完整
            # readiness 契约决定退出；否则慢 WSL 会在 Action server 和
            # /cmd_vel publisher 出现前返回，与真正的控制栈故障表现相同。
            blockers = readiness_blockers(
                odom_count=self.odom_count,
                scan_count=self.scan_count,
                odom_stable=(
                    self.odom_count >= min_odom_samples
                    and odom_recent_drift <= stable_epsilon_m
                ),
                cmd_vel_publishers=self.node.count_publishers("/cmd_vel"),
                cmd_vel_subscribers=self.node.count_subscribers("/cmd_vel"),
                action_server_seen=(
                    self.node.count_services(
                        "/robot/execute_command/_action/send_goal"
                    )
                    > 0
                ),
                clock_count=self.clock_count,
                clock_span_s=self._clock_span_s(),
                min_odom_samples=min_odom_samples,
                min_scan_samples=min_scan_samples,
                min_clock_samples=min_clock_samples,
                min_clock_advance_s=min_clock_advance_s,
            )
            if not blockers:
                break

        cmd_vel_publishers = self.node.count_publishers("/cmd_vel")
        cmd_vel_subscribers = self.node.count_subscribers("/cmd_vel")
        action_server_seen = self.node.count_services("/robot/execute_command/_action/send_goal") > 0
        odom_recent_drift = _recent_odom_drift(self.odom_positions, stable_window)
        odom_stable = (
            self.odom_count >= min_odom_samples
            and odom_recent_drift <= stable_epsilon_m
        )
        clock_span_s = self._clock_span_s()
        clock_count = self.clock_count
        clock_advanced = (
            clock_count >= min_clock_samples
            and clock_span_s >= min_clock_advance_s
        )
        blockers = readiness_blockers(
            odom_count=self.odom_count,
            scan_count=self.scan_count,
            odom_stable=odom_stable,
            cmd_vel_publishers=cmd_vel_publishers,
            cmd_vel_subscribers=cmd_vel_subscribers,
            action_server_seen=action_server_seen,
            clock_count=clock_count,
            clock_span_s=clock_span_s,
            min_odom_samples=min_odom_samples,
            min_scan_samples=min_scan_samples,
            min_clock_samples=min_clock_samples,
            min_clock_advance_s=min_clock_advance_s,
        )

        return SimulationReadinessReport(
            odom_seen=self.odom_count > 0,
            scan_seen=self.scan_count > 0,
            odom_count=self.odom_count,
            scan_count=self.scan_count,
            clock_seen=clock_count > 0,
            clock_count=clock_count,
            clock_advanced=clock_advanced,
            clock_span_s=clock_span_s,
            odom_stable=odom_stable,
            odom_recent_drift_m=odom_recent_drift,
            cmd_vel_publishers=cmd_vel_publishers,
            cmd_vel_subscribers=cmd_vel_subscribers,
            action_server_seen=action_server_seen,
            ok=not blockers,
            blockers=blockers,
        )

    def close(self) -> None:
        self.node.destroy_node()


def format_report(report: SimulationReadinessReport) -> str:
    status = "PASS" if report.ok else "BLOCKED"
    lines = [
        f"{status}: TurtleBot3 simulation readiness",
        f"  odom_seen: {report.odom_seen} count={report.odom_count}",
        f"  scan_seen: {report.scan_seen} count={report.scan_count}",
        (
            f"  clock_seen: {report.clock_seen} count={report.clock_count} "
            f"advanced={report.clock_advanced} span_s={report.clock_span_s:.3f}"
        ),
        f"  odom_stable: {report.odom_stable} recent_drift_m={report.odom_recent_drift_m:.3f}",
        f"  cmd_vel_publishers: {report.cmd_vel_publishers}",
        f"  cmd_vel_subscribers: {report.cmd_vel_subscribers}",
        f"  action_server_seen: {report.action_server_seen}",
    ]
    if report.blockers:
        lines.append("  blockers:")
        for blocker in report.blockers:
            lines.append(f"    - {blocker}")
        lines.append(
            "  next: 检查 Gazebo 是否启动、turtlebot3_gazebo 是否安装、"
            "ros_gz_bridge 是否正常、GZ_PARTITION 是否隔离，以及 /clock 是否持续推进。"
        )
    else:
        lines.append("  next: 小车模型、里程计、雷达和动作执行入口已就绪。")
    return "\n".join(lines)


def _recent_odom_drift(positions: list[tuple[float, float]], window: int) -> float:
    recent = positions[-window:]
    if len(recent) < window:
        return math.inf
    xs = [item[0] for item in recent]
    ys = [item[1] for item in recent]
    return math.hypot(max(xs) - min(xs), max(ys) - min(ys))


def clock_span_seconds(samples_ns: list[int]) -> float:
    """返回采样窗口内 ROS 仿真时间的推进量。"""

    if len(samples_ns) < 2:
        return 0.0
    return max(0.0, (max(samples_ns) - min(samples_ns)) / 1_000_000_000.0)


def readiness_blockers(
    *,
    odom_count: int,
    scan_count: int,
    odom_stable: bool,
    cmd_vel_publishers: int,
    cmd_vel_subscribers: int,
    action_server_seen: bool,
    clock_count: int,
    clock_span_s: float,
    min_odom_samples: int,
    min_scan_samples: int,
    min_clock_samples: int,
    min_clock_advance_s: float,
) -> tuple[str, ...]:
    """纯函数形式的仿真门禁，便于锁定“/clock 冻结但 topic 存在”的回归。"""

    blockers: list[str] = []
    if odom_count < min_odom_samples:
        blockers.append("no_odom_from_turtlebot3")
    if scan_count < min_scan_samples:
        blockers.append("no_scan_from_turtlebot3_lidar")
    if clock_count < min_clock_samples:
        blockers.append("no_clock_from_gazebo")
    elif clock_span_s < min_clock_advance_s:
        blockers.append("simulation_clock_not_advancing")
    if not odom_stable:
        blockers.append("odom_not_stable_after_spawn")
    if cmd_vel_publishers == 0:
        blockers.append("no_cmd_vel_publisher")
    if cmd_vel_subscribers == 0:
        blockers.append("no_cmd_vel_subscriber")
    if not action_server_seen:
        blockers.append("robot_execute_command_action_server_missing")
    return tuple(blockers)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--min-odom-samples", type=int, default=5)
    parser.add_argument("--min-scan-samples", type=int, default=1)
    parser.add_argument("--min-clock-samples", type=int, default=3)
    parser.add_argument("--min-clock-advance-s", type=float, default=0.1)
    parser.add_argument("--stable-window", type=int, default=5)
    parser.add_argument("--stable-epsilon-m", type=float, default=0.03)
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()

    import rclpy

    rclpy.init()
    checker = SimulationReadinessNode()
    try:
        report = checker.collect(
            args.timeout,
            min_odom_samples=max(1, args.min_odom_samples),
            min_scan_samples=max(1, args.min_scan_samples),
            min_clock_samples=max(2, args.min_clock_samples),
            min_clock_advance_s=max(0.001, args.min_clock_advance_s),
            stable_window=max(2, args.stable_window),
            stable_epsilon_m=max(0.0, args.stable_epsilon_m),
        )
    finally:
        checker.close()
        if rclpy.ok():
            rclpy.shutdown()

    if args.json:
        print(json.dumps(asdict(report), ensure_ascii=False, indent=2))
    else:
        print(format_report(report))
    raise SystemExit(0 if report.ok else 1)


if __name__ == "__main__":
    main()
