"""真人语音 SLAM 探针的 ROS graph 行为契约。"""

from __future__ import annotations

import sys
from pathlib import Path
import time

import rclpy
from embodied_agent_core.ros_qos import command_qos, event_qos, state_qos
from embodied_agent_interfaces.msg import AudioFrontendStatus, WakeEvent
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String


ROOT = Path(__file__).resolve().parents[3]
PACKAGE_ROOT = ROOT / "src" / "embodied_slam_tools"
if str(PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PACKAGE_ROOT))

from tools.acceptance.probes.slam_nav.session_observer import SessionObserver
from tools.acceptance.probes.slam_nav.live_voice_trigger import (
    await_automatic_mission_trigger,
    finalize_trigger_report,
)


def _spin_until(executor, predicate, timeout_s=2.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)
        if predicate():
            return
    raise TimeoutError("ROS graph condition was not observed")


class _StubAsrPublisher:
    def __init__(self):
        self.messages = []

    @staticmethod
    def get_subscription_count():
        return 1

    def publish(self, message):
        self.messages.append(message)


def test_trigger_strategy_preserves_the_synthetic_acceptance_path():
    node = type("SyntheticObserver", (), {})()
    node.asr_pub = _StubAsrPublisher()

    await_automatic_mission_trigger(
        node,
        source="synthetic",
        timeout_s=1.0,
        agent_mode="offline",
    )

    assert [message.data for message in node.asr_pub.messages] == [
        "开始自动巡检建图"
    ]


def test_live_trigger_strategy_prompts_only_after_sources_are_ready(capsys):
    class LiveObserver:
        asr_pub = None

        def __init__(self):
            self.window_started = False
            self.wait_timeout = None

        def begin_live_voice_trigger_window(self):
            self.window_started = True

        def live_voice_sources_ready(self):
            return self.window_started

        def wait_for_live_voice_trigger(self, timeout_s):
            self.wait_timeout = timeout_s
            return True

    node = LiveObserver()
    await_automatic_mission_trigger(
        node,
        source="live_voice",
        timeout_s=45.0,
        agent_mode="online",
    )

    assert node.wait_timeout == 45.0
    output = capsys.readouterr().out
    assert "online" in output
    assert "小智，开始自动巡检建图" in output


def test_report_finalizer_wraps_only_the_live_voice_run():
    core_report = {"schema_version": 4, "session_id": "session-live"}

    class EvidenceObserver:
        def __init__(self):
            self.calls = []

        def build_live_voice_evidence(self, report, *, agent_mode):
            self.calls.append((report, agent_mode))
            return {"schema_version": 1, "core_report": report}

    observer = EvidenceObserver()
    live_report = finalize_trigger_report(
        observer,
        core_report,
        live_voice_trigger=True,
        agent_mode="online",
    )
    synthetic_report = finalize_trigger_report(
        observer,
        core_report,
        live_voice_trigger=False,
        agent_mode="offline",
    )

    assert live_report == {"schema_version": 1, "core_report": core_report}
    assert synthetic_report is core_report
    assert observer.calls == [(core_report, "online")]


def test_report_finalizer_leaves_a_fail_closed_envelope_before_voice_window():
    core_failure = {
        "schema_version": 4,
        "session_id": "session-startup-failed",
        "passed": False,
    }

    class NoWindowObserver:
        @staticmethod
        def build_live_voice_evidence(_report, *, agent_mode):
            raise RuntimeError("live voice trigger window has not started")

    report = finalize_trigger_report(
        NoWindowObserver(),
        core_failure,
        live_voice_trigger=True,
        agent_mode="offline",
    )

    assert report["schema_version"] == 1
    assert report["session_id"] == "session-startup-failed"
    assert report["passed"] is False
    assert not any(report["checks"].values())
    assert report["core_report"] is core_failure
    assert "window has not started" in report["error"]


def test_live_voice_observer_physically_omits_the_synthetic_asr_publisher(
    monkeypatch,
):
    # 使用独立 domain，避免开发机上残留的 Agent publisher 污染 graph 断言。
    monkeypatch.setenv("ROS_DOMAIN_ID", "177")
    rclpy.init()
    live_observer = None
    try:
        live_observer = SessionObserver(synthetic_asr_enabled=False)
        assert live_observer.asr_pub is None
        assert live_observer.count_publishers("/agent/asr_final") == 0
    finally:
        if live_observer is not None:
            live_observer.destroy_node()
        rclpy.shutdown()


def test_synthetic_observer_preserves_the_existing_asr_publisher(monkeypatch):
    monkeypatch.setenv("ROS_DOMAIN_ID", "178")
    rclpy.init()
    synthetic_observer = None
    try:
        synthetic_observer = SessionObserver(synthetic_asr_enabled=True)
        assert synthetic_observer.asr_pub is not None
        assert synthetic_observer.count_publishers("/agent/asr_final") == 1
    finally:
        if synthetic_observer is not None:
            synthetic_observer.destroy_node()
        rclpy.shutdown()


def test_live_window_ignores_old_and_filler_events_then_accepts_real_intent(
    monkeypatch,
):
    monkeypatch.setenv("ROS_DOMAIN_ID", "179")
    rclpy.init()
    observer = SessionObserver(synthetic_asr_enabled=False)
    source = Node("live_voice_test_source")
    executor = SingleThreadedExecutor()
    executor.add_node(observer)
    executor.add_node(source)
    asr = source.create_publisher(String, "/agent/asr_final", command_qos(10))
    audio = source.create_publisher(
        AudioFrontendStatus, "/audio/frontend_metrics", state_qos()
    )
    started = source.create_publisher(
        Empty, "/audio/speech_started", event_qos(10)
    )
    ended = source.create_publisher(
        Empty, "/audio/speech_ended", event_qos(10)
    )
    wake = source.create_publisher(WakeEvent, "/agent/wake_event", event_qos())
    # 模拟真正消费门控命令的 SessionOrchestratorNode；探针自身是另一个订阅者。
    source.create_subscription(
        WakeEvent, "/agent/wake_event", lambda _message: None, event_qos()
    )
    try:
        # 窗口建立前没有 ASR subscription，因此旧的 volatile final 无法被
        # 延迟回调误认成本次真人触发。
        asr.publish(String(data="小智，开始自动建图"))
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)

        observer.begin_live_voice_trigger_window()
        _spin_until(executor, observer.live_voice_sources_ready)
        assert asr.get_subscription_count() == 1
        assert observer.wait_for_live_voice_trigger(0.01) is False

        asr.publish(String(data="嗯"))
        for _ in range(5):
            executor.spin_once(timeout_sec=0.02)
        assert observer.wait_for_live_voice_trigger(0.01) is False

        started.publish(Empty())
        for _ in range(3):
            executor.spin_once(timeout_sec=0.02)
        metric = AudioFrontendStatus()
        metric.speech = True
        metric.rms = 0.08
        metric.peak = 4_000
        metric.vad_provider = "energy"
        audio.publish(metric)
        for _ in range(3):
            executor.spin_once(timeout_sec=0.02)
        ended.publish(Empty())
        for _ in range(3):
            executor.spin_once(timeout_sec=0.02)
        asr.publish(String(data="小智，开始自动巡检建图"))
        for _ in range(3):
            executor.spin_once(timeout_sec=0.02)
        # 裸 ASR 即使语义匹配，也不能代替 Agent 的会话授权。
        assert observer.wait_for_live_voice_trigger(0.01) is False

        wake_event = WakeEvent()
        wake_event.kind = WakeEvent.KIND_WAKE
        wake_event.provider = "text"
        wake_event.transcript = "小智，开始自动巡检建图"
        wake_event.command_known = False
        wake_event.command = "开始自动巡检建图"
        wake.publish(wake_event)
        for _ in range(3):
            executor.spin_once(timeout_sec=0.02)
        assert observer.wait_for_live_voice_trigger(0.01) is False

        wake_event.command_known = True
        wake.publish(wake_event)

        _spin_until(
            executor,
            lambda: observer.wait_for_live_voice_trigger(0.0),
        )
        for _ in range(3):
            executor.spin_once(timeout_sec=0.02)
        envelope = observer.build_live_voice_evidence({}, agent_mode="offline")
        assert envelope["checks"] == {
            "real_audio_observed": True,
            "endpoint_observed": True,
            "wake_accepted": True,
            "automatic_mission_asr_final": True,
            "strict_core_schema_v4_passed": False,
        }
        assert envelope["voice_window"]["matched_wake_event"]["command"] == (
            "开始自动巡检建图"
        )
    finally:
        executor.shutdown()
        observer.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
