"""仓库结构约束：用户命令与集成测试必须分区，避免 scripts/ 再次退化成杂物箱。"""

import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def test_integration_probes_are_not_mixed_with_user_scripts():
    misplaced = sorted((ROOT / "scripts").glob("test_*"))
    assert misplaced == [], f"测试探针应放入 tests/integration: {misplaced}"


def test_critical_full_chain_probes_remain_discoverable():
    integration = ROOT / "tests" / "integration"
    required = {
        "test_gazebo_voice.py",
        "test_mock_executor_pipeline.py",
        "test_online_api.py",
        "test_recognition_retry.py",
        "test_continuous_command_ttl.py",
        "test_continuous_endpoint_asr.py",
        "test_continuous_multi_command.py",
        "test_continuous_navigation_queue.py",
        "test_continuous_live_check.py",
        "test_continuous_session_timeout.py",
        "test_continuous_voice_soak.py",
        "test_continuous_voice_control.py",
        "test_continuous_voice_control_script.py",
        "test_continuous_nav2_voice_control_script.py",
        "test_continuous_voice_monitor.py",
        "test_continuous_kws_sidecar.py",
        "test_voice_provider_preflight.py",
        "test_audio_frontend_calibration.py",
        "test_typed_action_server.py",
        "test_navigation_sequence.py",
        "test_nav2_bridge_sequence.py",
        "test_nav2_turtlebot3_voice.py",
    }
    present = {path.name for path in integration.glob("test_*")}
    assert required <= present


def test_voice_navigation_acceptance_entrypoints_remain_available():
    """语音目标点导航/巡航是当前阶段核心能力，入口脚本不能在整理中丢失。"""

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    for mode in (
        "navigation-demo",
        "nav2-bridge",
        "nav2-preflight",
        "nav2-turtlebot3",
        "continuous-nav2-offline",
        "continuous-nav2-online",
        "continuous-nav2-live-check",
        "continuous-navigation",
    ):
        assert mode in acceptance

    for script in (
        "smoke_test_navigation_sequence.sh",
        "smoke_test_continuous_navigation_queue.sh",
        "smoke_test_nav2_bridge.sh",
        "smoke_test_nav2_preflight.sh",
        "smoke_test_nav2_turtlebot3_voice.sh",
        "continuous_nav2_voice_control.sh",
        "publish_nav2_initial_pose.py",
    ):
        assert (ROOT / "scripts" / script).is_file()

    assert (
        ROOT / "src" / "embodied_simulation" / "launch" / "voice_nav2_turtlebot3.launch.py"
    ).is_file()
    assert (ROOT / "src" / "embodied_simulation" / "config" / "places.yaml").is_file()


def test_audio_endpoint_events_remain_wired_through_frontend_and_agents():
    """连续语音控制依赖 speech_started/speech_ended 做端点观测与 ASR commit。

    这是一个轻量结构护栏：真正的行为由 C++ audio_processing 单测和
    smoke_test_audio_endpoint.sh 覆盖；这里防止重构时误删 topic wiring。
    """

    audio_frontend = (
        ROOT / "src" / "embodied_agent_cpp" / "src" / "audio_frontend_node.cpp"
    ).read_text(encoding="utf-8")
    online_agent = (
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "online_agent_node.py"
    ).read_text(encoding="utf-8")
    offline_agent = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py"
    ).read_text(encoding="utf-8")
    audio_smoke = (ROOT / "scripts" / "smoke_test_audio_endpoint.sh").read_text(
        encoding="utf-8"
    )

    for topic in ("/audio/speech_started", "/audio/speech_ended"):
        assert topic in audio_frontend
        assert topic in online_agent
        assert topic in offline_agent
        assert topic in audio_smoke

    assert "endpoint_events_enabled" in audio_frontend
    assert "speech_started_publisher_->publish" in audio_frontend
    assert "speech_ended_publisher_->publish" in audio_frontend


def test_continuous_mode_does_not_drop_busy_asr_or_endpoint_commits():
    """真实麦克风路径 busy 时也必须继续接收 ASR final 与 speech endpoint。

    之前真人连续说话体验差的根因之一是 Agent 忙时直接丢弃后续 ASR final。
    这里锁住 online/offline 两条真实音频路径的关键不变量：
    只有“非连续模式”可以因为 busy 抑制 ASR final 或 endpoint commit；
    连续模式必须让 transcript 进入 ContinuousCommandQueue。
    """

    agent_sources = [
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "online_agent_node.py",
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py",
    ]
    for source_path in agent_sources:
        source = source_path.read_text(encoding="utf-8")
        for function_name in ("_commit_asr_endpoint", "_on_asr_final"):
            match = re.search(
                rf"def {function_name}\([^)]*\):(?P<body>.*?)(?=\n    def |\n\nclass |\Z)",
                source,
                flags=re.S,
            )
            assert match is not None, f"{function_name} missing in {source_path}"
            body = match.group("body")
            assert "self._is_busy()" in body, f"{function_name} lost busy guard"
            assert "not self._continuous_enabled" in body, (
                f"{function_name} must not suppress continuous-mode ASR in {source_path}"
            )


def test_continuous_voice_state_machine_remains_shared_by_online_and_offline_agents():
    """online/offline Agent 必须共享连续会话、队列与执行追踪模块。

    连续语音控制最容易在修 bug 时退回“两份差不多的状态机”。这里用结构测试锁住
    deep module 边界：状态机/队列/执行事件在 embodied_online_agent.continuous_voice，
    两个 Agent 只负责 ROS wiring 和 provider 差异。
    """

    shared = (
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "continuous_voice.py"
    ).read_text(encoding="utf-8")
    online_agent = (
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "online_agent_node.py"
    ).read_text(encoding="utf-8")
    offline_agent = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py"
    ).read_text(encoding="utf-8")

    for class_name in (
        "ContinuousVoiceSession",
        "ContinuousCommandQueue",
        "CommandExecutionTracker",
    ):
        assert f"class {class_name}" in shared
        assert class_name in online_agent
        assert class_name in offline_agent

    assert "from .continuous_voice import" in online_agent
    assert "from embodied_online_agent.continuous_voice import" in offline_agent
