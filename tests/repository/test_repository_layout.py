"""仓库结构约束：用户命令与集成测试必须分区，避免 scripts/ 再次退化成杂物箱。"""

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
        "test_continuous_session_timeout.py",
        "test_continuous_voice_control_script.py",
        "test_voice_provider_preflight.py",
        "test_audio_frontend_calibration.py",
        "test_typed_action_server.py",
    }
    present = {path.name for path in integration.glob("test_*")}
    assert required <= present


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
