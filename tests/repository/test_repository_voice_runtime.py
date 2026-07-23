"""连续语音、VAD/KWS、导航与校准链路约束。"""

import re

from repository_test_support import (
    BRINGUP_ROOT,
    CORE_ROOT,
    ROOT,
    VOICE_FRONTEND_ROOT,
    _python_literal,
    assert_acceptance_modes,
)


def test_voice_navigation_places_stay_consistent_across_agent_guard_and_nav2():
    """Agent、ActionGuard、Nav2 executor 必须共享同一组标准地点名。

    语音导航链路跨 Python Agent、C++ 安全网关和 Nav2 坐标配置。任何一层漏掉
    某个 canonical place，都会导致“ASR/LLM 已解析，但机器人不执行”的现场事故。
    这里不限制中文别名，只锁住标准地点名和默认巡航点。
    """

    navigation_phrases = CORE_ROOT / "navigation_phrases.py"
    action_validator = (
        ROOT / "src" / "embodied_agent_cpp" / "src" / "action_validator.cpp"
    ).read_text(encoding="utf-8")
    places_yaml = (
        ROOT / "src" / "embodied_simulation" / "config" / "places.yaml"
    ).read_text(encoding="utf-8")

    agent_places = set(_python_literal(navigation_phrases, "PLACE_ALIASES"))
    default_patrol = _python_literal(navigation_phrases, "DEFAULT_PATROL_WAYPOINTS")
    guard_function = re.search(
        r"supported_navigation_places\(\).*?static const std::set<std::string> places\{(?P<body>.*?)\};",
        action_validator,
        flags=re.S,
    )
    assert guard_function is not None
    guard_places = set(re.findall(r'"([a-zA-Z0-9_]+)"', guard_function.group("body")))
    nav2_places = set(re.findall(r"^  ([a-zA-Z0-9_]+):\s*\{", places_yaml, flags=re.M))

    assert agent_places == guard_places == nav2_places
    assert set(default_patrol) <= agent_places
    assert default_patrol == ["door", "desk", "home"]

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
    agent_topics = (CORE_ROOT / "ros_topics.py").read_text(encoding="utf-8")
    audio_smoke = (ROOT / "scripts" / "smoke_test_audio_endpoint.sh").read_text(
        encoding="utf-8"
    )

    for topic in ("/audio/speech_started", "/audio/speech_ended"):
        assert topic in audio_frontend
        assert topic in agent_topics
        assert topic in audio_smoke

    for node in (online_agent, offline_agent):
        assert "AgentRosIo" in node
        assert "speech_started=self._on_speech_started" in node
        assert "speech_ended=self._on_speech_ended" in node

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
        match = re.search(
            r"def _on_asr_final\([^)]*\):(?P<body>.*?)(?=\n    def |\n\nclass |\Z)",
            source,
            flags=re.S,
        )
        assert match is not None, f"_on_asr_final missing in {source_path}"
        body = match.group("body")
        assert "self._is_busy()" in body
        assert "not self._continuous_enabled" in body

        # endpoint 的 busy/continuous gate 已下沉到共享 runtime 构造参数；节点 wrapper
        # 只负责转发 source，避免 online/offline 再复制计时与 timer 状态机。
        assert "self._runtime.endpoint.request(source)" in source
        assert "blocked=lambda: self._is_busy()" in source
        assert "and not self._continuous_enabled" in source

def test_continuous_voice_state_machine_remains_shared_by_online_and_offline_agents():
    """online/offline Agent 必须共享连续会话、队列与执行追踪模块。

    连续语音控制最容易在修 bug 时退回“两份差不多的状态机”。这里用结构测试锁住
    deep module 边界：状态机/队列/执行事件在 embodied_agent_core.continuous_voice，
    两个 Agent 只负责 ROS wiring 和 provider 差异。
    """

    shared = (CORE_ROOT / "continuous_voice.py").read_text(encoding="utf-8")
    control_plane = (CORE_ROOT / "agent_control_plane.py").read_text(
        encoding="utf-8"
    )
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
        assert class_name in control_plane
        assert f"class {class_name}" not in online_agent
        assert f"class {class_name}" not in offline_agent

    for node in (online_agent, offline_agent):
        assert "AgentControlPlane" in node
        assert "AgentRosIo" in node
        assert "AgentExecutionRuntime" in node
        assert "AsrEndpointRuntime" in node
        assert "def _run_command_worker" not in node
        assert ".command_nlu.parse(command)" not in node
        assert "threading.Timer(" not in node

    execution_runtime = (CORE_ROOT / "agent_execution_runtime.py").read_text(
        encoding="utf-8"
    )
    endpoint_runtime = (CORE_ROOT / "asr_endpoint_runtime.py").read_text(
        encoding="utf-8"
    )
    streaming_turn = (CORE_ROOT / "streaming_turn.py").read_text(encoding="utf-8")
    assert "class AgentExecutionRuntime" in execution_runtime
    assert "class AsrEndpointRuntime" in endpoint_runtime
    assert "class StreamingTurnRuntime" in streaming_turn
    assert "def enqueue_command(" in control_plane

    # LLM tagged stream、分句和安全动作选择只能由共享运行时拥有，节点只接 provider。
    for node in (online_agent, offline_agent):
        assert "StreamingTurnRuntime" in node
        assert "TaggedStreamParser" not in node
        assert "SentenceChunker" not in node
        assert "parse_fallback_actions" not in node
        assert "should_block_model_actions" not in node

def test_ros_dds_env_disables_fastdds_shm_by_default_for_wsl_demos():
    """WSL 真实语音/Gazebo 演示默认绕开 FastDDS SHM 端口锁。

    用户现场经常遇到 `Failed init_port fastrtps_port7000`。这个结构测试锁住
    activate.sh 的默认 DDS 环境，避免后续脚本整理时把 UDPv4 fallback 删掉。
    """

    activate = (ROOT / "scripts" / "activate.sh").read_text(encoding="utf-8")
    dds_env_path = ROOT / "scripts" / "ros_dds_env.sh"
    dds_env = dds_env_path.read_text(encoding="utf-8")
    continuous = (ROOT / "scripts" / "continuous_voice_control.sh").read_text(
        encoding="utf-8"
    )

    assert dds_env_path.is_file()
    assert "source \"$WORKSPACE/scripts/ros_dds_env.sh\"" in activate
    assert "FASTDDS_BUILTIN_TRANSPORTS" in dds_env
    assert "UDPv4" in dds_env
    assert "EMBODIED_ALLOW_FASTDDS_SHM" in dds_env
    assert "FASTDDS_BUILTIN_TRANSPORTS" in continuous

def test_webrtc_vad_sidecar_remains_integrated_as_optional_voice_provider():
    """真实麦克风稳定性不能只依赖 energy VAD。

    WebRTC VAD 是本项目的轻量成熟 provider：依赖比 Silero 小，适合 WSL/笔记本演示；
    这里锁住 entry point、launch 条件、preflight auto fallback 和文档入口。
    """

    setup_py = (ROOT / "src" / "embodied_voice_frontend" / "setup.py").read_text(
        encoding="utf-8"
    )
    sidecar = (
        ROOT
        / "src"
        / "embodied_voice_frontend"
        / "embodied_voice_frontend"
        / "silero_vad_sidecar.py"
    ).read_text(encoding="utf-8")
    node_path = (
        ROOT
        / "src"
        / "embodied_voice_frontend"
        / "embodied_voice_frontend"
        / "webrtc_vad_node.py"
    )
    online_launch = (
        ROOT / "src" / "embodied_online_agent" / "launch" / "online_agent.launch.py"
    ).read_text(encoding="utf-8")
    offline_launch = (
        ROOT / "src" / "embodied_offline_agent" / "launch" / "offline_agent.launch.py"
    ).read_text(encoding="utf-8")
    frontend_launch = (BRINGUP_ROOT / "voice_frontend_launch_contract.py").read_text(
        encoding="utf-8"
    )
    preflight = (ROOT / "scripts" / "voice_provider_preflight.py").read_text(
        encoding="utf-8"
    )
    continuous = (ROOT / "scripts" / "continuous_voice_control.sh").read_text(
        encoding="utf-8"
    )

    assert node_path.is_file()
    assert "webrtc-vad" in setup_py
    assert "silero-vad" in setup_py
    assert "webrtc_vad = embodied_voice_frontend.webrtc_vad_node:main" in setup_py
    assert "class WebRtcVadProvider" in sidecar
    assert "class SileroOnnxVadProvider" in sidecar
    assert "WebRTC VAD frame_ms must be one of [10, 20, 30]" in sidecar
    assert "executable=\"webrtc_vad\"" in frontend_launch
    assert "' != 'silero' and '" in frontend_launch
    for launch in (online_launch, offline_launch):
        assert "voice_frontend_nodes" in launch
    assert "vad:auto_fallback:webrtc" in preflight
    assert "webrtcvad_package_missing" in preflight
    assert "auto 会优先 Silero，其次 WebRTC，最后降级 energy" in continuous
    assert (ROOT / "scripts" / "setup_voice_vad_runtime.sh").is_file()
    assert (ROOT / "scripts" / "silero_onnx_smoke.py").is_file()
    assert (ROOT / "scripts" / "silero_ros_runtime_probe.py").is_file()
    assert (ROOT / "scripts" / "smoke_test_silero_vad_runtime.sh").is_file()
    assert (ROOT / "scripts" / "smoke_test_webrtc_vad_sidecar.sh").is_file()
    assert_acceptance_modes(
        "voice-vad-runtime-dry-run", "webrtc-vad-sidecar", "silero-vad-runtime"
    )

def test_acoustic_keyword_wake_runtime_entrypoints_remain_available():
    """声学唤醒不能只停留在 mock_text seam，需要有可部署的 provider/runtime 入口。"""

    setup_py = (ROOT / "src" / "embodied_voice_frontend" / "setup.py").read_text(
        encoding="utf-8"
    )
    keyword_wake = (
        ROOT
        / "src"
        / "embodied_voice_frontend"
        / "embodied_voice_frontend"
        / "keyword_wake.py"
    ).read_text(encoding="utf-8")
    preflight = (ROOT / "scripts" / "voice_provider_preflight.py").read_text(
        encoding="utf-8"
    )

    assert "kws" in setup_py
    assert "livekit-kws" in setup_py
    assert "from openwakeword.model import Model" in keyword_wake
    assert "from livekit.wakeword import WakeWordModel" in keyword_wake
    assert "kws:openwakeword_package_missing" in preflight
    assert "kws:sherpa_onnx_package_missing" in preflight
    assert (ROOT / "scripts" / "setup_voice_kws_runtime.sh").is_file()
    assert (ROOT / "scripts" / "smoke_test_sherpa_kws_sidecar.sh").is_file()
    assert_acceptance_modes("voice-kws-runtime-dry-run", "sherpa-kws-sidecar")

def test_audio_calibration_outputs_copyable_live_demo_advice():
    """真实麦克风校准必须产出可复制的下一步命令。"""

    calibration = (ROOT / "scripts" / "audio_frontend_calibration.py").read_text(
        encoding="utf-8"
    )
    bundle = (ROOT / "scripts" / "voice_calibration_report.py").read_text(
        encoding="utf-8"
    )
    readiness = (ROOT / "scripts" / "voice_control_readiness_check.py").read_text(
        encoding="utf-8"
    )
    continuous = (ROOT / "scripts" / "continuous_voice_control.sh").read_text(
        encoding="utf-8"
    )
    readme = (ROOT / "README.md").read_text(encoding="utf-8")

    for token in (
        "recommended_environment",
        "next_command",
        "SPEECH_START_THRESHOLD",
        "continuous-{mode}",
    ):
        assert token in calibration
    assert "recommended_environment:" in readiness
    assert "next_command:" in readiness
    assert "voice_calibration_report" in bundle
    assert "logs/voice_calibration_report.json" in bundle
    assert "logs/voice_calibration.env" in bundle
    assert "render_env" in bundle
    assert "recommended_environment" in bundle
    assert "logs/audio_calibration.json" in readme
    assert "voice-calibration-report" in readme
    assert "APPLY_VOICE_CALIBRATION" in readme
    assert "CONTINUOUS_SAMPLE_LOG" in readme
    assert "APPLY_VOICE_CALIBRATION" in continuous
    assert "VOICE_CALIBRATION_ENV_APPLIED" in continuous
    assert "CONTINUOUS_SAMPLE_LOG" in continuous
