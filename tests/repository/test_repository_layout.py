"""仓库结构约束：用户命令与集成测试必须分区，避免 scripts/ 再次退化成杂物箱。"""

import ast
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _python_literal(module_path: Path, name: str):
    tree = ast.parse(module_path.read_text(encoding="utf-8"))
    for node in tree.body:
        if isinstance(node, ast.AnnAssign) and isinstance(node.target, ast.Name):
            if node.target.id == name:
                return ast.literal_eval(node.value)
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name:
                    return ast.literal_eval(node.value)
    raise AssertionError(f"{name} not found in {module_path}")


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
        "test_continuous_navigation_natural.py",
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
        "test_offline_sherpa_typed_simulation.py",
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
        "nav2-stage",
        "nav2-turtlebot3",
        "continuous-nav2-offline",
        "continuous-nav2-online",
        "continuous-nav2-evidence",
        "continuous-nav2-live-check",
        "continuous-navigation",
        "continuous-navigation-natural",
    ):
        assert mode in acceptance

    for script in (
        "smoke_test_navigation_sequence.sh",
        "smoke_test_continuous_navigation_queue.sh",
        "smoke_test_continuous_navigation_natural.sh",
        "smoke_test_nav2_bridge.sh",
        "smoke_test_nav2_preflight.sh",
        "smoke_test_nav2_turtlebot3_voice.sh",
        "continuous_nav2_voice_control.sh",
        "continuous_nav2_voice_evidence.sh",
        "publish_nav2_initial_pose.py",
    ):
        assert (ROOT / "scripts" / script).is_file()

    assert (
        ROOT / "src" / "embodied_simulation" / "launch" / "voice_nav2_turtlebot3.launch.py"
    ).is_file()
    assert (ROOT / "src" / "embodied_simulation" / "config" / "places.yaml").is_file()


def test_sherpa_asr_deployment_entrypoints_remain_available():
    """离线 ASR 真实部署必须有轻量入口，不能只依赖完整离线大脚本。

    setup_offline_runtime.sh 会同时拉 TTS、llama.cpp 和 Qwen GGUF，适合完整离线链路；
    但真实排查 ASR 时需要 ASR-only 预检和单 wav smoke，便于快速确认 sherpa-onnx
    推理框架、ZipFormer 模型文件和项目 provider seam 是否可用。
    """

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    for mode in ("sherpa-asr-preflight", "sherpa-asr-smoke", "offline-sherpa-typed"):
        assert mode in acceptance

    setup_script = ROOT / "scripts" / "setup_sherpa_asr_runtime.sh"
    smoke_script = ROOT / "scripts" / "sherpa_asr_smoke.py"
    typed_script = ROOT / "scripts" / "smoke_test_offline_sherpa_typed_simulation.sh"
    typed_probe = ROOT / "tests" / "integration" / "test_offline_sherpa_typed_simulation.py"
    assert setup_script.is_file()
    assert smoke_script.is_file()
    assert typed_script.is_file()
    assert typed_probe.is_file()

    setup_text = setup_script.read_text(encoding="utf-8")
    smoke_text = smoke_script.read_text(encoding="utf-8")
    typed_text = typed_probe.read_text(encoding="utf-8")
    assert "sherpa-onnx==" in setup_text
    assert "k2fsa-zipformer-bilingual-zh-en-t" in setup_text
    assert "setup_offline_runtime.sh" in setup_text
    assert "SherpaZipformerAsr" in smoke_text
    assert "--preflight-only" in smoke_text
    assert "SherpaVitsTts" in typed_text
    assert "/robot/action_command_typed" in typed_text
    assert "/cmd_vel" in typed_text
    assert "RobotCommand.MOVE" in typed_text


def test_llama_cpp_deployment_entrypoints_remain_available():
    """llama.cpp 离线推理必须能独立预检，不能只能挂在完整离线验收里排查。"""

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    for mode in ("llama-cpp-preflight", "llama-cpp-smoke"):
        assert mode in acceptance

    start_script = ROOT / "scripts" / "start_llama_server.sh"
    smoke_script = ROOT / "scripts" / "smoke_test_llama_cpp.sh"
    preflight_script = ROOT / "scripts" / "llama_cpp_preflight.py"
    provider = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "providers"
        / "llama_cpp.py"
    )
    assert start_script.is_file()
    assert smoke_script.is_file()
    assert preflight_script.is_file()
    assert provider.is_file()

    start_text = start_script.read_text(encoding="utf-8")
    preflight_text = preflight_script.read_text(encoding="utf-8")
    provider_text = provider.read_text(encoding="utf-8")
    assert "LLAMA_EXTRA_ARGS" in start_text
    assert "/v1/chat/completions" in preflight_text
    assert "LlamaCppMetrics" in provider_text
    assert "timeout_s" in provider_text


def test_summer_tts_deployment_entrypoints_remain_available():
    """SummerTTS 是独立 C++ 离线 TTS 后端，必须能单独部署和验收。"""

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    for mode in ("summer-tts-preflight", "summer-tts-smoke", "summer-pseudo-tts"):
        assert mode in acceptance

    setup_script = ROOT / "scripts" / "setup_summer_tts_runtime.sh"
    smoke_script = ROOT / "scripts" / "summer_tts_smoke.py"
    pseudo_script = ROOT / "scripts" / "smoke_test_summer_pseudo_tts.py"
    provider = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "providers"
        / "summer_tts.py"
    )
    offline_node = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py"
    ).read_text(encoding="utf-8")
    offline_launch = (
        ROOT / "src" / "embodied_offline_agent" / "launch" / "offline_agent.launch.py"
    ).read_text(encoding="utf-8")

    assert setup_script.is_file()
    assert smoke_script.is_file()
    assert pseudo_script.is_file()
    assert provider.is_file()

    setup_text = setup_script.read_text(encoding="utf-8")
    provider_text = provider.read_text(encoding="utf-8")
    assert "huakunyang/SummerTTS" in setup_text
    assert "patch_missing_cstdint" in setup_text
    assert "tts_test" in provider_text
    assert "SummerTts" in offline_node
    assert "tts_provider" in offline_node
    assert "tts_provider" in offline_launch
    assert "summer_tts_binary" in offline_launch


def test_summer_tts_resident_ros_component_entrypoints_remain_available():
    """SummerTTS 常驻 C++ ROS 组件化入口必须可构建、可验收、可从 Agent 选择。"""

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    cpp_cmake = (ROOT / "src" / "embodied_agent_cpp" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    offline_node = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py"
    ).read_text(encoding="utf-8")
    offline_launch = (
        ROOT / "src" / "embodied_offline_agent" / "launch" / "offline_agent.launch.py"
    ).read_text(encoding="utf-8")

    assert (ROOT / "src" / "embodied_agent_interfaces" / "srv" / "SynthesizeSpeech.srv").is_file()
    srv_text = (
        ROOT / "src" / "embodied_agent_interfaces" / "srv" / "SynthesizeSpeech.srv"
    ).read_text(encoding="utf-8")
    assert (
        ROOT
        / "src"
        / "embodied_agent_cpp"
        / "include"
        / "embodied_agent_cpp"
        / "summer_tts_service_node.hpp"
    ).is_file()
    assert (
        ROOT / "src" / "embodied_agent_cpp" / "src" / "summer_tts_service_node.cpp"
    ).is_file()
    assert "summer_tts_component" in cpp_cmake
    assert "rclcpp_components_register_nodes" in cpp_cmake
    assert "summer_tts_service" in cpp_cmake
    assert "bool cache_hit" in srv_text
    assert "cache_enabled" in (
        ROOT / "src" / "embodied_agent_cpp" / "src" / "summer_tts_service_node.cpp"
    ).read_text(encoding="utf-8")
    assert "cache_hit" in (ROOT / "scripts" / "summer_tts_service_probe.py").read_text(
        encoding="utf-8"
    )
    assert "summer-tts-service" in acceptance
    assert "summer_tts_service_probe.py" in acceptance or "smoke_test_summer_tts_service.sh" in acceptance
    assert "summer_ros" in offline_node
    assert "summer_tts_service" in offline_launch


def test_offline_runtime_versions_are_pinned_and_documented():
    """离线运行时必须有固定版本，避免第三方 main 分支漂移破坏演示。"""

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    setup_offline = (ROOT / "scripts" / "setup_offline_runtime.sh").read_text(
        encoding="utf-8"
    )
    setup_summer = (ROOT / "scripts" / "setup_summer_tts_runtime.sh").read_text(
        encoding="utf-8"
    )
    version_probe = ROOT / "scripts" / "offline_runtime_versions.py"
    version_doc = ROOT / "docs" / "OFFLINE_RUNTIME_VERSIONS.md"
    version_text = version_probe.read_text(encoding="utf-8")
    doc_text = version_doc.read_text(encoding="utf-8")

    expected_llama = "0eca4d490e591d4e93058d07540cf47278a72577"
    expected_summer = "c90e0e8d31e09c98199ab9b5a605af74c179f811"
    expected_sherpa = "1.13.3"

    assert "offline-runtime-versions" in acceptance
    assert version_probe.is_file()
    assert version_doc.is_file()
    assert expected_llama in setup_offline
    assert expected_summer in setup_summer
    assert expected_sherpa in setup_offline
    for expected in (expected_llama, expected_summer, expected_sherpa):
        assert expected in version_text
        assert expected in doc_text


def test_offline_latency_gate_remains_available_and_documented():
    """LLM/TTS 延迟目标必须有可执行 gate，不能只停留在 README 声明。"""

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    latency_probe = ROOT / "scripts" / "offline_latency_targets.py"
    latency_smoke = ROOT / "scripts" / "smoke_test_offline_latency.sh"
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    testing_doc = (ROOT / "docs" / "TESTING_AND_ACCEPTANCE.md").read_text(
        encoding="utf-8"
    )

    assert "offline-latency" in acceptance
    assert "smoke_test_offline_latency.sh" in acceptance
    assert latency_probe.is_file()
    assert latency_smoke.is_file()
    probe_text = latency_probe.read_text(encoding="utf-8")
    assert "LLM_FIRST_TOKEN_TARGET_MS = 1000.0" in probe_text
    assert "TTS_FIRST_AUDIO_TARGET_MS = 300.0" in probe_text
    assert "--tts-provider" in probe_text
    assert "offline-latency" in readme
    assert "≤ 1000ms" in readme
    assert "≤ 300ms" in readme
    assert "SummerTTS 命令行 provider" in testing_doc
    assert "tts_provider:=summer_ros" in testing_doc


def test_job_presentation_doc_remains_discoverable():
    """求职展示版必须有稳定的汇报入口，方便按代码讲完整链路。"""

    presentation = ROOT / "docs" / "PROJECT_PRESENTATION_15MIN.md"
    diagrams = ROOT / "docs" / "FINAL_ARCHITECTURE_DIAGRAMS.md"
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    presentation_text = presentation.read_text(encoding="utf-8")
    diagrams_text = diagrams.read_text(encoding="utf-8")

    assert presentation.is_file()
    assert diagrams.is_file()
    assert "PROJECT_PRESENTATION_15MIN.md" in readme
    assert "FINAL_ARCHITECTURE_DIAGRAMS.md" in readme
    assert "15 分钟项目汇报" in presentation_text
    assert "FINAL_ARCHITECTURE_DIAGRAMS.md" in presentation_text
    assert "从语音输入到仿真执行的代码走读地图" in presentation_text
    assert "最终系统架构图" in diagrams_text
    assert "端到端数据流图" in diagrams_text
    assert "flowchart TB" in diagrams_text
    assert "sequenceDiagram" in diagrams_text
    assert "C++ ActionGuard" in diagrams_text
    assert "typed_action_demo_client" in diagrams_text
    assert "offline_showcase_report" in diagrams_text
    for required in (
        "continuous-offline",
        "continuous-multi-command",
        "ActionGuard",
        "BehaviorTree",
        "SummerTTS",
    ):
        assert required in presentation_text


def test_showcase_hardening_artifacts_remain_discoverable():
    """缺点收口阶段的展示硬化产物不能在后续整理中丢失。"""

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    release_gate = ROOT / "scripts" / "showcase_release_gate.py"
    offline_showcase_report = ROOT / "scripts" / "generate_offline_showcase_report.py"
    eval_validator = ROOT / "scripts" / "validate_instruction_eval_dataset.py"
    parser_eval = ROOT / "scripts" / "evaluate_instruction_parser.py"
    eval_dataset = ROOT / "training" / "robot_instruction_eval.jsonl"
    interview_doc = ROOT / "docs" / "INTERVIEW_QA.md"
    gaps_doc = ROOT / "docs" / "PROJECT_GAPS_AND_OPTIMIZATION.md"
    benchmark_doc = ROOT / "docs" / "OFFLINE_BENCHMARK_REPORT.md"

    for path in (
        release_gate,
        offline_showcase_report,
        eval_validator,
        parser_eval,
        eval_dataset,
        interview_doc,
        gaps_doc,
        benchmark_doc,
    ):
        assert path.is_file()

    assert "release-gate" in acceptance
    assert "demo-gate" in acceptance
    assert "offline-showcase-report" in acceptance
    assert "instruction-eval-dataset" in acceptance
    assert "instruction-parser-eval" in acceptance
    assert "logs/acceptance_report.json" in readme
    assert "logs/demo_acceptance_report.json" in readme
    assert "离线模型 Benchmark 与展示报告" in readme
    offline_report_text = offline_showcase_report.read_text(encoding="utf-8")
    assert "offline_deployment_showcase" in offline_report_text
    assert "model_inventory" in offline_report_text
    assert "runtime_versions" in offline_report_text
    assert "instruction_parser" in offline_report_text
    release_gate_text = release_gate.read_text(encoding="utf-8")
    assert "job_showcase_release_gate" in release_gate_text
    assert "CORE_COMMANDS" in release_gate_text
    assert "DEMO_COMMANDS" in release_gate_text
    assert "FULL_COMMANDS" in release_gate_text
    assert "\"core\": CORE_COMMANDS" in release_gate_text
    assert "\"demo\": DEMO_COMMANDS" in release_gate_text
    assert "command_count" in release_gate_text
    assert "instruction_parser_eval" in release_gate_text
    assert "tag_accuracy" in parser_eval.read_text(encoding="utf-8")
    assert "source_counts" in parser_eval.read_text(encoding="utf-8")
    assert "failed_cases" in parser_eval.read_text(encoding="utf-8")
    assert "ActionGuard" in interview_doc.read_text(encoding="utf-8")
    assert "真实语音稳定性" in gaps_doc.read_text(encoding="utf-8")
    assert "evaluate_instruction_following.sh" in benchmark_doc.read_text(encoding="utf-8")
    dataset_lines = [
        line
        for line in eval_dataset.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    assert len(dataset_lines) >= 24
    dataset_text = "\n".join(dataset_lines)
    assert "expected_actions" in dataset_text
    assert '"safety"' in dataset_text
    assert '"navigation"' in dataset_text
    assert '"asr_noise"' in dataset_text


def test_nav2_live_evidence_script_keeps_control_and_scoring_together():
    """一键现场留证脚本必须同时启动控制链路与 live-check，并保存可复核报告。"""

    evidence = (ROOT / "scripts" / "continuous_nav2_voice_evidence.sh").read_text(
        encoding="utf-8"
    )

    assert "continuous_nav2_voice_control.sh" in evidence
    assert "continuous_live_check.py" in evidence
    assert "CONTINUOUS_LIVE_CHECK_REPORT" in evidence
    assert "--output \"$REPORT_PATH\"" in evidence
    assert "--require-candidate navigate_to" in evidence
    assert "--require-candidate follow_waypoints" in evidence
    assert "CONTINUOUS_NAV2_EVIDENCE_DRY_RUN" in evidence
    assert "LIVE_CHECK_ARGS" in evidence
    assert "DRY RUN" in evidence
    assert "export ROS_DOMAIN_ID" in evidence
    assert "kill -TERM -- \"-$CONTROL_PID\"" in evidence


def test_voice_navigation_places_stay_consistent_across_agent_guard_and_nav2():
    """Agent、ActionGuard、Nav2 executor 必须共享同一组标准地点名。

    语音导航链路跨 Python Agent、C++ 安全网关和 Nav2 坐标配置。任何一层漏掉
    某个 canonical place，都会导致“ASR/LLM 已解析，但机器人不执行”的现场事故。
    这里不限制中文别名，只锁住标准地点名和默认巡航点。
    """

    navigation_phrases = (
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "navigation_phrases.py"
    )
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


def test_nav2_executor_failure_details_are_preserved():
    """Nav2 失败原因必须从 executor 透传到本项目 typed action。

    语音目标点导航现场排障最怕只看到 `blocked` 或 `executor_rejected`。这个结构测试锁住
    external detail seam：Nav2 插件负责记录 server unavailable、goal rejected、missed
    waypoints 等细节，simulation_control_node 负责把细节写进 feedback/result。
    """

    executor_header = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "include"
        / "embodied_simulation"
        / "robot_executor.hpp"
    ).read_text(encoding="utf-8")
    executor_plugin = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "src"
        / "robot_executor_plugins.cpp"
    ).read_text(encoding="utf-8")
    control_node = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "src"
        / "simulation_control_node.cpp"
    ).read_text(encoding="utf-8")

    assert "external_action_detail()" in executor_header
    for detail_token in (
        "server_unavailable",
        "goal_rejected",
        "goal_response_timeout_or_rejected",
        "missed_waypoints",
        "error_msg",
        "nav2:cancel_requested",
    ):
        assert detail_token in executor_plugin
    assert "executor_->external_action_detail()" in control_node
    assert "detail.empty() ? \"executor_rejected\" : detail" in control_node
    assert "Nav2 这类外部 action 的失败原因" in control_node


def test_webrtc_vad_sidecar_remains_integrated_as_optional_voice_provider():
    """真实麦克风稳定性不能只依赖 energy VAD。

    WebRTC VAD 是本项目的轻量成熟 provider：依赖比 Silero 小，适合 WSL/笔记本演示；
    这里锁住 entry point、launch 条件、preflight auto fallback 和文档入口。
    """

    setup_py = (ROOT / "src" / "embodied_online_agent" / "setup.py").read_text(
        encoding="utf-8"
    )
    sidecar = (
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "silero_vad_sidecar.py"
    ).read_text(encoding="utf-8")
    node_path = (
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "webrtc_vad_node.py"
    )
    online_launch = (
        ROOT / "src" / "embodied_online_agent" / "launch" / "online_agent.launch.py"
    ).read_text(encoding="utf-8")
    offline_launch = (
        ROOT / "src" / "embodied_offline_agent" / "launch" / "offline_agent.launch.py"
    ).read_text(encoding="utf-8")
    preflight = (ROOT / "scripts" / "voice_provider_preflight.py").read_text(
        encoding="utf-8"
    )
    continuous = (ROOT / "scripts" / "continuous_voice_control.sh").read_text(
        encoding="utf-8"
    )
    acceptance_doc = (ROOT / "docs" / "TESTING_AND_ACCEPTANCE.md").read_text(
        encoding="utf-8"
    )

    assert node_path.is_file()
    assert "webrtc-vad" in setup_py
    assert "webrtc_vad = embodied_online_agent.webrtc_vad_node:main" in setup_py
    assert "class WebRtcVadProvider" in sidecar
    assert "WebRTC VAD frame_ms must be one of [10, 20, 30]" in sidecar
    assert "executable=\"webrtc_vad\"" in online_launch
    assert "executable=\"webrtc_vad\"" in offline_launch
    assert "' != 'silero' and '" in online_launch
    assert "' != 'silero' and '" in offline_launch
    assert "vad:auto_fallback:webrtc" in preflight
    assert "webrtcvad_package_missing" in preflight
    assert "auto 会优先 Silero，其次 WebRTC，最后降级 energy" in continuous
    assert "pip install webrtcvad" in acceptance_doc


def test_audio_calibration_outputs_copyable_live_demo_advice():
    """真实麦克风校准必须产出可复制的下一步命令。"""

    calibration = (ROOT / "scripts" / "audio_frontend_calibration.py").read_text(
        encoding="utf-8"
    )
    readiness = (ROOT / "scripts" / "voice_control_readiness_check.py").read_text(
        encoding="utf-8"
    )
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    acceptance_doc = (ROOT / "docs" / "TESTING_AND_ACCEPTANCE.md").read_text(
        encoding="utf-8"
    )

    for token in (
        "recommended_environment",
        "next_command",
        "SPEECH_START_THRESHOLD",
        "continuous-{mode}",
    ):
        assert token in calibration
    assert "recommended_environment:" in readiness
    assert "next_command:" in readiness
    assert "logs/audio_calibration.json" in readme
    assert "recommended_environment" in acceptance_doc


def test_cpp_typed_action_demo_client_remains_available():
    """ROS2/C++ 求职展示必须保留独立 rclcpp_action client 示例。"""

    cmake = (ROOT / "src" / "embodied_agent_cpp" / "CMakeLists.txt").read_text(
        encoding="utf-8"
    )
    demo_client_path = (
        ROOT
        / "src"
        / "embodied_agent_cpp"
        / "src"
        / "typed_action_demo_client.cpp"
    )
    demo_client = demo_client_path.read_text(encoding="utf-8")
    smoke_script = ROOT / "scripts" / "smoke_test_cpp_action_client.sh"
    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    learning = (ROOT / "docs" / "LEARNING_NOTES.md").read_text(encoding="utf-8")
    presentation = (ROOT / "docs" / "PROJECT_PRESENTATION_15MIN.md").read_text(
        encoding="utf-8"
    )

    assert demo_client_path.is_file()
    assert smoke_script.is_file()
    assert "add_executable(typed_action_demo_client" in cmake
    assert "typed_action_demo_client" in cmake
    assert "rclcpp_action::create_client<ExecuteRobotCommand>" in demo_client
    assert "feedback_callback" in demo_client
    assert "result_callback" in demo_client
    assert "cpp-action-client" in acceptance
    assert "typed_action_demo_client" in smoke_script.read_text(encoding="utf-8")
    assert "typed_action_demo_client.cpp" in learning
    assert "typed_action_demo_client.cpp" in presentation
