"""仓库结构约束：用户命令与集成测试必须分区，避免 scripts/ 再次退化成杂物箱。"""

import ast
import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CORE_ROOT = ROOT / "src" / "embodied_agent_core" / "embodied_agent_core"
VOICE_FRONTEND_ROOT = (
    ROOT / "src" / "embodied_voice_frontend" / "embodied_voice_frontend"
)


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


def test_agent_core_is_the_one_way_shared_dependency():
    """公共实现与输入 Adapter 必须独立，禁止 online/offline 相互依赖。"""

    core_package = ROOT / "src" / "embodied_agent_core"
    assert (core_package / "package.xml").is_file()
    assert (core_package / "setup.py").is_file()
    assert (core_package / "config" / "command_normalization_zh.yaml").is_file()
    assert (core_package / "prompts" / "system_prompt_zh.txt").is_file()
    voice_package = ROOT / "src" / "embodied_voice_frontend"
    assert (voice_package / "package.xml").is_file()
    assert (voice_package / "setup.py").is_file()

    core_source = "\n".join(
        path.read_text(encoding="utf-8") for path in CORE_ROOT.glob("*.py")
    )
    assert "embodied_online_agent" not in core_source
    assert "embodied_offline_agent" not in core_source

    for package_name, node_name in (
        ("embodied_online_agent", "online_agent_node.py"),
        ("embodied_offline_agent", "offline_agent_node.py"),
    ):
        package_root = ROOT / "src" / package_name
        manifest = (package_root / "package.xml").read_text(encoding="utf-8")
        node = (package_root / package_name / node_name).read_text(encoding="utf-8")
        assert "<exec_depend>embodied_agent_core</exec_depend>" in manifest
        assert "<exec_depend>embodied_voice_frontend</exec_depend>" in manifest
        other_agent = (
            "embodied_offline_agent"
            if package_name == "embodied_online_agent"
            else "embodied_online_agent"
        )
        assert f"<exec_depend>{other_agent}</exec_depend>" not in manifest
        assert "from embodied_agent_core.agent_control_plane import" in node
        assert "from embodied_agent_core.agent_lifecycle_runtime import" in node

    # 删除旧实现而非保留转发 shim，确保公共逻辑只有一个权威位置。
    old_online_root = (
        ROOT / "src" / "embodied_online_agent" / "embodied_online_agent"
    )
    for moved_module in (
        "agent_control_plane.py",
        "agent_lifecycle_runtime.py",
        "agent_ros_io.py",
        "continuous_voice.py",
        "ros_event_transport.py",
    ):
        assert not (old_online_root / moved_module).exists()
        assert (CORE_ROOT / moved_module).is_file()

    for adapter in (
        "keyword_wake_node.py",
        "silero_vad_node.py",
        "speaker_identity_node.py",
        "webrtc_vad_node.py",
    ):
        assert not (old_online_root / adapter).exists()
        assert (VOICE_FRONTEND_ROOT / adapter).is_file()


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
        "test_asr_nlu_samples_to_eval_candidates.py",
        "test_evaluate_asr_nlu_eval_candidates.py",
        "test_continuous_kws_sidecar.py",
        "test_voice_provider_preflight.py",
        "test_audio_frontend_calibration.py",
        "test_typed_action_server.py",
        "test_navigation_sequence.py",
        "test_nav2_bridge_sequence.py",
        "test_nav2_turtlebot3_voice.py",
        "test_offline_sherpa_typed_simulation.py",
        "test_agent_lifecycle.py",
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
        "nav2-assets",
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
        "audit_nav2_demo_assets.py",
        "continuous_nav2_voice_control.sh",
        "continuous_nav2_voice_evidence.sh",
        "publish_nav2_initial_pose.py",
    ):
        assert (ROOT / "scripts" / script).is_file()

    assert (
        ROOT / "src" / "embodied_simulation" / "launch" / "voice_nav2_turtlebot3.launch.py"
    ).is_file()
    assert (ROOT / "src" / "embodied_simulation" / "config" / "places.yaml").is_file()
    assert (ROOT / "src" / "embodied_simulation" / "maps" / "voice_demo.yaml").is_file()
    assert (ROOT / "src" / "embodied_simulation" / "maps" / "voice_demo.pgm").is_file()
    assert (ROOT / "src" / "embodied_simulation" / "rviz" / "voice_nav2_demo.rviz").is_file()
    assert (
        ROOT / "src" / "embodied_simulation" / "worlds" / "voice_demo.sdf.xacro"
    ).is_file()


def test_real_asr_sample_eval_loop_entrypoints_remain_available():
    """真实 ASR 错词/多命令样本要能从现场日志沉淀为评估候选集。"""

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    assert "asr-nlu-samples-to-eval" in acceptance
    assert "asr-nlu-candidate-eval" in acceptance
    assert (ROOT / "scripts" / "asr_nlu_samples_to_eval_candidates.py").is_file()
    assert (ROOT / "scripts" / "evaluate_asr_nlu_eval_candidates.py").is_file()
    assert (ROOT / "training" / "robot_instruction_eval.jsonl").is_file()


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
    for mode in ("llama-cpp-preflight", "llama-cpp-smoke", "llama-decode-benchmark"):
        assert mode in acceptance

    start_script = ROOT / "scripts" / "start_llama_server.sh"
    smoke_script = ROOT / "scripts" / "smoke_test_llama_cpp.sh"
    preflight_script = ROOT / "scripts" / "llama_cpp_preflight.py"
    decode_benchmark_script = ROOT / "scripts" / "benchmark_llama_decode_speed.py"
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
    assert decode_benchmark_script.is_file()
    assert provider.is_file()

    start_text = start_script.read_text(encoding="utf-8")
    preflight_text = preflight_script.read_text(encoding="utf-8")
    provider_text = provider.read_text(encoding="utf-8")
    assert "LLAMA_EXTRA_ARGS" in start_text
    assert "/v1/chat/completions" in preflight_text
    assert "llama-bench" in decode_benchmark_script.read_text(encoding="utf-8")
    assert "LlamaCppMetrics" in provider_text
    assert "timeout_s" in provider_text


def test_robot_action_transport_is_fully_typed_without_legacy_json_adapter():
    """控制接口必须保持 typed；JSON 只能用于日志/报告，不能重新进入机器人链路。"""

    interfaces = ROOT / "src" / "embodied_agent_interfaces" / "msg"
    for name in (
        "RobotCommand.msg",
        "RobotCommandFeedback.msg",
        "RobotCommandResult.msg",
    ):
        assert (interfaces / name).is_file()

    cpp = ROOT / "src" / "embodied_agent_cpp"
    guard = (cpp / "src" / "action_guard_node.cpp").read_text(encoding="utf-8")
    bridge = (cpp / "src" / "typed_action_bridge_node.cpp").read_text(
        encoding="utf-8"
    )
    sequencer = (CORE_ROOT / "action_sequence.py").read_text(encoding="utf-8")

    assert "Subscription<\n    embodied_agent_interfaces::msg::RobotCommand>" in guard
    assert "RobotCommandFeedback" in bridge
    assert "RobotCommandResult" in bridge
    assert "ActionScheduler" in bridge
    assert '"/diagnostics"' in bridge
    assert "async_cancel_goal" in bridge
    assert "nlohmann/json" not in bridge
    assert "json.loads" not in sequencer
    assert "_legacy_results" not in sequencer
    assert not (cpp / "src" / "robot_command_adapter.cpp").exists()
    assert not (cpp / "include" / "embodied_agent_cpp" / "robot_command_adapter.hpp").exists()
    assert (cpp / "include" / "embodied_agent_cpp" / "action_scheduler.hpp").is_file()
    assert (cpp / "src" / "action_scheduler.cpp").is_file()
    assert (cpp / "test" / "test_action_scheduler.cpp").is_file()

    command_msg = (interfaces / "RobotCommand.msg").read_text(encoding="utf-8")
    assert "bool priority" in command_msg
    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(encoding="utf-8")
    assert "cpp-action-scheduler" in acceptance


def test_offline_voice_e2e_reuses_an_existing_llama_server():
    """聚合报告连续跑 latency/E2E 时不能重复绑定 8080 端口。"""
    script = (ROOT / "scripts" / "smoke_test_offline_voice_real.sh").read_text(
        encoding="utf-8"
    )

    assert 'SERVER_PID=""' in script
    assert 'if ! curl -fsS "$BASE_URL/health"' in script
    assert '[[ -z "$SERVER_PID" ]] || kill "$SERVER_PID"' in script
    assert 'memory_path:="$MEMORY_DIR/conversation.json"' in script
    assert 'user_memory_dir:="$MEMORY_DIR/users"' in script
    assert "/agent/clear_memory" not in script


def test_agent_parameter_contract_has_one_authoritative_schema():
    """在线/离线节点不得重新复制默认参数表，组合 launch 也必须复用转发契约。"""

    online_package = ROOT / "src" / "embodied_online_agent"
    schema = CORE_ROOT / "agent_parameters.py"
    launch_contract = CORE_ROOT / "agent_launch_contract.py"
    assert schema.is_file()
    assert launch_contract.is_file()

    for node_path in (
        online_package / "embodied_online_agent" / "online_agent_node.py",
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py",
    ):
        text = node_path.read_text(encoding="utf-8")
        assert "declare_agent_parameters" in text
        assert "self._agent_parameters =" in text
        assert "self._parameters = declare_agent_parameters" not in text
        assert "def _declare_parameters" not in text

    for launch_path in (
        ROOT / "src" / "embodied_simulation" / "launch" / "voice_turtlebot3.launch.py",
        ROOT
        / "src"
        / "embodied_simulation"
        / "launch"
        / "voice_nav2_turtlebot3.launch.py",
    ):
        text = launch_path.read_text(encoding="utf-8")
        assert "declare_forwarded_agent_arguments" in text
        assert "forwarded_agent_launch_arguments" in text
        assert 'DeclareLaunchArgument("continuous_command_queue_size"' not in text


def test_provider_yaml_does_not_duplicate_control_plane_defaults():
    """YAML 只描述 provider；公共控制面默认值由 agent_parameters.py 管理。"""

    forbidden = (
        "continuous_command_queue_size:",
        "voice_session_timeout_s:",
        "command_normalization_fuzzy_threshold:",
        "memory_max_turns:",
    )
    for profile in (
        ROOT / "src" / "embodied_online_agent" / "config" / "online_agent.yaml",
        ROOT / "src" / "embodied_offline_agent" / "config" / "offline_agent.yaml",
    ):
        text = profile.read_text(encoding="utf-8")
        for key in forbidden:
            assert key not in text


def test_parameter_smoke_uses_bounded_rclpy_client_instead_of_ros2cli_daemon():
    probe = ROOT / "scripts" / "ros_parameter_check.py"
    smoke = ROOT / "scripts" / "smoke_test_online_wake_config.sh"
    assert probe.is_file()
    probe_text = probe.read_text(encoding="utf-8")
    smoke_text = smoke.read_text(encoding="utf-8")
    assert "AsyncParameterClient" in probe_text
    assert "spin_until_future_complete" in probe_text
    assert "--timeout 15" in smoke_text
    assert "ros2 param get" not in smoke_text


def test_instruction_following_lora_review_workflow_remains_available():
    """失败样例只能先进入候选集；人工审核后才允许导出 approved LoRA 数据集。"""

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    for mode in (
        "instruction-following-lora-candidates",
        "instruction-following-lora-review",
    ):
        assert mode in acceptance

    candidate_script = ROOT / "scripts" / "export_instruction_following_lora_candidates.py"
    review_script = ROOT / "scripts" / "review_lora_candidates.py"
    dataset_info = (ROOT / "training" / "dataset_info.json").read_text(
        encoding="utf-8"
    )
    training_readme = (ROOT / "training" / "README.md").read_text(encoding="utf-8")

    assert candidate_script.is_file()
    assert review_script.is_file()
    assert "review_required" in candidate_script.read_text(encoding="utf-8")
    review_text = review_script.read_text(encoding="utf-8")
    assert "lora_candidate_review" in review_text
    assert "lora_approved_dataset_export" in review_text
    assert "robot_dialogue_lora_approved" in dataset_info
    assert "instruction-following-lora-review" in training_readme


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
    service_probe = ROOT / "scripts" / "summer_tts_service_probe.py"
    service_smoke = ROOT / "scripts" / "smoke_test_summer_tts_service.sh"
    cache_audit = ROOT / "scripts" / "audit_summer_tts_cache_evidence.py"
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
    assert service_probe.is_file()
    assert service_smoke.is_file()
    assert cache_audit.is_file()
    assert provider.is_file()

    setup_text = setup_script.read_text(encoding="utf-8")
    provider_text = provider.read_text(encoding="utf-8")
    service_probe_text = service_probe.read_text(encoding="utf-8")
    service_smoke_text = service_smoke.read_text(encoding="utf-8")
    cache_audit_text = cache_audit.read_text(encoding="utf-8")
    assert "huakunyang/SummerTTS" in setup_text
    assert "patch_missing_cstdint" in setup_text
    assert "tts_test" in provider_text
    assert "--require-cache-hit" in service_probe_text
    assert "--require-cache-hit" in service_smoke_text
    assert "summer_tts_cache_evidence_audit" in cache_audit_text
    assert "summer-tts-cache-audit" in acceptance
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
    """组件延迟和真实 Agent E2E 必须分开测量，不能把整句合成冒充首音频。"""

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
    assert "offline-voice-e2e-report" in acceptance
    assert "smoke_test_offline_latency.sh" in acceptance
    assert latency_probe.is_file()
    assert latency_smoke.is_file()
    probe_text = latency_probe.read_text(encoding="utf-8")
    assert "LLM_FIRST_TOKEN_TARGET_MS = 1000.0" in probe_text
    assert "TTS_SYNTHESIS_TARGET_MS = 600.0" in probe_text
    assert '"measurement_kind": "full_utterance_synthesis"' in probe_text
    assert "--tts-provider" in probe_text
    assert "offline-latency" in readme
    assert "≤ 1000ms" in readme
    assert "≤ 600ms" in readme
    assert "offline-voice-e2e-report" in readme
    assert "SummerTTS 命令行 provider" in testing_doc
    assert "tts_provider:=summer_ros" in testing_doc


def test_job_presentation_doc_remains_discoverable():
    """求职展示版必须有稳定的汇报入口，方便按代码讲完整链路。"""

    presentation = ROOT / "docs" / "PROJECT_PRESENTATION_15MIN.md"
    diagrams = ROOT / "docs" / "FINAL_ARCHITECTURE_DIAGRAMS.md"
    walkthrough = ROOT / "docs" / "VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md"
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    learning_notes = (ROOT / "docs" / "LEARNING_NOTES.md").read_text(encoding="utf-8")
    presentation_text = presentation.read_text(encoding="utf-8")
    diagrams_text = diagrams.read_text(encoding="utf-8")
    walkthrough_text = walkthrough.read_text(encoding="utf-8")

    assert presentation.is_file()
    assert diagrams.is_file()
    assert walkthrough.is_file()
    assert "PROJECT_PRESENTATION_15MIN.md" in readme
    assert "FINAL_ARCHITECTURE_DIAGRAMS.md" in readme
    assert "VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md" in readme
    assert "VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md" in learning_notes
    assert "VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md" in presentation_text
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
    assert "语音输入到仿真执行" in walkthrough_text
    assert "ContinuousCommandQueue" in walkthrough_text
    assert "CommandNLU.parse()" in walkthrough_text
    assert "ExecuteRobotCommand" in walkthrough_text
    assert "evidence_kind" in walkthrough_text
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
    demo_evidence_checklist = ROOT / "scripts" / "demo_evidence_checklist.py"
    offline_showcase_report = ROOT / "scripts" / "generate_offline_showcase_report.py"
    offline_evidence_audit = ROOT / "scripts" / "audit_offline_showcase_evidence.py"
    eval_validator = ROOT / "scripts" / "validate_instruction_eval_dataset.py"
    parser_eval = ROOT / "scripts" / "evaluate_instruction_parser.py"
    eval_dataset = ROOT / "training" / "robot_instruction_eval.jsonl"
    interview_doc = ROOT / "docs" / "INTERVIEW_QA.md"
    gaps_doc = ROOT / "docs" / "PROJECT_GAPS_AND_OPTIMIZATION.md"
    benchmark_doc = ROOT / "docs" / "OFFLINE_BENCHMARK_REPORT.md"

    for path in (
        release_gate,
        demo_evidence_checklist,
        offline_showcase_report,
        offline_evidence_audit,
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
    assert "demo-evidence-checklist" in acceptance
    assert "offline-showcase-report" in acceptance
    assert "offline-evidence-audit" in acceptance
    assert "instruction-eval-dataset" in acceptance
    assert "instruction-parser-eval" in acceptance
    assert "instruction-following-eval" in acceptance
    assert "instruction-following-lora-candidates" in acceptance
    assert "OFFLINE_SHOWCASE_RUN_INSTRUCTION_FOLLOWING" in acceptance
    assert "OFFLINE_EVIDENCE_REQUIRE_INSTRUCTION_FOLLOWING" in acceptance
    assert "logs/acceptance_report.json" in readme
    assert "logs/demo_acceptance_report.json" in readme
    assert "离线模型 Benchmark 与展示报告" in readme
    offline_report_text = offline_showcase_report.read_text(encoding="utf-8")
    assert "offline_deployment_showcase" in offline_report_text
    assert "model_inventory" in offline_report_text
    assert "runtime_versions" in offline_report_text
    assert "instruction_parser" in offline_report_text
    audit_text = offline_evidence_audit.read_text(encoding="utf-8")
    assert "offline_showcase_evidence_audit" in audit_text
    assert "claim_guidance" in audit_text
    assert "不要说：LoRA" in audit_text
    assert "require_llama_bench" in audit_text
    assert "OFFLINE_SHOWCASE_RUN_LLAMA_BENCH" in acceptance
    assert "OFFLINE_EVIDENCE_REQUIRE_LLAMA_BENCH" in acceptance
    release_gate_text = release_gate.read_text(encoding="utf-8")
    checklist_text = demo_evidence_checklist.read_text(encoding="utf-8")
    assert "job_showcase_release_gate" in release_gate_text
    assert "CORE_COMMANDS" in release_gate_text
    assert "DEMO_COMMANDS" in release_gate_text
    assert "FULL_COMMANDS" in release_gate_text
    assert "\"core\": CORE_COMMANDS" in release_gate_text
    assert "\"demo\": DEMO_COMMANDS" in release_gate_text
    assert "command_count" in release_gate_text
    assert "job_showcase_demo_evidence_checklist" in checklist_text
    assert "voice_stability_preflight" in checklist_text
    assert "mature_vad_active" in checklist_text
    assert "continuous-live-check" in checklist_text
    assert "demo_recording.mp4" in checklist_text
    assert "instruction_parser_eval" in release_gate_text
    assert "tag_accuracy" in parser_eval.read_text(encoding="utf-8")
    assert "source_counts" in parser_eval.read_text(encoding="utf-8")
    assert "failed_cases" in parser_eval.read_text(encoding="utf-8")
    following_eval = (ROOT / "scripts" / "evaluate_instruction_following.py").read_text(
        encoding="utf-8"
    )
    lora_export = (
        ROOT / "scripts" / "export_instruction_following_lora_candidates.py"
    ).read_text(encoding="utf-8")
    assert "offline_llm_instruction_following_eval" in following_eval
    assert "minimum_effective" in following_eval
    assert "instruction_following_lora_candidate_export" in lora_export
    assert "review_required" in lora_export
    assert "ActionGuard" in interview_doc.read_text(encoding="utf-8")
    assert "真实语音稳定性" in gaps_doc.read_text(encoding="utf-8")
    assert "instruction-following-eval" in benchmark_doc.read_text(encoding="utf-8")
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
    action_runtime = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "src"
        / "active_action_runtime.cpp"
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
    assert "Nav2 这类外部 action 的失败原因" in action_runtime


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
    preflight = (ROOT / "scripts" / "voice_provider_preflight.py").read_text(
        encoding="utf-8"
    )
    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
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
    assert "silero-vad" in setup_py
    assert "webrtc_vad = embodied_voice_frontend.webrtc_vad_node:main" in setup_py
    assert "class WebRtcVadProvider" in sidecar
    assert "class SileroOnnxVadProvider" in sidecar
    assert "WebRTC VAD frame_ms must be one of [10, 20, 30]" in sidecar
    assert "executable=\"webrtc_vad\"" in online_launch
    assert "executable=\"webrtc_vad\"" in offline_launch
    assert "' != 'silero' and '" in online_launch
    assert "' != 'silero' and '" in offline_launch
    assert "vad:auto_fallback:webrtc" in preflight
    assert "webrtcvad_package_missing" in preflight
    assert "auto 会优先 Silero，其次 WebRTC，最后降级 energy" in continuous
    assert (ROOT / "scripts" / "setup_voice_vad_runtime.sh").is_file()
    assert (ROOT / "scripts" / "silero_onnx_smoke.py").is_file()
    assert (ROOT / "scripts" / "silero_ros_runtime_probe.py").is_file()
    assert (ROOT / "scripts" / "smoke_test_silero_vad_runtime.sh").is_file()
    assert (ROOT / "scripts" / "smoke_test_webrtc_vad_sidecar.sh").is_file()
    assert "voice-vad-runtime-dry-run" in acceptance
    assert "webrtc-vad-sidecar" in acceptance
    assert "silero-vad-runtime" in acceptance
    assert "setup_voice_vad_runtime.sh webrtc" in acceptance_doc
    assert "webrtc-vad-sidecar" in acceptance_doc
    assert "silero-vad-runtime" in acceptance_doc
    assert "embodied_voice_frontend[webrtc-vad]" in acceptance_doc


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
    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    acceptance_doc = (ROOT / "docs" / "TESTING_AND_ACCEPTANCE.md").read_text(
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
    assert "voice-kws-runtime-dry-run" in acceptance
    assert "sherpa-kws-sidecar" in acceptance
    assert "setup_voice_kws_runtime.sh openwakeword" in acceptance_doc
    assert "setup_voice_kws_runtime.sh sherpa" in acceptance_doc
    assert "source logs/sherpa_kws.env" in acceptance_doc
    assert "sherpa-kws-sidecar" in acceptance_doc


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


def test_voice_control_events_are_strongly_typed_and_use_named_qos():
    """语音控制面属于中间件契约，禁止退回 String + JSON。"""

    interfaces = ROOT / "src" / "embodied_agent_interfaces" / "msg"
    online = (
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "online_agent_node.py"
    ).read_text(encoding="utf-8")
    offline = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py"
    ).read_text(encoding="utf-8")
    transport = (CORE_ROOT / "ros_event_transport.py").read_text(encoding="utf-8")
    qos = (CORE_ROOT / "ros_qos.py").read_text(encoding="utf-8")
    event_adapter = (CORE_ROOT / "ros_agent_events.py").read_text(encoding="utf-8")
    ros_io = (CORE_ROOT / "agent_ros_io.py").read_text(encoding="utf-8")
    topics = (CORE_ROOT / "ros_topics.py").read_text(encoding="utf-8")

    assert (interfaces / "CommandContext.msg").is_file()
    assert (interfaces / "CommandQueueEvent.msg").is_file()
    assert (interfaces / "CommandExecutionEvent.msg").is_file()
    assert (interfaces / "WakeEvent.msg").is_file()
    assert (interfaces / "RecognitionFeedback.msg").is_file()
    assert (interfaces / "TextRewrite.msg").is_file()
    assert (interfaces / "CommandSlot.msg").is_file()
    assert (interfaces / "NluCommand.msg").is_file()
    assert (interfaces / "NluParseEvent.msg").is_file()
    for node in (online, offline):
        assert "AgentRosIo" in node
        assert 'String, "/agent/command_queue"' not in node
        assert 'String, "/agent/command_execution"' not in node
        assert 'String, "/agent/wake_event"' not in node
        assert 'String, "/agent/recognition_feedback"' not in node
    for field, topic in (
        ("command_queue", "/agent/command_queue"),
        ("command_execution", "/agent/command_execution"),
        ("wake_event", "/agent/wake_event"),
        ("recognition_feedback", "/agent/recognition_feedback"),
        ("nlu_parse", "/agent/nlu_parse"),
    ):
        assert f"self._topics.{field}" in event_adapter
        assert topic in topics
    assert "event_qos()" in event_adapter
    assert "state_qos()" in event_adapter
    assert "RosAgentEventPublisher" in ros_io
    assert "audio_qos" in ros_io
    assert "event_qos" in ros_io
    assert "unsupported queue event" in transport
    assert "unsupported wake event" in transport
    assert "unsupported recognition feedback" in transport
    assert "nlu_parse_to_message" in transport
    assert "ReliabilityPolicy.RELIABLE" in qos
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in qos

    # 节点只能依赖共享 I/O Facade；硬编码接口名必须收敛到 Topic contract。
    for node in (online, offline):
        assert '"/agent/' not in node
        assert '"/audio/' not in node
        assert '"/robot/' not in node


def test_runtime_status_topics_are_strongly_typed():
    """运行状态也是跨进程契约，不能退回各节点自行拼装 JSON。"""
    interfaces = ROOT / "src" / "embodied_agent_interfaces" / "msg"
    for name in (
        "AudioFrontendStatus.msg",
        "VadEvent.msg",
        "KwsEvent.msg",
        "KwsScore.msg",
        "SimulationState.msg",
        "RobotActionAck.msg",
        "BehaviorTreeStatus.msg",
        "ComponentHealth.msg",
        "SystemReadiness.msg",
    ):
        assert (interfaces / name).is_file()

    online = ROOT / "src" / "embodied_online_agent" / "embodied_online_agent"
    runtime_transport = (CORE_ROOT / "runtime_status_transport.py").read_text(
        encoding="utf-8"
    )
    assert "action_ack_to_dict" in runtime_transport
    assert "simulation_state_to_dict" in runtime_transport
    assert not (CORE_ROOT / "wake_event_input.py").exists()

    sources = [
        VOICE_FRONTEND_ROOT / "keyword_wake_node.py",
        VOICE_FRONTEND_ROOT / "silero_vad_node.py",
        VOICE_FRONTEND_ROOT / "webrtc_vad_node.py",
        ROOT / "src" / "embodied_agent_cpp" / "src" / "audio_frontend_node.cpp",
        ROOT / "src" / "embodied_simulation" / "src" / "simulation_control_node.cpp",
    ]
    combined = "\n".join(path.read_text(encoding="utf-8") for path in sources)
    for topic in (
        "/agent/kws_event",
        "/agent/kws_score",
        "/audio/vad_event",
        "/audio/frontend_metrics",
        "robot/action_ack",
        "robot/simulation_state",
        "robot/bt_status",
    ):
        assert topic in combined
    assert 'create_publisher(String, "/audio/vad_event"' not in combined
    assert 'create_publisher(String, "/agent/kws_event"' not in combined


def test_speaker_memory_has_one_deep_module_and_typed_transport():
    interfaces = ROOT / "src" / "embodied_agent_interfaces" / "msg"
    for name in (
        "SpeakerIdentity.msg",
        "SpeakerEnrollRequest.msg",
        "SpeakerEnrollStatus.msg",
    ):
        assert (interfaces / name).is_file()
    online_root = ROOT / "src" / "embodied_online_agent" / "embodied_online_agent"
    service = (CORE_ROOT / "memory_command_service.py").read_text(encoding="utf-8")
    context_runtime = (CORE_ROOT / "user_context_runtime.py").read_text(
        encoding="utf-8"
    )
    transport = (CORE_ROOT / "speaker_transport.py").read_text(encoding="utf-8")
    online = (online_root / "online_agent_node.py").read_text(encoding="utf-8")
    offline = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py"
    ).read_text(encoding="utf-8")
    sidecar = (VOICE_FRONTEND_ROOT / "speaker_identity_node.py").read_text(
        encoding="utf-8"
    )

    assert "class MemoryCommandService" in service
    assert "def handle(" in service
    assert "class UserContextRuntime" in context_runtime
    assert "class UserContextSnapshot" in context_runtime
    for node in (online, offline):
        assert "UserContextRuntime" in node
        assert "MemoryCommandService" not in node
        assert "parse_memory_command" not in node
        assert "def _record_user_interaction" not in node
        assert "def _current_user_preferences" not in node
        assert "def _system_prompt_with_user_memory" not in node
        assert "def _actions_from_context" not in node
        assert 'String, "/agent/speaker_identity"' not in node
        assert 'String, "/agent/speaker_enroll_request"' not in node
    assert "identity_message_to_domain" in transport
    assert "SpeakerEnrollStatus" in sidecar
    assert "json.loads(message.data)" not in sidecar


def test_simulation_action_lifecycle_has_one_runtime_owner():
    """长动作状态应由深模块统一拥有，Lifecycle 节点只做 ROS 装配。"""
    package = ROOT / "src" / "embodied_simulation"
    header = package / "include" / "embodied_simulation" / "active_action_runtime.hpp"
    source = package / "src" / "active_action_runtime.cpp"
    node = (package / "src" / "simulation_control_node.cpp").read_text(
        encoding="utf-8"
    )

    assert header.is_file()
    assert source.is_file()
    assert "ActiveActionRuntime" in node
    assert "action_runtime_->update" in node
    assert "std::optional<ActionExecution> action_execution_" not in node
    assert "active_action_uses_external_result_" not in node


def test_action_guard_buffers_only_the_dds_startup_window():
    """控制命令不能因 discovery 竞态丢失，也不能以 transient-local 重放旧动作。"""
    package = ROOT / "src" / "embodied_agent_cpp"
    header = package / "include" / "embodied_agent_cpp" / "guarded_command_outbox.hpp"
    node = (package / "src" / "action_guard_node.cpp").read_text(encoding="utf-8")

    assert header.is_file()
    assert "GuardedCommandOutbox" in node
    assert "downstream_wait_timeout_s" in node
    assert "get_subscription_count() > 0" in node
    assert "transient_local" not in node.lower()


def test_python_and_cpp_nodes_share_one_qos_vocabulary():
    middleware = ROOT / "src" / "embodied_agent_middleware"
    qos_header = (
        middleware / "include" / "embodied_agent_middleware" / "qos_profiles.hpp"
    )
    qos_text = qos_header.read_text(encoding="utf-8")
    python_qos = (CORE_ROOT / "ros_qos.py").read_text(encoding="utf-8")
    for profile in (
        "command_qos",
        "event_qos",
        "state_qos",
        "sensor_qos",
        "audio_qos",
        "diagnostics_qos",
    ):
        assert profile in qos_text
        assert f"def {profile}(" in python_qos

    sources = (
        ROOT / "src" / "embodied_agent_cpp" / "src" / "action_guard_node.cpp",
        ROOT / "src" / "embodied_agent_cpp" / "src" / "typed_action_bridge_node.cpp",
        ROOT / "src" / "embodied_agent_cpp" / "src" / "audio_frontend_node.cpp",
        ROOT / "src" / "embodied_simulation" / "src" / "simulation_control_node.cpp",
    )
    for source in sources:
        text = source.read_text(encoding="utf-8")
        assert "embodied_agent_middleware/qos_profiles.hpp" in text
    assert "rclcpp::QoS(10)" not in sources[-1].read_text(encoding="utf-8")

    # Voice Adapter 只能选择项目语义，禁止重新散落 History/Reliability 魔法配置。
    for source in VOICE_FRONTEND_ROOT.glob("*_node.py"):
        text = source.read_text(encoding="utf-8")
        assert "QoSProfile" not in text
        assert "HistoryPolicy" not in text
        assert "ReliabilityPolicy" not in text
        assert "from embodied_agent_core.ros_qos import" in text


def test_system_readiness_is_typed_profile_based_and_heartbeat_driven():
    middleware = ROOT / "src" / "embodied_agent_middleware"
    registry = (
        middleware / "include" / "embodied_agent_middleware" / "component_health_registry.hpp"
    )
    aggregator = middleware / "src" / "system_readiness_node.cpp"
    assert registry.is_file()
    text = aggregator.read_text(encoding="utf-8")
    assert "required_components_csv" in text
    assert "stale_timeout_s" in text
    assert "SystemReadiness" in text

    launch = (
        ROOT / "src" / "embodied_simulation" / "launch" / "simulation_control.launch.py"
    ).read_text(encoding="utf-8")
    assert "system_readiness_node" in launch
    assert "readiness_required_components" in launch

    check = (ROOT / "scripts" / "system_readiness_check.py").read_text(
        encoding="utf-8"
    )
    assert '"/system/readiness"' in check


def test_online_and_offline_agents_have_real_lifecycle_resource_ownership():
    online_root = ROOT / "src" / "embodied_online_agent"
    offline_root = ROOT / "src" / "embodied_offline_agent"
    online_node = (
        online_root / "embodied_online_agent" / "online_agent_node.py"
    ).read_text(encoding="utf-8")
    offline_node = (
        offline_root / "embodied_offline_agent" / "offline_agent_node.py"
    ).read_text(encoding="utf-8")
    ros_io = (CORE_ROOT / "agent_ros_io.py").read_text(encoding="utf-8")
    lifecycle_runtime = (CORE_ROOT / "agent_lifecycle_runtime.py").read_text(
        encoding="utf-8"
    )

    for class_name, node in (
        ("OnlineAgentNode", online_node),
        ("OfflineAgentNode", offline_node),
    ):
        assert f"class {class_name}(LifecycleNode)" in node
        for callback in (
            "on_configure",
            "on_activate",
            "on_deactivate",
            "on_cleanup",
            "on_shutdown",
            "on_error",
        ):
            assert f"def {callback}(" in node
        assert "AgentRosIo" in node
        assert "AgentLifecycleRuntime" in node
        assert "self._runtime.activate(" in node
        assert "self._runtime.deactivate(" in node
        assert "self._runtime.release(" in node
        assert "self._lifecycle_active =" not in node
        assert "self._execution =" not in node
        assert "self._asr_endpoint =" not in node
        assert "start_background_turn(" in node
        assert "threading.Thread(\n            target=self._run_turn" not in node

    assert "create_lifecycle_publisher" in ros_io
    assert "create_subscription" in ros_io
    for ordered_step in (
        'cancel("lifecycle_deactivated")',
        "command_queue.clear()",
        "_publish_priority_stop()",
        "stop_input()",
        "_execution.stop(",
        'publish_stopped("lifecycle_inactive")',
    ):
        assert ordered_step in lifecycle_runtime
    assert (
        ROOT / "src" / "embodied_agent_core" / "test" / "test_agent_lifecycle_runtime.py"
    ).is_file()

    online_launch = (online_root / "launch" / "online_agent.launch.py").read_text(
        encoding="utf-8"
    )
    offline_launch = (
        offline_root / "launch" / "offline_agent.launch.py"
    ).read_text(encoding="utf-8")
    assert '["action_guard", "online_agent"]' in online_launch
    assert '["action_guard", "offline_agent"]' in offline_launch
    for launch in (online_launch, offline_launch):
        assert '"agent_lifecycle_autostart": False' in launch

    acceptance = (ROOT / "scripts" / "acceptance_test.sh").read_text(
        encoding="utf-8"
    )
    assert "agent-lifecycle" in acceptance
    assert (ROOT / "scripts" / "smoke_test_agent_lifecycle.sh").is_file()
    assert (ROOT / "tests" / "integration" / "test_agent_lifecycle.py").is_file()


def test_agent_turn_metrics_use_one_strongly_typed_ros_contract():
    """在线/离线指标必须共享 schema，禁止重新引入双 topic 或 JSON wire。"""

    interface = (
        ROOT
        / "src"
        / "embodied_agent_interfaces"
        / "msg"
        / "AgentTurnMetrics.msg"
    )
    ros_io = (CORE_ROOT / "agent_ros_io.py").read_text(encoding="utf-8")
    transport = (CORE_ROOT / "metrics_transport.py").read_text(encoding="utf-8")
    online = (
        ROOT
        / "src"
        / "embodied_online_agent"
        / "embodied_online_agent"
        / "online_agent_node.py"
    ).read_text(encoding="utf-8")
    offline = (
        ROOT
        / "src"
        / "embodied_offline_agent"
        / "embodied_offline_agent"
        / "offline_agent_node.py"
    ).read_text(encoding="utf-8")
    live_check = (ROOT / "scripts" / "continuous_live_check.py").read_text(
        encoding="utf-8"
    )

    assert interface.is_file()
    schema = interface.read_text(encoding="utf-8")
    assert "TARGET_UNKNOWN" in schema
    assert "llm_decode_tokens_per_s" in schema
    assert "tts_first_text_to_first_audio_ms" in schema
    assert "AgentTurnMetrics" in ros_io
    assert "String(data=payload)" not in ros_io
    assert "agent_turn_metrics_to_message" in online
    assert "agent_turn_metrics_to_message" in offline
    assert "agent_turn_metrics_message_to_dict" in live_check
    assert '"/agent/metrics"' in live_check
    assert "diagnostics_qos(depth=10)" in live_check
    assert "_finite_or_nan" in transport

    # 显式扫描保证新增脚本也受守卫约束；拆开 legacy 字符串，避免测试命中自身。
    legacy_metrics_topic = "/offline_agent" + "/metrics"
    tracked = "\n".join(
        path.read_text(encoding="utf-8", errors="ignore")
        for root in (
            ROOT / "src" / "embodied_online_agent",
            ROOT / "src" / "embodied_offline_agent",
            ROOT / "scripts",
            ROOT / "tests",
            ROOT / "docs",
        )
        for path in root.rglob("*")
        if path.is_file() and path.suffix in {".py", ".sh", ".md"}
    )
    assert legacy_metrics_topic not in tracked
