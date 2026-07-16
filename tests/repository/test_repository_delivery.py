"""部署入口、模型运行时与展示交付物约束。"""

import re

from repository_test_support import (
    CORE_ROOT,
    ROOT,
    VOICE_FRONTEND_ROOT,
    acceptance_handler_source,
    assert_acceptance_modes,
)


def test_generated_ros_domain_ids_stay_within_fastdds_port_limit():
    """Fast DDS 默认端口映射要求 domain <= 232，所有 PID 取模公式都必须守住上界。"""

    pattern = re.compile(r"\$\(\((\d+) \+ \$\$ % (\d+)\)\)")
    violations = []
    for script in sorted((ROOT / "scripts").glob("*.sh")):
        for base_text, modulus_text in pattern.findall(
            script.read_text(encoding="utf-8")
        ):
            maximum = int(base_text) + int(modulus_text) - 1
            if maximum > 232:
                violations.append(f"{script.name}: maximum={maximum}")
    assert violations == [], violations


def test_integration_probes_are_not_mixed_with_user_scripts():
    misplaced = sorted((ROOT / "scripts").glob("test_*"))
    assert misplaced == [], f"测试探针应放入 tests/integration: {misplaced}"

def test_live_voice_entrypoints_share_one_profile_resolver():
    """普通控制与 Nav2 可以覆盖场景基线，但不能复制四套麦克风 profile。"""
    resolver = ROOT / "scripts" / "voice_control_profile.sh"
    assert resolver.is_file()
    resolver_text = resolver.read_text(encoding="utf-8")
    assert "apply_voice_control_profile_defaults" in resolver_text
    for profile in ("normal", "quiet", "low_gain", "noisy_room"):
        assert profile in resolver_text

    for name, scenario in (
        ("continuous_voice_control.sh", "control"),
        ("continuous_nav2_voice_control.sh", "navigation"),
    ):
        text = (ROOT / "scripts" / name).read_text(encoding="utf-8")
        assert 'source "$WORKSPACE/scripts/voice_control_profile.sh"' in text
        assert f'apply_voice_control_profile_defaults "{scenario}"' in text
        assert 'case "$VOICE_CONTROL_PROFILE"' not in text

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
    present = {path.name for path in (ROOT / "tests").rglob("test_*")}
    assert required <= present

def test_voice_navigation_acceptance_entrypoints_remain_available():
    """语音目标点导航/巡航是当前阶段核心能力，入口脚本不能在整理中丢失。"""

    assert_acceptance_modes(
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
    )

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

    assert_acceptance_modes("asr-nlu-samples-to-eval", "asr-nlu-candidate-eval")
    assert (ROOT / "tools" / "evaluation" / "asr_nlu_samples_to_eval_candidates.py").is_file()
    assert (ROOT / "tools" / "evaluation" / "evaluate_asr_nlu_eval_candidates.py").is_file()
    assert (ROOT / "training" / "robot_instruction_eval.jsonl").is_file()


def test_nav2_showcase_relaxes_progress_check_without_editing_system_params():
    """WSL/Gazebo 低实时率下，项目覆盖应避免默认 0.5m/10s 的进度假失败。"""

    launch = (
        ROOT
        / "src"
        / "embodied_simulation"
        / "launch"
        / "voice_nav2_turtlebot3.launch.py"
    ).read_text(encoding="utf-8")
    package_xml = (
        ROOT / "src" / "embodied_simulation" / "package.xml"
    ).read_text(encoding="utf-8")

    assert "RewrittenYaml" in launch
    assert '"required_movement_radius": nav2_progress_radius' in launch
    assert '"movement_time_allowance": nav2_progress_timeout' in launch
    assert '"stop_on_failure": "true"' in launch
    assert 'DeclareLaunchArgument("nav2_progress_radius", default_value="0.10")' in launch
    assert 'DeclareLaunchArgument("nav2_progress_timeout", default_value="30.0")' in launch
    assert "<exec_depend>nav2_common</exec_depend>" in package_xml
    assert "<exec_depend>nav2_bringup</exec_depend>" in package_xml

def test_sherpa_asr_deployment_entrypoints_remain_available():
    """离线 ASR 真实部署必须有轻量入口，不能只依赖完整离线大脚本。

    setup_offline_runtime.sh 会同时拉 TTS、llama.cpp 和 Qwen GGUF，适合完整离线链路；
    但真实排查 ASR 时需要 ASR-only 预检和单 wav smoke，便于快速确认 sherpa-onnx
    推理框架、ZipFormer 模型文件和项目 provider seam 是否可用。
    """

    assert_acceptance_modes(
        "sherpa-asr-preflight", "sherpa-asr-smoke", "offline-sherpa-typed"
    )

    setup_script = ROOT / "scripts" / "setup_sherpa_asr_runtime.sh"
    smoke_script = ROOT / "scripts" / "sherpa_asr_smoke.py"
    typed_script = ROOT / "scripts" / "smoke_test_offline_sherpa_typed_simulation.sh"
    typed_probe = (
        ROOT
        / "tests"
        / "integration"
        / "voice"
        / "test_offline_sherpa_typed_simulation.py"
    )
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

    assert_acceptance_modes(
        "llama-cpp-preflight", "llama-cpp-smoke", "llama-decode-benchmark"
    )

    start_script = ROOT / "scripts" / "start_llama_server.sh"
    smoke_script = ROOT / "scripts" / "smoke_test_llama_cpp.sh"
    preflight_script = ROOT / "scripts" / "llama_cpp_preflight.py"
    decode_benchmark_script = ROOT / "tools" / "evaluation" / "benchmark_llama_decode_speed.py"
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
    # Lifecycle 重构后 readiness 必须读取强类型状态服务，不能等待易漂移的日志字符串。
    assert "activate_lifecycle_node.py" in script
    assert "--wait-only --timeout 60" in script
    assert "offline agent ready" not in script

def test_instruction_following_lora_review_workflow_remains_available():
    """失败样例只能先进入候选集；人工审核后才允许导出 approved LoRA 数据集。"""

    assert_acceptance_modes(
        "instruction-following-lora-candidates",
        "instruction-following-lora-review",
    )

    candidate_script = ROOT / "tools" / "evaluation" / "export_instruction_following_lora_candidates.py"
    review_script = ROOT / "tools" / "evaluation" / "review_lora_candidates.py"
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

    assert_acceptance_modes(
        "summer-tts-preflight", "summer-tts-smoke", "summer-pseudo-tts",
        "summer-tts-cache-audit",
    )

    setup_script = ROOT / "scripts" / "setup_summer_tts_runtime.sh"
    smoke_script = ROOT / "scripts" / "summer_tts_smoke.py"
    pseudo_script = ROOT / "scripts" / "smoke_test_summer_pseudo_tts.py"
    service_probe = ROOT / "scripts" / "summer_tts_service_probe.py"
    service_smoke = ROOT / "scripts" / "smoke_test_summer_tts_service.sh"
    cache_audit = ROOT / "tools" / "evaluation" / "audit_summer_tts_cache_evidence.py"
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
    assert "SummerTts" in offline_node
    assert "tts_provider" in offline_node
    assert "tts_provider" in offline_launch
    assert "summer_tts_binary" in offline_launch

def test_summer_tts_resident_ros_component_entrypoints_remain_available():
    """SummerTTS 常驻 C++ ROS 组件化入口必须可构建、可验收、可从 Agent 选择。"""

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
    assert_acceptance_modes("summer-tts-service")
    assert "smoke_test_summer_tts_service.sh" in acceptance_handler_source(
        "summer-tts-service"
    )
    assert "summer_ros" in offline_node
    assert "summer_tts_service" in offline_launch

def test_offline_runtime_versions_are_pinned_and_documented():
    """离线运行时必须有固定版本，避免第三方 main 分支漂移破坏演示。"""

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

    assert_acceptance_modes("offline-runtime-versions")
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

    latency_probe = ROOT / "scripts" / "offline_latency_targets.py"
    latency_smoke = ROOT / "scripts" / "smoke_test_offline_latency.sh"
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    testing_doc = (ROOT / "docs" / "TESTING_AND_ACCEPTANCE.md").read_text(
        encoding="utf-8"
    )

    assert_acceptance_modes("offline-latency", "offline-voice-e2e-report")
    assert "smoke_test_offline_latency.sh" in acceptance_handler_source(
        "offline-latency"
    )
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

    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    release_gate = ROOT / "scripts" / "showcase_release_gate.py"
    demo_evidence_checklist = ROOT / "scripts" / "demo_evidence_checklist.py"
    offline_showcase_report = ROOT / "tools" / "evaluation" / "generate_offline_showcase_report.py"
    offline_evidence_audit = ROOT / "tools" / "evaluation" / "audit_offline_showcase_evidence.py"
    eval_validator = ROOT / "tools" / "evaluation" / "validate_instruction_eval_dataset.py"
    parser_eval = ROOT / "tools" / "evaluation" / "evaluate_instruction_parser.py"
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

    assert_acceptance_modes(
        "release-gate",
        "robotics-gate",
        "demo-gate",
        "demo-evidence-checklist",
        "offline-showcase-report",
        "offline-evidence-audit",
        "instruction-eval-dataset",
        "instruction-parser-eval",
        "instruction-following-eval",
        "instruction-following-lora-candidates",
    )
    showcase_handler = acceptance_handler_source("offline-showcase-report")
    audit_handler = acceptance_handler_source("offline-evidence-audit")
    assert "OFFLINE_SHOWCASE_RUN_INSTRUCTION_FOLLOWING" in showcase_handler
    assert "OFFLINE_EVIDENCE_REQUIRE_INSTRUCTION_FOLLOWING" in audit_handler
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
    assert "OFFLINE_SHOWCASE_RUN_LLAMA_BENCH" in showcase_handler
    assert "OFFLINE_EVIDENCE_REQUIRE_LLAMA_BENCH" in audit_handler
    release_gate_text = release_gate.read_text(encoding="utf-8")
    checklist_text = demo_evidence_checklist.read_text(encoding="utf-8")
    assert "job_showcase_release_gate" in release_gate_text
    assert "CORE_COMMANDS" in release_gate_text
    assert "DEMO_COMMANDS" in release_gate_text
    assert "FULL_COMMANDS" in release_gate_text
    assert "ROBOTICS_COMMANDS" in release_gate_text
    assert "\"core\": CORE_COMMANDS" in release_gate_text
    assert "\"demo\": DEMO_COMMANDS" in release_gate_text
    assert "\"robotics\": ROBOTICS_COMMANDS" in release_gate_text
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
    following_eval = (ROOT / "tools" / "evaluation" / "evaluate_instruction_following.py").read_text(
        encoding="utf-8"
    )
    lora_export = (
        ROOT / "tools" / "evaluation" / "export_instruction_following_lora_candidates.py"
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
