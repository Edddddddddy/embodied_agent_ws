"""验收模式的单一事实源：公开接口、分类和底层处理函数。"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class HandlerDomain(str, Enum):
    """验收 handler 的显式所有权；mode 名称不再参与运行时路由。"""

    CONTROL = "control"
    VOICE = "voice"
    SLAM_NAV = "slam_nav"
    EVALUATION = "evaluation"

    @property
    def handler_library(self) -> str:
        return {
            HandlerDomain.CONTROL: "tools/acceptance/handlers/control.sh",
            HandlerDomain.VOICE: "tools/acceptance/handlers/voice.sh",
            HandlerDomain.SLAM_NAV: "tools/acceptance/handlers/slam_nav.sh",
            HandlerDomain.EVALUATION: "tools/evaluation/acceptance_handlers.sh",
        }[self]


@dataclass(frozen=True, slots=True)
class AcceptanceMode:
    name: str
    description: str
    handler: str
    category: str
    public: bool
    requires_ros_environment: bool
    domain: HandlerDomain

    @property
    def handler_library(self) -> str:
        return self.domain.handler_library


PUBLIC_MODE_NAMES = (
    'core',
    'continuous-offline',
    'continuous-online',
    'gazebo',
    'nav2-stage',
    'slam-nav-e2e',
    'unknown-world-slam-e2e',
    'voice-unknown-world-slam-e2e',
    'robotics-gate',
)

# 一行描述一个路由；复杂执行细节不再与公开 CLI/帮助耦合。
_MODE_SPECS = (
    ('core', 'Typical developer gate: repository, Python unit, C++ unit tests', 'accept_core', 'public', True, True, HandlerDomain.CONTROL),
    ('architecture-facts', 'Verify generated package/CI/CLI/release-gate architecture evidence', 'accept_architecture_facts', 'internal', False, False, HandlerDomain.CONTROL),
    ('control-authority-stage', 'Typed authority and autonomy/keyboard velocity allowlist gate', 'accept_control_authority_stage', 'internal', False, True, HandlerDomain.CONTROL),
    ('agent-lifecycle', 'Online/offline configure -> activate -> deactivate -> reactivate', 'accept_agent_lifecycle', 'internal', False, True, HandlerDomain.CONTROL),
    ('preflight', 'Check offline model/runtime files', 'accept_preflight', 'internal', False, True, HandlerDomain.CONTROL),
    ('mock', 'Build, unit tests, and dependency-free ROS smokes', 'accept_mock', 'internal', False, True, HandlerDomain.CONTROL),
    ('online', 'Minimal-token live ASR/LLM/TTS verification', 'accept_online', 'internal', False, True, HandlerDomain.VOICE),
    ('offline', 'Real ZipFormer/llama.cpp/Sherpa-TTS verification', 'accept_offline', 'internal', False, True, HandlerDomain.VOICE),
    ('offline-runtime-versions', 'Check pinned llama.cpp/SummerTTS/sherpa-onnx versions', 'accept_offline_runtime_versions', 'internal', False, True, HandlerDomain.VOICE),
    ('offline-voice-e2e-report', 'Real ZipFormer -> llama.cpp -> pseudo-streaming TTS latency evidence', 'accept_offline_voice_e2e_report', 'internal', False, True, HandlerDomain.VOICE),
    ('offline-showcase-report', 'Generate offline deployment JSON/Markdown evidence report', 'accept_offline_showcase_report', 'internal', False, True, HandlerDomain.VOICE),
    ('offline-evidence-audit', 'Audit offline report evidence and over-claiming boundaries', 'accept_offline_evidence_audit', 'internal', False, True, HandlerDomain.VOICE),
    ('offline-latency', 'Check llama.cpp first token and Sherpa short-sentence synthesis latency', 'accept_offline_latency', 'internal', False, True, HandlerDomain.VOICE),
    ('llama-cpp-preflight', 'Check llama.cpp binary/model plus llama-server health/models API', 'accept_llama_cpp_preflight', 'internal', False, True, HandlerDomain.VOICE),
    ('llama-cpp-smoke', 'Low-token llama.cpp streaming chat verification', 'accept_llama_cpp_smoke', 'internal', False, True, HandlerDomain.VOICE),
    ('llama-decode-benchmark', 'Measure llama.cpp CPU decode tokens/s with llama-bench', 'accept_llama_decode_benchmark', 'internal', False, True, HandlerDomain.EVALUATION),
    ('summer-tts-preflight', 'Check SummerTTS source/binary/model runtime files', 'accept_summer_tts_preflight', 'internal', False, True, HandlerDomain.VOICE),
    ('summer-tts-smoke', 'Real SummerTTS synthesis verification', 'accept_summer_tts_smoke', 'internal', False, True, HandlerDomain.VOICE),
    ('summer-pseudo-tts', 'Real SummerTTS + pseudo-streaming double-buffer verification', 'accept_summer_pseudo_tts', 'internal', False, True, HandlerDomain.VOICE),
    ('summer-tts-service', 'Resident C++ ROS SummerTTS service smoke', 'accept_summer_tts_service', 'internal', False, True, HandlerDomain.VOICE),
    ('summer-tts-cache-audit', 'Audit SummerTTS short-feedback cache latency evidence', 'accept_summer_tts_cache_audit', 'internal', False, True, HandlerDomain.VOICE),
    ('pseudo-tts', 'Dependency-free llama-style stream + pseudo TTS pipeline smoke', 'accept_pseudo_tts', 'internal', False, True, HandlerDomain.VOICE),
    ('sherpa-asr-preflight', 'ASR-only check: sherpa_onnx import + ZipFormer model files', 'accept_sherpa_asr_preflight', 'internal', False, True, HandlerDomain.VOICE),
    ('sherpa-asr-smoke', 'ASR-only real decode on bundled ZipFormer test wav', 'accept_sherpa_asr_smoke', 'internal', False, True, HandlerDomain.VOICE),
    ('offline-sherpa-typed', 'Real Sherpa ASR/TTS + llama.cpp through typed Action simulation', 'accept_offline_sherpa_typed', 'internal', False, True, HandlerDomain.VOICE),
    ('demo', 'Rich mock demo: ordered actions, accessories, and arc motion', 'accept_demo', 'internal', False, True, HandlerDomain.CONTROL),
    ('navigation-demo', 'Voice-style target navigation and multi-waypoint patrol smoke', 'accept_navigation_demo', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('nav2-bridge', 'Voice navigation commands are converted to Nav2 action goals', 'accept_nav2_bridge', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('nav2-preflight', 'Check Nav2/TurtleBot3 voice launch dependencies and arguments', 'accept_nav2_preflight', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('nav2-assets', 'Audit Nav2 voice demo places/RViz/launch assets', 'accept_nav2_assets', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('nav2-stage', 'Stage gate for voice navigation/patrol; excludes heavy Gazebo/Nav2', 'accept_nav2_stage', 'public', True, True, HandlerDomain.SLAM_NAV),
    ('nav2-turtlebot3', 'Heavy Gazebo/Nav2 run: voice text drives target navigation/patrol', 'accept_nav2_turtlebot3', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('nav2-resilience', 'Heavy Gazebo/Nav2 run: dynamic replan + unreachable failure', 'accept_nav2_resilience', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('mapping-stage', 'Build/test/audit the controlled-drift SLAM mapping baseline', 'accept_mapping_stage', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('slam-nav-showcase-stage', 'Audit realistic scene, semantic goals, and mapping/navigation stages', 'accept_slam_nav_showcase_stage', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('slam-nav-showcase', 'Heavy realistic apartment + AMCL + Nav2 motion gate', 'accept_slam_nav_showcase', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('slam-nav-showcase-mapping', 'Heavy realistic apartment + SLAM map-save gate', 'accept_slam_nav_showcase_mapping', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('slam-session-orchestrator-stage', 'Typed single-terminal mapping/save/navigation FSM gate', 'accept_slam_session_orchestrator_stage', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('slam-autonomous-mission-stage', 'One intent -> exploration/save/localization/navigation FSM gate', 'accept_slam_autonomous_mission_stage', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('slam-nav-e2e', 'Known-world deterministic SLAM/Nav2 regression (legacy stable entry)', 'accept_slam_nav_e2e', 'public', True, True, HandlerDomain.SLAM_NAV),
    ('unknown-world-slam-e2e', 'Unknown-world autonomous exploration -> SLAM -> AMCL/Nav2 evidence gate', 'accept_unknown_world_slam_e2e', 'public', True, True, HandlerDomain.SLAM_NAV),
    ('voice-unknown-world-slam-e2e', '{offline|online} Live microphone -> full-evidence unknown-world SLAM/Nav2 gate', 'accept_voice_unknown_world_slam_e2e', 'public', True, True, HandlerDomain.SLAM_NAV),
    ('showcase-gazebo-e2e', 'Persistent Gazebo/RViz mapping -> localization -> navigation evidence gate', 'accept_showcase_gazebo_e2e', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('slam-session-orchestrator', 'Heavy one-terminal Gazebo mapping/save/restart/navigation gate', 'accept_slam_session_orchestrator', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('slam-benchmark', 'Heavy Gazebo run: fixed loop, 5 cm map, and drift metrics report', 'accept_slam_benchmark', 'internal', False, True, HandlerDomain.EVALUATION),
    ('slam-gtsam-benchmark', 'Heavy Gazebo run with the project GTSAM ScanSolver plugin', 'accept_slam_gtsam_benchmark', 'internal', False, True, HandlerDomain.EVALUATION),
    ('slam-ab-benchmark', 'Run Ceres/GTSAM on the same scenario and compare evidence', 'accept_slam_ab_benchmark', 'internal', False, True, HandlerDomain.EVALUATION),
    ('slam-navigation', 'Heavy run: saved map -> AMCL -> Nav2 plan -> goal execution', 'accept_slam_navigation', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('slam-evaluation-stage', 'Synthetic ATE/RPE/loop-correction gate without Gazebo or downloads', 'accept_slam_evaluation_stage', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-groundtruth', 'Download and verify one public OpenLORIS ground-truth trajectory', 'accept_openloris_groundtruth', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-rosbag-setup', 'Resume and verify a tar-range or standalone public rosbag', 'accept_openloris_rosbag_setup', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-sequence-ranking', 'Rank all ground-truth trajectories before large bag download', 'accept_openloris_sequence_ranking', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-long-loop-evidence', 'Verified corridor1-1 long-loop GTSAM/frontend experiment', 'accept_openloris_long_loop_evidence', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-robust-kernel-ablation', 'Re-optimize one fixed graph with none/Huber/Cauchy', 'accept_openloris_robust_kernel_ablation', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-loop-consistency-ablation', 'Reject geometrically inconsistent non-local graph edges', 'accept_openloris_loop_consistency_ablation', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-scan-overlap-ablation', 'Validate accepted constraints with no-GT scan overlap evidence', 'accept_openloris_scan_overlap_ablation', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-scan-overlap-multisequence', 'Aggregate independent fixed-graph overlap ablations', 'accept_openloris_scan_overlap_multisequence', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-lidar-loop-candidates', 'Evaluate C++ LiDAR loop retrieval on two real sequences', 'accept_openloris_lidar_loop_candidates', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-lidar-shadow-matches', 'Evaluate C++ shadow scan matching on two real sequences', 'accept_openloris_lidar_shadow_matches', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-lidar-submap-ablation', 'Compare scan-to-scan and local-submap shadow matching', 'accept_openloris_lidar_submap_ablation', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('lidar-loop-runtime', 'Lifecycle scan -> submap verification -> shadow gate -> guarded Karto adapter', 'accept_lidar_loop_runtime', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('openloris-replay-stage', 'Generate a tiny bag and replay it through Ceres/GTSAM SLAM', 'accept_openloris_replay_stage', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-bag-preflight', 'Validate OPENLORIS_BAG topics, frames, and optional runtime', 'accept_openloris_bag_preflight', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-slam-ceres', 'Replay a real OpenLORIS bag through the Ceres backend', 'accept_openloris_slam_ceres', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-slam-gtsam', 'Replay a real OpenLORIS bag through the GTSAM backend', 'accept_openloris_slam_gtsam', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-slam-ab', 'Run both backends on one bag and compare their reports', 'accept_openloris_slam_ab', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-loop-evidence', 'Audit office1-7 revisits and run accepted-loop evaluation', 'accept_openloris_loop_evidence', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-loop-sweep', 'Build a lossless topic subset and sweep loop-front-end thresholds', 'accept_openloris_loop_sweep', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('openloris-evaluate', 'Evaluate SLAM_ESTIMATE_FILE against an OpenLORIS sequence', 'accept_openloris_evaluate', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('dynamic-obstacle-stage', 'Build/test tracker, motion predictor, and Nav2 costmap plugin seam', 'accept_dynamic_obstacle_stage', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('dynamic-obstacle-ablation', 'Compare current-only/CV/Kalman/IMM on one fixed C++ scenario', 'accept_dynamic_obstacle_ablation', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('dynamic-obstacle-navigation', 'Heavy run: predicted crossing obstacle -> Nav2 replan -> goal', 'accept_dynamic_obstacle_navigation', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('dynamic-obstacle-navigation-ablation', 'Heavy four-model Gazebo/Nav2 crossing comparison', 'accept_dynamic_obstacle_navigation_ablation', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('continuous-mock', 'One wake word, several queued commands, and sleep gate', 'accept_continuous_mock', 'internal', False, True, HandlerDomain.VOICE),
    ('continuous-soak', 'Long wake session keeps accepting many queued commands', 'accept_continuous_soak', 'internal', False, True, HandlerDomain.VOICE),
    ('continuous-endpoint', 'Endpoint speech_ended commits feed continuous ASR commands', 'accept_continuous_endpoint', 'internal', False, True, HandlerDomain.VOICE),
    ('continuous-multi-command', 'NLU parses one ASR final into ordered queued commands', 'accept_continuous_multi_command', 'internal', False, True, HandlerDomain.VOICE),
    ('continuous-navigation', 'Multi-target navigation and patrol commands queue in a continuous session', 'accept_continuous_navigation', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('continuous-navigation-natural', 'Natural multi-target speech becomes waypoint patrol commands', 'accept_continuous_navigation_natural', 'internal', False, True, HandlerDomain.SLAM_NAV),
    ('continuous-queue-full', 'Busy continuous queue rejects excess commands with feedback', 'accept_continuous_queue_full', 'internal', False, True, HandlerDomain.VOICE),
    ('continuous-ttl', 'Busy continuous queue expires stale non-priority commands', 'accept_continuous_ttl', 'internal', False, True, HandlerDomain.VOICE),
    ('continuous-timeout', 'Voice session timeout requires a fresh wake word', 'accept_continuous_timeout', 'internal', False, True, HandlerDomain.VOICE),
    ('continuous-kws-mock', 'KWS sidecar opens a continuous session and executes a command', 'accept_continuous_kws_mock', 'internal', False, True, HandlerDomain.VOICE),
    ('speaker-memory-mock', 'Speaker identity and per-user memory smoke test', 'accept_speaker_memory_mock', 'internal', False, True, HandlerDomain.VOICE),
    ('speaker-enroll', 'Speaker enrollment request saves wav samples and speakers.txt', 'accept_speaker_enroll', 'internal', False, True, HandlerDomain.VOICE),
    ('speaker-runtime', 'Real sherpa speaker embedding self-match and ambiguity guard', 'accept_speaker_runtime', 'internal', False, True, HandlerDomain.VOICE),
    ('vad-sidecar', 'Dependency-free Silero VAD sidecar seam smoke test', 'accept_vad_sidecar', 'internal', False, True, HandlerDomain.VOICE),
    ('silero-vad-runtime', 'Real lightweight Silero ONNX inference and latency report', 'accept_silero_vad_runtime', 'internal', False, True, HandlerDomain.VOICE),
    ('webrtc-vad-sidecar', 'Installed WebRTC VAD sidecar runtime smoke test', 'accept_webrtc_vad_sidecar', 'internal', False, True, HandlerDomain.VOICE),
    ('kws-sidecar', 'Dependency-free keyword wake sidecar seam smoke test', 'accept_kws_sidecar', 'internal', False, True, HandlerDomain.VOICE),
    ('sherpa-kws-sidecar', 'Installed Sherpa-ONNX KWS runtime startup smoke test', 'accept_sherpa_kws_sidecar', 'internal', False, True, HandlerDomain.VOICE),
    ('openwakeword-sidecar', 'Dependency-free openWakeWord adapter runtime smoke test', 'accept_openwakeword_sidecar', 'internal', False, True, HandlerDomain.VOICE),
    ('livekit-sidecar', 'Dependency-free LiveKit WakeWord adapter runtime smoke test', 'accept_livekit_sidecar', 'internal', False, True, HandlerDomain.VOICE),
    ('kws-calibration', 'Dependency-free KWS score calibration smoke test', 'accept_kws_calibration', 'internal', False, True, HandlerDomain.VOICE),
    ('voice-readiness', 'Dependency-free voice readiness smoke test', 'accept_voice_readiness', 'internal', False, True, HandlerDomain.VOICE),
    ('provider-preflight', 'Optional VAD/KWS provider unit tests plus current-env preflight', 'accept_provider_preflight', 'internal', False, True, HandlerDomain.VOICE),
    ('voice-stability-preflight', 'Strict preflight requiring Silero/WebRTC mature VAD', 'accept_voice_stability_preflight', 'internal', False, True, HandlerDomain.VOICE),
    ('voice-vad-runtime-dry-run', 'Show optional WebRTC/Silero VAD install commands without installing', 'accept_voice_vad_runtime_dry_run', 'internal', False, True, HandlerDomain.VOICE),
    ('voice-kws-runtime-dry-run', 'Show optional openWakeWord/sherpa KWS install commands without installing', 'accept_voice_kws_runtime_dry_run', 'internal', False, True, HandlerDomain.VOICE),
    ('voice-calibration-report', 'Generate voice profile/threshold calibration report', 'accept_voice_calibration_report', 'internal', False, True, HandlerDomain.VOICE),
    ('instruction-eval-dataset', 'Validate lightweight robot instruction eval dataset', 'accept_instruction_eval_dataset', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('instruction-parser-eval', 'Evaluate deterministic command parser on instruction eval set', 'accept_instruction_parser_eval', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('instruction-following-eval', 'Evaluate offline LLM instruction following with llama.cpp', 'accept_instruction_following_eval', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('instruction-following-lora-candidates', 'Export failed instruction-following cases for LoRA review', 'accept_instruction_following_lora_candidates', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('instruction-following-lora-review', 'Init/apply human review for approved LoRA dataset', 'accept_instruction_following_lora_review', 'evaluation', False, True, HandlerDomain.EVALUATION),
    ('lora-q8-pipeline', 'Dry-run LoRA merge -> GGUF -> Q8 pipeline and artifact audit', 'accept_lora_q8_pipeline', 'internal', False, True, HandlerDomain.EVALUATION),
    ('lora-q8-comparison', 'Run isolated baseline/tuned Q8 holdout evaluation and audit', 'accept_lora_q8_comparison', 'internal', False, True, HandlerDomain.EVALUATION),
    ('asr-nlu-samples-to-eval', 'Convert live ASR/NLU sample JSONL into reviewable eval candidates', 'accept_asr_nlu_samples_to_eval', 'internal', False, True, HandlerDomain.VOICE),
    ('asr-nlu-candidate-eval', 'Evaluate parser accuracy on reviewable ASR/NLU eval candidates', 'accept_asr_nlu_candidate_eval', 'internal', False, True, HandlerDomain.VOICE),
    ('release-gate', 'Job-showcase core 5-command gate with logs/acceptance_report.json', 'accept_release_gate', 'internal', False, True, HandlerDomain.CONTROL),
    ('robotics-gate', 'Unified ROS/Nav2/SLAM/dynamic-obstacle evidence gate', 'accept_robotics_gate', 'public', True, True, HandlerDomain.CONTROL),
    ('demo-gate', 'Pre-demo automatic evidence gate with logs/demo_acceptance_report.json', 'accept_demo_gate', 'internal', False, True, HandlerDomain.CONTROL),
    ('demo-evidence-checklist', 'Summarize automatic/live/visual demo evidence into JSON/Markdown', 'accept_demo_evidence_checklist', 'internal', False, True, HandlerDomain.CONTROL),
    ('wsl-microphone-preflight', 'PulseAudio/WSLg microphone capture check before live demos', 'accept_wsl_microphone_preflight', 'internal', False, True, HandlerDomain.CONTROL),
    ('gazebo', 'Typed Action physical motion verification', 'accept_gazebo', 'public', True, True, HandlerDomain.CONTROL),
    ('cpp-action-client', 'Verify C++ typed Action success/feedback/cancel/timeout lifecycle', 'accept_cpp_action_client', 'internal', False, True, HandlerDomain.CONTROL),
    ('cpp-action-scheduler', 'Verify C++ FIFO, priority cancel, result correlation, diagnostics', 'accept_cpp_action_scheduler', 'internal', False, True, HandlerDomain.CONTROL),
    ('cpp-action-bridge-lifecycle', 'Verify inactive reject, cleanup, and reactivate', 'accept_cpp_action_bridge_lifecycle', 'internal', False, True, HandlerDomain.CONTROL),
    ('gazebo-voice', 'Offline synthesized speech through typed Action to Gazebo', 'accept_gazebo_voice', 'internal', False, True, HandlerDomain.CONTROL),
    ('gazebo-voice-online', 'Online voice provider through typed Action to Gazebo', 'accept_gazebo_voice_online', 'internal', False, True, HandlerDomain.CONTROL),
    ('continuous-offline', 'Long-running microphone control using the offline Agent', 'accept_continuous_offline', 'public', True, True, HandlerDomain.VOICE),
    ('continuous-online', 'Long-running microphone control using the online Agent', 'accept_continuous_online', 'public', True, True, HandlerDomain.VOICE),
    ('continuous-nav2-offline', 'Long-running microphone target navigation with Nav2/TurtleBot3', 'accept_continuous_nav2_offline', 'interactive', False, True, HandlerDomain.SLAM_NAV),
    ('continuous-nav2-online', 'Long-running microphone target navigation with Nav2/TurtleBot3', 'accept_continuous_nav2_online', 'interactive', False, True, HandlerDomain.SLAM_NAV),
    ('voice-slam-workplace-demo', '{offline|online}  Live office survey -> SLAM -> Nav2', 'accept_voice_slam_workplace_demo', 'interactive', False, True, HandlerDomain.SLAM_NAV),
    ('continuous-nav2-evidence', '{offline|online}  Run Nav2 microphone demo and live-check evidence in one terminal', 'accept_continuous_nav2_evidence', 'interactive', False, True, HandlerDomain.SLAM_NAV),
    ('continuous-voice-evidence', '{offline|online}  Run microphone demo and 5-minute benchmark in one terminal', 'accept_continuous_voice_evidence', 'interactive', False, True, HandlerDomain.VOICE),
    ('continuous-live-check', '{offline|online}  Observe a running live microphone demo and score evidence', 'accept_continuous_live_check', 'interactive', False, True, HandlerDomain.VOICE),
    ('continuous-voice-benchmark', '{offline|online}  Score a 5-minute, 10-command microphone benchmark', 'accept_continuous_voice_benchmark', 'interactive', False, True, HandlerDomain.EVALUATION),
    ('continuous-nav2-live-check', '{offline|online}  Score a running live Nav2 microphone demo', 'accept_continuous_nav2_live_check', 'interactive', False, True, HandlerDomain.SLAM_NAV),
    ('continuous-live-report', 'REPORT_FILE  Re-score a saved continuous live-check report', 'accept_continuous_live_report', 'interactive', False, True, HandlerDomain.VOICE),
    ('runtime-evidence-summary', 'Summarize online/offline 5-minute, LLM, and latency evidence', 'accept_runtime_evidence_summary', 'internal', False, True, HandlerDomain.CONTROL),
    ('voice-benchmark-report', 'REPORT_FILE  Evaluate recognition/action/false-trigger/latency metrics', 'accept_voice_benchmark_report', 'interactive', False, True, HandlerDomain.EVALUATION),
    ('continuous-nav2-live-report', 'REPORT_FILE  Re-score a saved Nav2 live-check report', 'accept_continuous_nav2_live_report', 'interactive', False, True, HandlerDomain.SLAM_NAV),
)

MODES = tuple(AcceptanceMode(*spec) for spec in _MODE_SPECS)
MODE_BY_NAME = {mode.name: mode for mode in MODES}

if len(MODE_BY_NAME) != len(MODES):
    raise RuntimeError("duplicate acceptance mode")
if {mode.name for mode in MODES if mode.public} != set(PUBLIC_MODE_NAMES):
    raise RuntimeError("public acceptance mode set drifted")
