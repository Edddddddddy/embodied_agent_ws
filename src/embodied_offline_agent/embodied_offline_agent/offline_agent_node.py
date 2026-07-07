import json
import os
import queue
import threading
import time
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Empty, String, UInt8MultiArray

from embodied_online_agent.memory import ConversationMemory
from embodied_online_agent.action_sequence import SequentialActionPublisher
from embodied_online_agent.command_completion import CommandCompleter
from embodied_online_agent.command_fallback import parse_fallback_actions, should_block_model_actions
from embodied_online_agent.command_nlu import CommandNLU
from embodied_online_agent.command_normalizer import CommandNormalizer
from embodied_online_agent.continuous_voice import ContinuousCommandQueue, ContinuousVoiceSession
from embodied_online_agent.continuous_voice import CommandExecutionTracker, QueueSnapshot
from embodied_online_agent.protocol import SentenceChunker, TaggedStreamParser
from embodied_online_agent.recognition_retry import RecognitionRetryTracker
from embodied_online_agent.types import ActionCommand
from embodied_online_agent.user_memory import (
    SpeakerIdentity,
    UserMemoryStore,
    parse_memory_command,
)
from embodied_online_agent.wake_event_input import parse_external_wake_event
from embodied_online_agent.wakeword import WakeWordGate

from .latency import OfflineLatency
from .pseudo_streaming_tts import PseudoStreamingTtsPipeline
from .providers.mock import MockOfflineAsr, MockOfflineLlm, MockOfflineTts


class OfflineAgentNode(Node):
    def __init__(self):
        super().__init__("offline_agent")
        self._declare_parameters()
        self._mode = self._param("mode")
        self._stopping = False
        self._busy = False
        self._state_lock = threading.Lock()
        self._last_asr_commit_monotonic = 0.0
        self._continuous_enabled = bool(self._param("continuous_control_enabled"))
        self._nlu_batch_sequence = 0
        self._command_queue = ContinuousCommandQueue(
            int(self._param("continuous_command_queue_size")),
            max_age_s=float(self._param("continuous_command_max_age_s")),
        )
        self._command_tracker = CommandExecutionTracker("offline")
        self._command_worker_thread = None
        self._latency = OfflineLatency()
        self._asr_events: queue.Queue[tuple[str, bytes | None]] = queue.Queue(maxsize=64)

        self._memory = ConversationMemory(
            self._param("memory_path"), self._param("memory_max_turns")
        )
        self._current_speaker = SpeakerIdentity()
        self._user_memory = UserMemoryStore(
            self._param("user_memory_dir"),
            max_recent=int(self._param("user_memory_max_recent")),
        )
        self._wake_gate = WakeWordGate(
            self._param("wake_words"),
            aliases=self._param("wake_word_aliases"),
            enabled=self._param("wake_word_enabled"),
            active_timeout_s=(
                self._param("voice_session_timeout_s")
                if self._continuous_enabled
                else self._param("wake_active_timeout_s")
            ),
        )
        self._voice_session = ContinuousVoiceSession(
            self._wake_gate,
            enabled=self._continuous_enabled,
            duplicate_window_s=float(self._param("continuous_duplicate_window_s")),
        )
        self._retry_tracker = RecognitionRetryTracker(
            self._param("recognition_max_retries")
        )
        self._command_normalizer = CommandNormalizer(
            fuzzy_threshold=float(self._param("command_normalization_fuzzy_threshold")),
            rules_path=self._command_normalization_path(),
        )
        self._command_completer = CommandCompleter(
            enabled=bool(self._param("command_completion_enabled"))
        )
        self._command_nlu = CommandNLU(
            enabled=bool(self._param("command_nlu_enabled")),
            min_confidence=float(self._param("command_nlu_min_confidence")),
        )
        self._action_sequencer = SequentialActionPublisher(
            self._param("action_sequence_wait_timeout_s")
        )
        prompt_path = Path(
            get_package_share_directory("embodied_online_agent")
        ) / "prompts" / "system_prompt_zh.txt"
        configured_prompt = self._param("system_prompt_path")
        self._system_prompt = Path(os.path.expanduser(configured_prompt)).read_text(encoding="utf-8") if configured_prompt else prompt_path.read_text(encoding="utf-8")

        self._asr_partial_pub = self.create_publisher(String, "/agent/asr_partial", 10)
        self._asr_final_pub = self.create_publisher(String, "/agent/asr_final", 10)
        self._response_delta_pub = self.create_publisher(String, "/agent/response_delta", 10)
        self._response_pub = self.create_publisher(String, "/agent/response_text", 10)
        self._action_pub = self.create_publisher(String, "/agent/action_candidate", 10)
        self._state_pub = self.create_publisher(String, "/agent/state", 10)
        self._wake_event_pub = self.create_publisher(String, "/agent/wake_event", 10)
        self._session_state_pub = self.create_publisher(String, "/agent/session_state", 10)
        self._command_queue_pub = self.create_publisher(String, "/agent/command_queue", 10)
        self._command_execution_pub = self.create_publisher(
            String, "/agent/command_execution", 10
        )
        self._recognition_feedback_pub = self.create_publisher(
            String, "/agent/recognition_feedback", 10
        )
        self._speaker_enroll_request_pub = self.create_publisher(
            String, "/agent/speaker_enroll_request", 10
        )
        self._metrics_pub = self.create_publisher(String, "/offline_agent/metrics", 10)
        audio_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        self._audio_pub = self.create_publisher(UInt8MultiArray, "/audio/tts_pcm", audio_qos)
        self.create_subscription(String, "/agent/text_input", self._on_text, 10)
        if self._param("external_wake_event_enabled"):
            self.create_subscription(
                String, "/agent/wake_event_input", self._on_wake_event_input, 10
            )
        self.create_subscription(
            String, "/agent/speaker_identity", self._on_speaker_identity, 10
        )
        self.create_subscription(Empty, "/agent/clear_memory", self._on_clear, 10)
        self.create_subscription(String, "/robot/action_result", self._on_action_result, 10)
        if self._continuous_enabled:
            self._command_worker_thread = threading.Thread(
                target=self._run_command_worker, daemon=True
            )
            self._command_worker_thread.start()

        self._asr, self._llm, self._tts = self._create_providers()
        if self._param("microphone_enabled"):
            self._asr.start(self._on_asr_partial, self._on_asr_final)
            self.create_subscription(UInt8MultiArray, "/audio/clean_pcm", self._on_audio, audio_qos)
            self.create_subscription(Empty, "/audio/silence_timeout", self._on_silence, 10)
            self.create_subscription(Empty, "/audio/speech_started", self._on_speech_started, 10)
            self.create_subscription(Empty, "/audio/speech_ended", self._on_speech_ended, 10)
            self._asr_thread = threading.Thread(target=self._run_asr, daemon=True)
            self._asr_thread.start()
        else:
            self._asr_thread = None
        self._publish_state("listening")
        self.get_logger().info(
            f"offline agent ready: mode={self._mode}, microphone={self._param('microphone_enabled')}"
        )

    def _declare_parameters(self):
        defaults = {
            "mode": "mock",
            "microphone_enabled": False,
            "audio_sample_rate": 16000,
            "wake_word_enabled": True,
            "wake_words": ["小智", "你好小智"],
            "wake_word_aliases": ["小志", "小治", "晓智", "晓志"],
            "wake_active_timeout_s": 10.0,
            "memory_path": "~/.ros/embodied_agent/offline_memory.json",
            "memory_max_turns": 6,
            "user_memory_dir": "~/.ros/embodied_agent/users",
            "user_memory_max_recent": 8,
            "speaker_identity_min_confidence": 0.55,
            "system_prompt_path": "",
            "asr_model_dir": "/home/ubuntu/embodied_agent_ws/models/sherpa-onnx-streaming-zipformer-small-bilingual-zh-en-2023-02-16",
            "asr_num_threads": 2,
            "asr_decoding_method": "modified_beam_search",
            "asr_hotwords_file": "",
            "asr_hotwords_score": 2.0,
            "asr_max_active_paths": 4,
            "asr_modeling_unit": "cjkchar",
            "recognition_max_retries": 3,
            "command_normalization_enabled": True,
            "command_normalization_feedback_enabled": True,
            "command_normalization_fuzzy_threshold": 0.82,
            "command_normalization_path": "",
            "command_completion_enabled": True,
            "command_nlu_enabled": True,
            "command_nlu_min_confidence": 0.18,
            "llm_base_url": "http://127.0.0.1:8080/v1",
            "llm_model": "Qwen3-0.6B-Q8_0.gguf",
            "llm_temperature": 0.7,
            "llm_max_tokens": 192,
            "llm_seed": 42,
            "llm_timeout_s": 30.0,
            "llm_max_retries": 1,
            "llm_first_token_warn_ms": 1000.0,
            "tts_model_dir": "/home/ubuntu/embodied_agent_ws/models/vits-melo-tts-zh_en",
            "tts_provider": "sherpa",
            "tts_num_threads": 2,
            "tts_speaker_id": 0,
            "tts_speed": 1.0,
            "tts_sample_rate": 44100,
            "tts_chunk_max_chars": 24,
            "tts_pcm_chunk_ms": 80,
            "summer_tts_binary": "/home/ubuntu/embodied_agent_ws/third_party/SummerTTS/build/tts_test",
            "summer_tts_model": "/home/ubuntu/embodied_agent_ws/third_party/SummerTTS/models/single_speaker_fast.bin",
            "summer_tts_timeout_s": 30.0,
            "mock_token_delay_s": 0.0,
            "mock_asr_finals": "",
            "action_sequence_wait_timeout_s": 12.0,
            "continuous_control_enabled": False,
            "voice_session_timeout_s": 60.0,
            "continuous_command_queue_size": 8,
            "continuous_command_max_age_s": 30.0,
            "continuous_duplicate_window_s": 1.2,
            "speech_endpoint_events_enabled": True,
            "asr_commit_delay_ms": 0,
            "external_wake_event_enabled": True,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _param(self, name):
        return self.get_parameter(name).value

    def _command_normalization_path(self) -> Path | str:
        configured = self._param("command_normalization_path")
        if configured:
            return Path(os.path.expanduser(configured))
        return (
            Path(get_package_share_directory("embodied_online_agent"))
            / "config"
            / "command_normalization_zh.yaml"
        )

    def _create_providers(self):
        if self._mode == "mock":
            return MockOfflineAsr(self._mock_asr_finals()), MockOfflineLlm(self._param("mock_token_delay_s")), MockOfflineTts(self._param("tts_sample_rate"))
        if self._mode != "offline":
            raise ValueError("mode must be 'mock' or 'offline'")
        # Native model wheels are intentionally optional in mock mode.
        from .providers.llama_cpp import LlamaCppLlm
        from .providers.sherpa_asr import SherpaZipformerAsr

        return (
            SherpaZipformerAsr(
                self._param("asr_model_dir"),
                self._param("audio_sample_rate"),
                self._param("asr_num_threads"),
                decoding_method=self._param("asr_decoding_method"),
                hotwords_file=self._hotwords_file(),
                hotwords_score=self._param("asr_hotwords_score"),
                max_active_paths=self._param("asr_max_active_paths"),
                modeling_unit=self._param("asr_modeling_unit"),
            ),
            LlamaCppLlm(
                self._param("llm_base_url"), self._param("llm_model"),
                self._param("llm_temperature"), self._param("llm_max_tokens"),
                self._param("llm_seed"),
                timeout_s=self._param("llm_timeout_s"),
                max_retries=self._param("llm_max_retries"),
                first_token_warn_ms=self._param("llm_first_token_warn_ms"),
            ),
            self._create_tts_provider(),
        )

    def _create_tts_provider(self):
        provider = str(self._param("tts_provider") or "sherpa").lower()
        if provider == "sherpa":
            from .providers.sherpa_tts import SherpaVitsTts

            return SherpaVitsTts(
                self._param("tts_model_dir"),
                self._param("tts_num_threads"),
                self._param("tts_speaker_id"),
                self._param("tts_speed"),
            )
        if provider == "summer":
            from .providers.summer_tts import SummerTts

            return SummerTts(
                self._param("summer_tts_binary"),
                self._param("summer_tts_model"),
                timeout_s=float(self._param("summer_tts_timeout_s")),
            )
        raise ValueError("tts_provider must be 'sherpa' or 'summer'")

    def _tts_sample_rate(self):
        return int(getattr(self._tts, "sample_rate", self._param("tts_sample_rate")))

    def _mock_asr_finals(self):
        scripted = str(self._param("mock_asr_finals") or "")
        return [item.strip() for item in scripted.split("|") if item.strip()]

    def _hotwords_file(self):
        configured = self._param("asr_hotwords_file")
        if configured:
            return str(Path(os.path.expanduser(configured)))
        return str(
            Path(get_package_share_directory("embodied_offline_agent"))
            / "config"
            / "hotwords_zh.txt"
        )

    def _on_audio(self, message):
        if self._is_busy() and not self._continuous_enabled:
            return
        self._enqueue_asr(("audio", bytes(message.data)))

    def _on_silence(self, _message):
        self._commit_asr_endpoint("silence_timeout")

    def _on_speech_started(self, _message):
        if self._is_busy() and not self._continuous_enabled:
            return
        self._publish_state("speech_detected")

    def _on_speech_ended(self, _message):
        if not self._param("speech_endpoint_events_enabled"):
            return
        self._commit_asr_endpoint("speech_ended")

    def _commit_asr_endpoint(self, source):
        if self._is_busy() and not self._continuous_enabled:
            return
        now = time.monotonic()
        if now - self._last_asr_commit_monotonic < 0.05:
            self.get_logger().debug(f"ignored duplicate ASR commit from {source}")
            return
        self._last_asr_commit_monotonic = now
        self._latency = OfflineLatency()
        self._latency.mark_silence()
        delay_ms = max(0, int(self._param("asr_commit_delay_ms")))
        self._publish_asr_endpoint_feedback(source, delay_ms)
        if delay_ms > 0:
            # 离线 ZipFormer 也可能在端点刚触发时漏掉尾部数字/量词；
            # commit delay 让最后一小段音频先进入 ASR 队列，再统一提交 final。
            timer = threading.Timer(
                delay_ms / 1000.0, self._enqueue_delayed_asr_commit, args=(source,)
            )
            timer.daemon = True
            timer.start()
            return
        self._enqueue_delayed_asr_commit(source)

    def _enqueue_delayed_asr_commit(self, source):
        if self._stopping:
            return
        if self._is_busy() and not self._continuous_enabled:
            return
        self._publish_asr_commit_feedback(source)
        self._enqueue_asr(("commit", None), preserve=True)

    def _enqueue_asr(self, event, preserve=False):
        try:
            self._asr_events.put_nowait(event)
        except queue.Full:
            try:
                self._asr_events.get_nowait()
                self._asr_events.task_done()
                self._asr_events.put_nowait(event)
                self.get_logger().warning("ASR input backlog: dropped oldest audio frame")
            except queue.Empty:
                if preserve:
                    self.get_logger().error("ASR commit event could not be queued")

    def _run_asr(self):
        while not self._stopping:
            kind, payload = self._asr_events.get()
            try:
                if kind == "stop":
                    return
                if (
                    kind in {"audio", "commit"}
                    and self._is_busy()
                    and not self._continuous_enabled
                ):
                    continue
                if kind == "audio" and payload is not None:
                    self._asr.push_audio(payload)
                elif kind == "commit":
                    self._asr.commit()
            except Exception as exc:
                self.get_logger().error(f"offline ASR failed: {exc}")
            finally:
                self._asr_events.task_done()

    def _is_busy(self):
        with self._state_lock:
            return self._busy

    def _on_asr_partial(self, text):
        if self._is_busy() and not self._continuous_enabled:
            return
        self._asr_partial_pub.publish(String(data=text))

    def _on_asr_final(self, text):
        if self._is_busy() and not self._continuous_enabled:
            self.get_logger().warning(
                "offline agent busy; suppressing overlapping ASR final"
            )
            return
        self._latency.mark_asr_final()
        self._asr_final_pub.publish(String(data=text))
        self._accept_transcript(text)

    def _on_text(self, message):
        self._latency = OfflineLatency()
        self._latency.mark_silence()
        self._latency.mark_asr_final()
        self._asr_final_pub.publish(String(data=message.data))
        self._accept_transcript(message.data)

    def _on_wake_event_input(self, message):
        event = parse_external_wake_event(message.data)
        if event is None:
            self.get_logger().warning(
                f"ignored invalid external wake event payload: {message.data!r}"
            )
            return
        if event.kind == "wake":
            session_event = self._voice_session.external_wake(
                event.provider, event.transcript
            )
            self._publish_session_event(session_event)
            self._publish_state("session_awake")
            self.get_logger().info(
                f"external wake event accepted from provider={event.provider}"
            )
            return

        dropped = self._command_queue.clear()
        self._publish_queue_event(
            "clear", "", QueueSnapshot(True, self._command_queue.size(), dropped)
        )
        self._action_sequencer.cancel("external_sleep")
        self._publish_actions([ActionCommand("stop", {})])
        session_event = self._voice_session.external_sleep(event.provider)
        self._publish_session_event(session_event)
        self._publish_state("sleeping")
        self.get_logger().info(
            f"external sleep event accepted from provider={event.provider}"
        )

    def _on_clear(self, _message):
        self._memory.clear()
        self._user_memory.clear(self._current_speaker)

    def _on_action_result(self, message):
        self._action_sequencer.notify_result(message.data)

    def _on_speaker_identity(self, message):
        identity = SpeakerIdentity.from_json(
            message.data,
            min_confidence=float(self._param("speaker_identity_min_confidence")),
        )
        self._current_speaker = identity
        if identity.usable:
            self.get_logger().info(
                f"speaker identity accepted: speaker_id={identity.speaker_id}, confidence={identity.confidence:.3f}"
            )

    def _accept_transcript(self, transcript):
        transcript = self._normalize_transcript(transcript)
        decision = self._voice_session.accept(transcript)
        self._publish_session_event(decision.event)
        if not decision.accepted or decision.command is None:
            if decision.reason == "session_sleep":
                dropped = self._command_queue.clear()
                self._publish_queue_event(
                    "clear", "", QueueSnapshot(True, self._command_queue.size(), dropped)
                )
                self._action_sequencer.cancel("session_sleep")
                self._publish_actions([ActionCommand("stop", {})])
                self._publish_state("sleeping")
                self.get_logger().info("continuous voice session sleeping")
            elif decision.reason == "session_awake":
                self._publish_state("session_awake")
                self.get_logger().info("continuous voice session awake")
            elif decision.reason in {"filler", "duplicate_command"}:
                self._publish_ignored_recognition(transcript, decision.reason)
                self._publish_state("listening")
            elif decision.reason == "session_timeout":
                feedback = self._retry_tracker.failed(transcript)
                self._recognition_feedback_pub.publish(
                    String(data=feedback.to_json())
                )
                self._publish_session_timeout_feedback(transcript)
                self.get_logger().warning(
                    "voice session timed out; waiting for a new wake word"
                )
                self._publish_state("retry_listening")
            elif self._param("wake_word_enabled") and not self._wake_gate.active:
                feedback = self._retry_tracker.failed(transcript)
                self._recognition_feedback_pub.publish(
                    String(data=feedback.to_json())
                )
                self.get_logger().warning(
                    f"wake word not detected ({feedback.attempt}/{feedback.max_attempts}); retrying"
                )
                self._publish_state("retry_listening")
            return
        self._retry_tracker.succeeded()
        command = decision.command
        if not decision.priority_stop:
            # 补全只作用于普通动作命令；停下/急停不能被延迟或改写。
            command = self._complete_command(command)
        if self._handle_memory_command(command):
            self._publish_state("listening")
            return
        if self._continuous_enabled:
            if decision.priority_stop:
                dropped = self._command_queue.clear()
                self._publish_queue_event(
                    "clear",
                    command,
                    QueueSnapshot(True, self._command_queue.size(), dropped),
                    priority_stop=True,
                )
                self._action_sequencer.cancel("priority_stop")
                self.get_logger().info("priority stop received; cancelling current sequence")
                if dropped:
                    self.get_logger().info(f"cleared {dropped} queued command(s)")
                self._publish_actions([ActionCommand("stop", {})])
                self._publish_state("listening")
                return
            self._enqueue_continuous_command(command)
            return
        with self._state_lock:
            if self._busy:
                self.get_logger().warning("offline agent busy; overlapping utterance dropped")
                return
            self._busy = True
        turn_latency = self._latency
        threading.Thread(
            target=self._run_turn, args=(command, turn_latency), daemon=True
        ).start()

    def _handle_memory_command(self, command):
        memory_command = parse_memory_command(command)
        if memory_command is None:
            return False
        identity = self._current_speaker
        if memory_command.kind == "whoami":
            if identity.usable:
                profile = self._user_memory.profile(identity)
                name = profile.display_name or identity.display_name or identity.speaker_id
                response = f"我识别到当前用户是：{name}。"
            else:
                response = "我还没有可靠识别到当前用户，可以先说“记住我，我是某某”。"
        elif memory_command.kind == "clear":
            self._user_memory.clear(identity)
            response = "已清除当前用户的本地行为记忆。"
        elif memory_command.kind == "enroll_request":
            if identity.usable:
                self._publish_speaker_enroll_request(
                    identity.speaker_id, identity.display_name or identity.speaker_id
                )
                response = "已开始声纹录入，请连续说三句短句用于采集样本。"
            else:
                response = "请先说“记住我，我是某某”，我会用这个名字开始声纹录入。"
        elif memory_command.kind == "enroll_name":
            if not identity.usable:
                # 没有真实声纹 identity 时提供演示兜底；真实部署仍以 /agent/speaker_identity 为准。
                identity = SpeakerIdentity(
                    speaker_id=str(memory_command.value),
                    confidence=1.0,
                    enrolled=True,
                    model="text-enroll-fallback",
                    display_name=str(memory_command.value),
                    updated_at=time.time(),
                )
                self._current_speaker = identity
            profile = self._user_memory.enroll(identity, str(memory_command.value))
            self._publish_speaker_enroll_request(identity.speaker_id, profile.display_name)
            response = (
                f"好的，我记住你是 {profile.display_name or profile.speaker_id}。"
                "如果已启动声纹 sidecar，请继续说三句短句完成样本采集。"
            )
        elif memory_command.kind == "preference":
            if not identity.usable:
                response = "我还没有可靠识别当前用户，先说“记住我，我是某某”后再记录偏好。"
            else:
                profile = self._user_memory.profile(identity)
                for key, value in dict(memory_command.value).items():
                    profile = self._user_memory.set_preference(identity, key, value)
                response = "已记录你的偏好：" + "，".join(
                    f"{key}={value}" for key, value in sorted(profile.preferences.items())
                )
        else:
            return False
        self._response_delta_pub.publish(String(data=response))
        self._response_pub.publish(String(data=response))
        threading.Thread(
            target=self._speak_memory_response, args=(response,), daemon=True
        ).start()
        self._user_memory.record_interaction(
            identity,
            user_text=command,
            assistant_text=response,
            actions=[],
            success=True,
        )
        return True

    def _speak_memory_response(self, response):
        try:
            self._audio_pub.publish(
                UInt8MultiArray(data=list(self._tts.synthesize(response)))
            )
        except Exception as exc:
            self.get_logger().warning(f"memory response TTS failed: {exc}")

    def _publish_speaker_enroll_request(self, speaker_id, display_name):
        payload = {
            "speaker_id": speaker_id,
            "display_name": display_name or speaker_id,
            "samples_required": 3,
        }
        self._speaker_enroll_request_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _normalize_transcript(self, transcript):
        if not self._param("command_normalization_enabled"):
            return transcript
        result = self._command_normalizer.normalize(transcript)
        if result.changed:
            self.get_logger().info(
                f"normalized ASR command: '{result.original}' -> '{result.text}'"
            )
            if self._param("command_normalization_feedback_enabled"):
                self._recognition_feedback_pub.publish(
                    String(data=result.to_feedback_json())
                )
        return result.text

    def _complete_command(self, command):
        result = self._command_completer.complete(command)
        if result.changed:
            self.get_logger().info(
                f"completed short ASR command: '{result.original}' -> '{result.text}'"
            )
            self._recognition_feedback_pub.publish(
                String(data=result.to_feedback_json())
            )
        return result.text

    def _run_command_worker(self):
        while not self._stopping:
            try:
                item = self._command_queue.get(
                    timeout=0.1,
                    on_stale=self._publish_stale_command_event,
                )
            except queue.Empty:
                continue
            try:
                with self._state_lock:
                    self._busy = True
                # execution 事件给 continuous monitor 使用，用于区分“已入队”和“正在执行”。
                self._publish_execution_event(
                    self._command_tracker.execution_started(item)
                )
                self._run_queued_turn(item)
                self._publish_execution_event(
                    self._command_tracker.execution_finished(
                        item, success=True, reason="completed"
                    )
                )
            except Exception as exc:
                self._publish_execution_event(
                    self._command_tracker.execution_finished(
                        item, success=False, reason=str(exc)
                    )
                )
                raise
            finally:
                self._command_queue.task_done()

    def _publish_stale_command_event(self, item):
        self._publish_command_queue_payload(
            self._command_tracker.queue_expired(item, size=self._command_queue.size())
        )

    def _run_turn(self, user_text, latency):
        tts_pipeline = PseudoStreamingTtsPipeline(
            synthesize=self._tts.synthesize,
            publish_audio=lambda pcm: self._audio_pub.publish(
                UInt8MultiArray(data=list(pcm))
            ),
            sample_rate=self._tts_sample_rate(),
            pcm_chunk_ms=int(self._param("tts_pcm_chunk_ms")),
            on_first_audio=latency.mark_first_audio,
        )
        tts_pipeline.start()
        parser = TaggedStreamParser()
        chunker = SentenceChunker(self._param("tts_chunk_max_chars"))
        speech_parts = []
        model_actions = []
        self._publish_state("thinking")
        try:
            messages = [
                {
                    "role": "system",
                    "content": self._system_prompt_with_user_memory()
                    + "\n/no_think",
                }
            ]
            messages.extend(self._memory.messages())
            messages.append({"role": "user", "content": user_text + " /no_think"})
            latency.mark_llm_start()
            first_token = True
            for token in self._llm.stream(messages):
                if first_token:
                    latency.mark_first_token()
                    first_token = False
                events = parser.feed(token)
                for delta in events.speech:
                    speech_parts.append(delta)
                    self._response_delta_pub.publish(String(data=delta))
                    for sentence in chunker.feed(delta):
                        self._publish_state("speaking")
                        if not tts_pipeline.put_text(sentence):
                            raise TimeoutError("message double buffer remained full")
                model_actions.extend(events.actions)
            final = parser.finish()
            model_actions.extend(final.actions)
            fallback_actions = parse_fallback_actions(user_text)
            if fallback_actions:
                self._publish_actions(fallback_actions)
            elif should_block_model_actions(user_text):
                if model_actions:
                    self.get_logger().warning("model actions blocked by semantic safety policy")
            else:
                self._publish_actions(model_actions)
            if not speech_parts:
                fallback = "抱歉，回复格式解析失败，请再说一次。"
                speech_parts.append(fallback)
                self._response_delta_pub.publish(String(data=fallback))
                for sentence in chunker.feed(fallback):
                    if not tts_pipeline.put_text(sentence):
                        raise TimeoutError("message double buffer remained full")
            for sentence in chunker.finish():
                if not tts_pipeline.put_text(sentence):
                    raise TimeoutError("message double buffer remained full")
            for error in final.errors:
                self.get_logger().warning(error)
            tts_metrics = tts_pipeline.close_and_wait(timeout_s=60.0)
            assistant_text = "".join(speech_parts).strip()
            self._response_pub.publish(String(data=assistant_text))
            self._memory.append_turn(user_text, assistant_text)
            self._user_memory.record_interaction(
                self._current_speaker,
                user_text=user_text,
                assistant_text=assistant_text,
                actions=[action.as_dict() for action in (fallback_actions or model_actions)],
                success=True,
            )
            latency.finish()
            report = latency.report(
                tts_metrics.message_buffer_dropped,
                tts_metrics.audio_buffer_dropped,
            )
            # llama.cpp 的吞吐和首 token 指标与端到端延迟分开记录；
            # 这样验收时能判断是 ASR、LLM 还是 TTS/动作链路导致慢。
            report["llm_provider"] = getattr(self._llm, "last_metrics", {})
            report["tts_pipeline"] = tts_metrics.as_dict()
            self._metrics_pub.publish(String(data=json.dumps(report, ensure_ascii=False)))
            self.get_logger().info(f"offline latency: {report}")
        except Exception as exc:
            tts_pipeline.abort()
            self.get_logger().error(f"offline turn failed: {exc}")
            self._publish_state("error")
        finally:
            with self._state_lock:
                self._busy = False
            if not self._stopping:
                self._publish_state("listening")

    def _run_queued_turn(self, item):
        context = item.context if isinstance(item.context, dict) else {}
        actions = self._actions_from_context(context)
        if actions:
            latency = context.get("latency")
            self._run_preparsed_turn(
                item.text,
                actions,
                latency if isinstance(latency, OfflineLatency) else OfflineLatency(),
            )
            return
        latency = item.context if isinstance(item.context, OfflineLatency) else OfflineLatency()
        self._run_turn(item.text, latency)

    def _run_preparsed_turn(self, user_text, actions, latency):
        self._publish_state("thinking")
        response = "好的，按顺序执行：" + "，".join(action.name for action in actions) + "。"
        self._response_delta_pub.publish(String(data=response))
        self._response_pub.publish(String(data=response))
        report = self._publish_actions(actions)
        self._user_memory.record_interaction(
            self._current_speaker,
            user_text=user_text,
            assistant_text=response,
            actions=[action.as_dict() for action in actions],
            success=not report.failed,
        )
        latency.finish()
        report = latency.report(0, 0)
        self._metrics_pub.publish(String(data=json.dumps(report, ensure_ascii=False)))

    @staticmethod
    def _actions_from_context(context):
        raw_actions = context.get("preparsed_actions") or []
        actions = []
        for raw in raw_actions:
            if isinstance(raw, dict) and isinstance(raw.get("name"), str):
                actions.append(
                    ActionCommand(raw["name"], dict(raw.get("arguments") or {}))
                )
        return actions

    def _enqueue_continuous_command(self, command):
        nlu_result = self._command_nlu.parse(command)
        if nlu_result.accepted:
            self._nlu_batch_sequence += 1
            batch_id = f"offline-nlu-{self._nlu_batch_sequence}"
            self._publish_nlu_feedback(command, nlu_result, batch_id)
            for index, parsed in enumerate(nlu_result.commands, start=1):
                metadata = {
                    "batch_id": batch_id,
                    "batch_index": index,
                    "batch_size": len(nlu_result.commands),
                    "source_text": command,
                    "nlu_intent": parsed.intent,
                    "nlu_confidence": round(parsed.confidence, 3),
                }
                context = {
                    **metadata,
                    "latency": self._latency,
                    "preparsed_actions": [action.as_dict() for action in parsed.actions],
                }
                snapshot = self._command_queue.put(
                    parsed.span_text, context=context, metadata=metadata
                )
                self._publish_queue_event("enqueue", parsed.span_text, snapshot)
                if not snapshot.accepted:
                    self._publish_queue_rejected_recognition(parsed.span_text, snapshot)
                    self._publish_state("queue_full")
                    return
            self._publish_state("queued")
            return

        snapshot = self._command_queue.put(command, context=self._latency)
        self._publish_queue_event("enqueue", command, snapshot)
        if snapshot.accepted:
            self.get_logger().info(
                f"continuous command queued: size={snapshot.size}, text={command}"
            )
            self._publish_state("queued")
        else:
            self.get_logger().warning(
                f"continuous command queue rejected input: {snapshot.reason}"
            )
            self._publish_queue_rejected_recognition(command, snapshot)
            self._publish_state("queue_full")

    def _publish_nlu_feedback(self, command, nlu_result, batch_id):
        payload = {
            "status": "nlu_parsed",
            "reason": "command_nlu",
            "transcript": command,
            "batch_id": batch_id,
            "commands": [
                {
                    "intent": parsed.intent,
                    "span_text": parsed.span_text,
                    "confidence": round(parsed.confidence, 3),
                    "actions": [action.as_dict() for action in parsed.actions],
                }
                for parsed in nlu_result.commands
            ],
        }
        self._recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _publish_actions(self, actions):
        action_list = list(actions)

        def publish_payload(payload):
            self._action_pub.publish(String(data=payload))

        report = self._action_sequencer.publish(
            action_list,
            publish_payload,
            wait_for_results=self._should_wait_for_action_results(action_list),
        )
        if report.failed:
            self.get_logger().warning(
                f"action sequence stopped after {report.completed} completed step(s): {report.reason}"
            )
        return report

    def _should_wait_for_action_results(self, action_list):
        if len(action_list) > 1:
            return True
        # 连续控制的 worker 必须等单动作 result 后再消费下一条命令。
        # 这样长时间语音输入时，普通动作会串行完成，而不是被后续 Action goal 抢占。
        # 优先级 stop 可能在 ROS 回调线程中发布，仍保持 fire-and-forget 以避免阻塞 result 回调。
        return (
            bool(action_list)
            and self._continuous_enabled
            and self._command_worker_thread is not None
            and threading.current_thread() is self._command_worker_thread
        )

    def _system_prompt_with_user_memory(self):
        summary = self._user_memory.prompt_summary(self._current_speaker)
        if not summary:
            return self._system_prompt
        return (
            self._system_prompt
            + "\n\n[用户画像记忆]\n"
            + summary
            + "\n请仅把用户画像作为偏好参考，所有动作仍必须遵守输出格式和安全限幅。"
        )

    def _publish_state(self, state):
        self._state_pub.publish(String(data=state))

    def _publish_session_event(self, event):
        self._wake_event_pub.publish(
            String(data=json.dumps(event.wake_event.as_dict(), ensure_ascii=False))
        )
        self._session_state_pub.publish(String(data=event.session_state))

    def _publish_queue_event(
        self,
        event,
        text,
        snapshot,
        *,
        priority_stop=False,
    ):
        payload = self._command_tracker.queue_event(
            event, text, snapshot, priority_stop=priority_stop
        )
        self._publish_command_queue_payload(payload)

    def _publish_command_queue_payload(self, payload):
        self._command_queue_pub.publish(
            String(data=json.dumps(payload.as_dict(), ensure_ascii=False))
        )

    def _publish_execution_event(self, event):
        self._command_execution_pub.publish(
            String(data=json.dumps(event.as_dict(), ensure_ascii=False))
        )

    def _publish_ignored_recognition(self, transcript, reason):
        payload = {
            "status": "ignored",
            "reason": reason,
            "transcript": transcript,
        }
        self._recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )
        self.get_logger().info(f"ignored ASR final: reason={reason}, text={transcript}")

    def _publish_queue_rejected_recognition(self, transcript, snapshot):
        payload = {
            "status": "queue_rejected",
            "reason": snapshot.reason,
            "transcript": transcript,
            "queue_size": snapshot.size,
        }
        self._recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _publish_asr_endpoint_feedback(self, source, delay_ms):
        payload = {
            "status": "asr_endpoint",
            "source": source,
            "delay_ms": delay_ms,
        }
        self._recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _publish_asr_commit_feedback(self, source):
        payload = {
            "status": "asr_commit",
            "source": source,
        }
        self._recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def _publish_session_timeout_feedback(self, transcript):
        payload = {
            "status": "session_timeout",
            "reason": "voice_session_timeout",
            "transcript": transcript,
            "prompt": "会话已超时，请先说小智",
        }
        self._recognition_feedback_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def shutdown(self):
        self._stopping = True
        if self._asr_thread:
            self._enqueue_asr(("stop", None), preserve=True)
            self._asr_thread.join(timeout=2.0)
        if self._command_worker_thread:
            self._command_worker_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OfflineAgentNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
