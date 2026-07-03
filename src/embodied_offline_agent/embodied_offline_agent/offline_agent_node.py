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
from embodied_online_agent.command_fallback import parse_fallback_actions, should_block_model_actions
from embodied_online_agent.command_normalizer import CommandNormalizer
from embodied_online_agent.continuous_voice import ContinuousCommandQueue, ContinuousVoiceSession
from embodied_online_agent.continuous_voice import CommandExecutionTracker, QueueSnapshot
from embodied_online_agent.protocol import SentenceChunker, TaggedStreamParser
from embodied_online_agent.recognition_retry import RecognitionRetryTracker
from embodied_online_agent.types import ActionCommand
from embodied_online_agent.wake_event_input import parse_external_wake_event
from embodied_online_agent.wakeword import WakeWordGate

from .double_buffer import DoubleBuffer
from .latency import OfflineLatency
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
            self._wake_gate, enabled=self._continuous_enabled
        )
        self._retry_tracker = RecognitionRetryTracker(
            self._param("recognition_max_retries")
        )
        self._command_normalizer = CommandNormalizer(
            fuzzy_threshold=float(self._param("command_normalization_fuzzy_threshold")),
            rules_path=self._command_normalization_path(),
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
            "llm_base_url": "http://127.0.0.1:8080/v1",
            "llm_model": "Qwen3-0.6B-Q8_0.gguf",
            "llm_temperature": 0.7,
            "llm_max_tokens": 192,
            "llm_seed": 42,
            "tts_model_dir": "/home/ubuntu/embodied_agent_ws/models/vits-melo-tts-zh_en",
            "tts_num_threads": 2,
            "tts_speaker_id": 0,
            "tts_speed": 1.0,
            "tts_sample_rate": 44100,
            "tts_chunk_max_chars": 24,
            "tts_pcm_chunk_ms": 80,
            "mock_token_delay_s": 0.0,
            "action_sequence_wait_timeout_s": 12.0,
            "continuous_control_enabled": False,
            "voice_session_timeout_s": 60.0,
            "continuous_command_queue_size": 8,
            "continuous_command_max_age_s": 30.0,
            "speech_endpoint_events_enabled": True,
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
            return MockOfflineAsr(), MockOfflineLlm(self._param("mock_token_delay_s")), MockOfflineTts(self._param("tts_sample_rate"))
        if self._mode != "offline":
            raise ValueError("mode must be 'mock' or 'offline'")
        # Native model wheels are intentionally optional in mock mode.
        from .providers.llama_cpp import LlamaCppLlm
        from .providers.sherpa_asr import SherpaZipformerAsr
        from .providers.sherpa_tts import SherpaVitsTts

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
            ),
            SherpaVitsTts(self._param("tts_model_dir"), self._param("tts_num_threads"), self._param("tts_speaker_id"), self._param("tts_speed")),
        )

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
        session_event = self._voice_session.external_sleep(event.provider)
        self._publish_session_event(session_event)
        self._publish_state("sleeping")
        self.get_logger().info(
            f"external sleep event accepted from provider={event.provider}"
        )

    def _on_clear(self, _message):
        self._memory.clear()

    def _on_action_result(self, message):
        self._action_sequencer.notify_result(message.data)

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
                self._publish_state("sleeping")
                self.get_logger().info("continuous voice session sleeping")
            elif decision.reason == "session_awake":
                self._publish_state("session_awake")
                self.get_logger().info("continuous voice session awake")
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
                self._publish_state("queue_full")
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

    def _run_command_worker(self):
        while not self._stopping:
            try:
                item = self._command_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            try:
                with self._state_lock:
                    self._busy = True
                self._publish_execution_event(
                    self._command_tracker.execution_started(item)
                )
                latency = item.context if isinstance(item.context, OfflineLatency) else OfflineLatency()
                self._run_turn(item.text, latency)
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

    def _run_turn(self, user_text, latency):
        message_buffer = DoubleBuffer[str](drop_oldest=False)
        audio_buffer = DoubleBuffer[bytes](drop_oldest=False)
        errors = []

        def tts_worker():
            try:
                while True:
                    text = message_buffer.get()
                    pcm = self._tts.synthesize(text)
                    bytes_per_chunk = max(2, int(self._param("tts_sample_rate") * self._param("tts_pcm_chunk_ms") / 1000) * 2)
                    for offset in range(0, len(pcm), bytes_per_chunk):
                        if not audio_buffer.put(pcm[offset : offset + bytes_per_chunk], timeout=5.0):
                            raise TimeoutError("audio double buffer remained full")
            except StopIteration:
                pass
            except Exception as exc:
                errors.append(exc)
            finally:
                audio_buffer.close()

        def audio_worker():
            try:
                while True:
                    pcm = audio_buffer.get()
                    latency.mark_first_audio()
                    self._audio_pub.publish(UInt8MultiArray(data=list(pcm)))
            except StopIteration:
                return

        tts_thread = threading.Thread(target=tts_worker, daemon=True)
        audio_thread = threading.Thread(target=audio_worker, daemon=True)
        tts_thread.start()
        audio_thread.start()
        parser = TaggedStreamParser()
        chunker = SentenceChunker(self._param("tts_chunk_max_chars"))
        speech_parts = []
        model_actions = []
        self._publish_state("thinking")
        try:
            messages = [{"role": "system", "content": self._system_prompt + "\n/no_think"}]
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
                        if not message_buffer.put(sentence, timeout=5.0):
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
                    if not message_buffer.put(sentence, timeout=5.0):
                        raise TimeoutError("message double buffer remained full")
            for sentence in chunker.finish():
                if not message_buffer.put(sentence, timeout=5.0):
                    raise TimeoutError("message double buffer remained full")
            for error in final.errors:
                self.get_logger().warning(error)
            message_buffer.close()
            tts_thread.join(timeout=60.0)
            audio_thread.join(timeout=60.0)
            if tts_thread.is_alive() or audio_thread.is_alive():
                raise TimeoutError("offline TTS pipeline did not drain")
            if errors:
                raise errors[0]
            assistant_text = "".join(speech_parts).strip()
            self._response_pub.publish(String(data=assistant_text))
            self._memory.append_turn(user_text, assistant_text)
            latency.finish()
            report = latency.report(message_buffer.stats.dropped, audio_buffer.stats.dropped)
            self._metrics_pub.publish(String(data=json.dumps(report, ensure_ascii=False)))
            self.get_logger().info(f"offline latency: {report}")
        except Exception as exc:
            message_buffer.abort()
            audio_buffer.abort()
            self.get_logger().error(f"offline turn failed: {exc}")
            self._publish_state("error")
        finally:
            with self._state_lock:
                self._busy = False
            if not self._stopping:
                self._publish_state("listening")

    def _publish_actions(self, actions):
        action_list = list(actions)

        def publish_payload(payload):
            self._action_pub.publish(String(data=payload))

        report = self._action_sequencer.publish(
            action_list,
            publish_payload,
            wait_for_results=len(action_list) > 1,
        )
        if report.failed:
            self.get_logger().warning(
                f"action sequence stopped after {report.completed} completed step(s): {report.reason}"
            )
        return report.published

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
        self._command_queue_pub.publish(
            String(data=json.dumps(payload.as_dict(), ensure_ascii=False))
        )

    def _publish_execution_event(self, event):
        self._command_execution_pub.publish(
            String(data=json.dumps(event.as_dict(), ensure_ascii=False))
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
