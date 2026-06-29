import json
import os
import queue
import threading
from pathlib import Path
from typing import Iterable

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Empty, String

from .actions import ActionValidator
from .audio import MicrophoneFrontend, NlmsEchoCanceller, PcmSpeaker
from .memory import ConversationMemory
from .metrics import LatencyTracker
from .protocol import SentenceChunker, TaggedStreamParser
from .providers.mock import MockAsr, MockLlm, MockTts
from .providers.openai_compatible_llm import OpenAiCompatibleLlm
from .providers.qwen_asr import QwenRealtimeAsr
from .providers.qwen_tts import QwenRealtimeTts
from .wakeword import WakeWordGate


class OnlineAgentNode(Node):
    def __init__(self):
        super().__init__("online_agent")
        self._declare_parameters()
        self.mode = self._param("mode")
        self._state_lock = threading.Lock()
        self._busy = False
        self._stopping = False
        self.metrics = LatencyTracker()
        self.action_validator = ActionValidator()
        self.wake_gate = WakeWordGate(
            self._param("wake_words"),
            enabled=self._param("wake_word_enabled"),
            active_timeout_s=self._param("wake_active_timeout_s"),
        )
        self.memory = ConversationMemory(
            self._param("memory_path"),
            max_turns=self._param("memory_max_turns"),
        )
        self.system_prompt = self._load_system_prompt()

        self.asr_partial_pub = self.create_publisher(String, "/agent/asr_partial", 10)
        self.asr_final_pub = self.create_publisher(String, "/agent/asr_final", 10)
        self.response_pub = self.create_publisher(String, "/agent/response_text", 10)
        self.response_delta_pub = self.create_publisher(String, "/agent/response_delta", 10)
        self.action_pub = self.create_publisher(String, "/robot/action_command", 10)
        self.state_pub = self.create_publisher(String, "/agent/state", 10)
        self.metrics_pub = self.create_publisher(String, "/agent/metrics", 10)
        self.create_subscription(String, "/agent/text_input", self._on_text_input, 10)
        self.create_subscription(Empty, "/agent/clear_memory", self._on_clear_memory, 10)

        self.echo_canceller = NlmsEchoCanceller(
            mic_rate=self._param("audio_sample_rate"),
            reference_rate=self._param("tts_sample_rate"),
            taps=self._param("aec_taps"),
            step=self._param("aec_step"),
        )
        self.speaker = PcmSpeaker(self._param("tts_sample_rate"))
        self.asr, self.llm, self.tts = self._create_providers()
        self.microphone = None

        if self._param("microphone_enabled"):
            self.asr.start(self._on_asr_partial, self._on_asr_final)
            self.microphone = MicrophoneFrontend(
                on_audio=self.asr.push_audio,
                on_silence=self.asr.commit,
                echo_canceller=self.echo_canceller,
                sample_rate=self._param("audio_sample_rate"),
                frame_ms=self._param("audio_frame_ms"),
                vad_threshold=self._param("vad_rms_threshold"),
                silence_timeout_s=self._param("silence_timeout_s"),
                input_device_index=self._optional_device_index(),
            )
            self.microphone.start()

        self._publish_state("listening")
        self.get_logger().info(
            f"online agent ready: mode={self.mode}, microphone={self._param('microphone_enabled')}"
        )

    def _declare_parameters(self):
        defaults = {
            "mode": "mock",
            "microphone_enabled": False,
            "speaker_enabled": False,
            "input_device_index": -1,
            "audio_sample_rate": 16000,
            "tts_sample_rate": 24000,
            "audio_frame_ms": 20,
            "vad_rms_threshold": 0.018,
            "silence_timeout_s": 0.4,
            "wake_word_enabled": True,
            "wake_words": ["小智", "你好小智"],
            "wake_active_timeout_s": 10.0,
            "aec_taps": 64,
            "aec_step": 0.35,
            "memory_path": "~/.ros/embodied_agent/memory.json",
            "memory_max_turns": 10,
            "system_prompt_path": "",
            "llm_model": "qwen-plus",
            "llm_base_url": "https://dashscope.aliyuncs.com/compatible-mode/v1",
            "llm_temperature": 0.2,
            "asr_model": "qwen3-asr-flash-realtime",
            "asr_url": "wss://dashscope.aliyuncs.com/api-ws/v1/realtime",
            "asr_language": "zh",
            "tts_model": "qwen3-tts-flash-realtime",
            "tts_voice": "Cherry",
            "tts_url": "wss://dashscope.aliyuncs.com/api-ws/v1/realtime",
            "tts_language": "Chinese",
            "tts_chunk_max_chars": 32,
            "llm_first_token_target_ms": 1000.0,
            "tts_first_audio_target_ms": 300.0,
            "mock_token_delay_s": 0.0,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _param(self, name):
        return self.get_parameter(name).value

    def _optional_device_index(self):
        value = self._param("input_device_index")
        return None if value < 0 else value

    def _load_system_prompt(self) -> str:
        configured = self._param("system_prompt_path")
        if configured:
            path = Path(os.path.expanduser(configured))
        else:
            path = Path(get_package_share_directory("embodied_online_agent")) / "prompts" / "system_prompt_zh.txt"
        return path.read_text(encoding="utf-8")

    def _create_providers(self):
        if self.mode == "mock":
            return (
                MockAsr(),
                MockLlm(self._param("mock_token_delay_s")),
                MockTts(self._param("tts_sample_rate")),
            )
        if self.mode != "online":
            raise ValueError("mode must be 'mock' or 'online'")
        return (
            QwenRealtimeAsr(
                model=self._param("asr_model"),
                url=self._param("asr_url"),
                sample_rate=self._param("audio_sample_rate"),
                language=self._param("asr_language"),
            ),
            OpenAiCompatibleLlm(
                model=self._param("llm_model"),
                base_url=self._param("llm_base_url"),
                temperature=self._param("llm_temperature"),
            ),
            QwenRealtimeTts(
                model=self._param("tts_model"),
                voice=self._param("tts_voice"),
                url=self._param("tts_url"),
                language=self._param("tts_language"),
            ),
        )

    def _on_asr_partial(self, text: str):
        self.asr_partial_pub.publish(String(data=text))

    def _on_asr_final(self, text: str):
        self.asr_final_pub.publish(String(data=text))
        self._accept_transcript(text)

    def _on_text_input(self, message: String):
        self.asr_final_pub.publish(String(data=message.data))
        self._accept_transcript(message.data)

    def _on_clear_memory(self, _message: Empty):
        self.memory.clear()
        self.get_logger().info("conversation memory cleared")

    def _accept_transcript(self, transcript: str):
        command = self.wake_gate.process(transcript)
        if command is None:
            if self._param("wake_word_enabled") and not self.wake_gate.active:
                self._publish_state("waiting_for_wake_word")
            return
        with self._state_lock:
            if self._busy:
                self.get_logger().warning("agent is busy; dropping overlapping utterance")
                return
            self._busy = True
        self.metrics.reset()
        self.metrics.mark_asr_final()
        threading.Thread(target=self._run_turn, args=(command,), daemon=True).start()

    def _run_turn(self, user_text: str):
        self._publish_state("thinking")
        text_queue = queue.Queue()
        tts_errors = []
        first_tts_text = False

        def text_chunks() -> Iterable[str]:
            while True:
                item = text_queue.get()
                if item is None:
                    return
                yield item

        def run_tts():
            try:
                self.tts.synthesize(text_chunks(), self._on_tts_audio)
            except Exception as exc:
                tts_errors.append(exc)
                self.get_logger().error(f"TTS failed: {exc}")

        tts_thread = threading.Thread(target=run_tts, daemon=True)
        tts_thread.start()
        parser = TaggedStreamParser()
        chunker = SentenceChunker(self._param("tts_chunk_max_chars"))
        spoken_parts = []
        first_token = True

        try:
            messages = [{"role": "system", "content": self.system_prompt}]
            messages.extend(self.memory.messages())
            messages.append({"role": "user", "content": user_text})
            self.metrics.mark_llm_requested()
            for token in self.llm.stream(messages):
                if first_token:
                    self.metrics.mark_llm_first_token()
                    first_token = False
                events = parser.feed(token)
                for delta in events.speech:
                    spoken_parts.append(delta)
                    self.response_delta_pub.publish(String(data=delta))
                    for speakable in chunker.feed(delta):
                        if not first_tts_text:
                            self.metrics.mark_tts_requested()
                            first_tts_text = True
                            self._publish_state("speaking")
                        text_queue.put(speakable)
                self._publish_actions(events.actions)
                for error in events.errors:
                    self.get_logger().warning(error)

            final_events = parser.finish()
            self._publish_actions(final_events.actions)
            for error in final_events.errors:
                self.get_logger().warning(error)
            for speakable in chunker.finish():
                if not first_tts_text:
                    self.metrics.mark_tts_requested()
                    first_tts_text = True
                text_queue.put(speakable)
            text_queue.put(None)
            tts_thread.join(timeout=35.0)
            if tts_thread.is_alive():
                raise TimeoutError("TTS worker did not stop")
            if tts_errors:
                raise tts_errors[0]

            assistant_text = "".join(spoken_parts).strip()
            self.response_pub.publish(String(data=assistant_text))
            self.memory.append_turn(user_text, assistant_text)
            self._publish_metrics()
        except Exception as exc:
            text_queue.put(None)
            self.get_logger().error(f"agent turn failed: {exc}")
            self._publish_state("error")
        finally:
            with self._state_lock:
                self._busy = False
            if not self._stopping:
                self._publish_state("listening")

    def _publish_actions(self, actions):
        for action in actions:
            try:
                safe = self.action_validator.validate(action)
                payload = json.dumps(safe.as_dict(), ensure_ascii=False)
                self.action_pub.publish(String(data=payload))
                self.get_logger().info(f"action: {payload}")
            except ValueError as exc:
                self.get_logger().warning(f"action rejected: {exc}")

    def _on_tts_audio(self, pcm16: bytes):
        self.metrics.mark_tts_first_audio()
        self.echo_canceller.add_reference(pcm16)
        if self._param("speaker_enabled"):
            self.speaker.write(pcm16)

    def _publish_metrics(self):
        snapshot = self.metrics.snapshot().as_dict()
        llm_ms = snapshot["llm_first_token_ms"]
        tts_ms = snapshot["tts_first_audio_ms"]
        snapshot["llm_target_met"] = (
            llm_ms is not None and llm_ms < self._param("llm_first_token_target_ms")
        )
        snapshot["tts_target_met"] = (
            tts_ms is not None and tts_ms < self._param("tts_first_audio_target_ms")
        )
        self.metrics_pub.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))
        self.get_logger().info(f"latency: {snapshot}")

    def _publish_state(self, state: str):
        self.state_pub.publish(String(data=state))

    def shutdown(self):
        self._stopping = True
        if self.microphone is not None:
            self.microphone.stop()
        self.asr.stop()
        self.speaker.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = OnlineAgentNode()
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

