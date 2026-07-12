"""ROS 2 sidecar for keyword spotting wake events."""

from __future__ import annotations

import rclpy
from embodied_agent_interfaces.msg import KwsEvent, KwsScore, WakeEvent
from rclpy.node import Node
from std_msgs.msg import String, UInt8MultiArray

from embodied_agent_core.ros_qos import (
    audio_qos,
    command_qos,
    event_qos,
    sensor_qos,
)
from embodied_agent_core.runtime_status_transport import (
    kws_event_to_message,
    kws_score_to_message,
)

from .keyword_wake import (
    KeywordWakeBridge,
    LiveKitWakeWordDetector,
    OpenWakeWordDetector,
    SherpaKeywordWakeDetector,
    TextKeywordWakeDetector,
)


def _string_list_param(value) -> list[str]:
    """把 ROS 参数规范成字符串列表。

    launch 文件从命令行接收 `a.onnx,b.onnx` 时通常会传成单个字符串；YAML 中则
    可能是原生 list。这里统一在节点边界做转换，避免 detector 误把字符串按字符迭代。
    """

    if value is None:
        return []
    if isinstance(value, str):
        return [item.strip() for item in value.split(",") if item.strip()]
    try:
        return [str(item).strip() for item in value if str(item).strip()]
    except TypeError:
        text = str(value).strip()
        return [text] if text else []


class KeywordWakeNode(Node):
    """把 KWS detector 输出桥接到 `/agent/wake_event_input`。

    - `mock_text`：无外部依赖，订阅 `/agent/kws_text_input`，适合验收 sidecar 链路。
    - `sherpa`：订阅 `/audio/clean_pcm`，使用 sherpa-onnx KeywordSpotter。
    - `openwakeword`：订阅 `/audio/clean_pcm`，使用可选 openWakeWord 模型。
    - `livekit`：订阅 `/audio/clean_pcm`，使用可选 LiveKit WakeWord 模型。
    """

    def __init__(self):
        super().__init__("keyword_wake")
        self._declare_parameters()
        self._mode = str(self.get_parameter("mode").value)
        self._bridge = KeywordWakeBridge(
            cooldown_s=float(self.get_parameter("cooldown_s").value)
        )
        self._score_period_s = max(
            0.0, float(self.get_parameter("score_publish_period_s").value)
        )
        self._next_score_publish_at = 0.0
        self._wake_pub = self.create_publisher(
            WakeEvent, "/agent/wake_event_input", command_qos(depth=10)
        )
        self._event_pub = self.create_publisher(
            KwsEvent, "/agent/kws_event", event_qos(depth=10)
        )
        self._score_pub = self.create_publisher(
            KwsScore, "/agent/kws_score", sensor_qos(depth=5)
        )
        # 声学 provider 共用同一个实时 PCM profile，避免不同实现产生 DDS
        # compatibility 漂移；队列深度仍允许通过节点参数按算力调节。
        audio_profile = audio_qos(
            depth=int(self.get_parameter("input_queue_depth").value)
        )

        if self._mode == "disabled":
            self.get_logger().info("keyword wake sidecar disabled")
            return
        if self._mode == "mock_text":
            self._detector = TextKeywordWakeDetector(
                self.get_parameter("keywords").value,
                aliases=self.get_parameter("aliases").value,
                provider_name=str(self.get_parameter("provider_name").value),
            )
            self.create_subscription(
                String,
                "/agent/kws_text_input",
                self._on_text,
                command_qos(depth=10),
            )
        elif self._mode == "sherpa":
            self._detector = SherpaKeywordWakeDetector(
                tokens=str(self.get_parameter("sherpa_tokens").value),
                encoder=str(self.get_parameter("sherpa_encoder").value),
                decoder=str(self.get_parameter("sherpa_decoder").value),
                joiner=str(self.get_parameter("sherpa_joiner").value),
                keywords_file=str(self.get_parameter("sherpa_keywords_file").value),
                sample_rate=int(self.get_parameter("sample_rate").value),
                num_threads=int(self.get_parameter("sherpa_num_threads").value),
                provider=str(self.get_parameter("sherpa_provider").value),
                provider_name=str(self.get_parameter("provider_name").value),
            )
            self.create_subscription(
                UInt8MultiArray, "/audio/clean_pcm", self._on_audio, audio_profile
            )
        elif self._mode == "openwakeword":
            self._detector = OpenWakeWordDetector(
                model_paths=_string_list_param(
                    self.get_parameter("openwakeword_models").value
                ),
                threshold=float(self.get_parameter("openwakeword_threshold").value),
                inference_framework=str(
                    self.get_parameter("openwakeword_inference_framework").value
                ),
                vad_threshold=float(self.get_parameter("openwakeword_vad_threshold").value),
                enable_speex_noise_suppression=bool(
                    self.get_parameter("openwakeword_speex_noise_suppression").value
                ),
                provider_name=str(self.get_parameter("provider_name").value),
            )
            self.create_subscription(
                UInt8MultiArray, "/audio/clean_pcm", self._on_audio, audio_profile
            )
        elif self._mode == "livekit":
            self._detector = LiveKitWakeWordDetector(
                model_paths=_string_list_param(
                    self.get_parameter("livekit_wakeword_models").value
                ),
                threshold=float(self.get_parameter("livekit_wakeword_threshold").value),
                provider_name=str(self.get_parameter("provider_name").value),
            )
            self.create_subscription(
                UInt8MultiArray, "/audio/clean_pcm", self._on_audio, audio_profile
            )
        else:
            raise ValueError(
                "mode must be disabled, mock_text, sherpa, openwakeword, or livekit"
            )

        self.get_logger().info(
            "keyword wake sidecar ready: mode=%s provider=%s"
            % (self._mode, self.get_parameter("provider_name").value)
        )

    def _declare_parameters(self):
        defaults = {
            "mode": "mock_text",
            "provider_name": "mock_kws",
            "keywords": ["小智", "你好小智"],
            "aliases": ["小志", "小治", "晓智", "晓志"],
            "sample_rate": 16000,
            "cooldown_s": 1.0,
            "score_publish_period_s": 0.2,
            "input_queue_depth": 20,
            "sherpa_tokens": "",
            "sherpa_encoder": "",
            "sherpa_decoder": "",
            "sherpa_joiner": "",
            "sherpa_keywords_file": "",
            "sherpa_num_threads": 1,
            "sherpa_provider": "cpu",
            "openwakeword_models": [""],
            "openwakeword_threshold": 0.5,
            "openwakeword_inference_framework": "onnx",
            "openwakeword_vad_threshold": -1.0,
            "openwakeword_speex_noise_suppression": False,
            "livekit_wakeword_models": [""],
            "livekit_wakeword_threshold": 0.5,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_text(self, message: String):
        self._handle_match(self._detector.detect_text(message.data))

    def _on_audio(self, message: UInt8MultiArray):
        self._handle_match(self._detector.detect_audio(bytes(message.data)))
        self._publish_scores_if_available()

    def _handle_match(self, match):
        accepted = self._bridge.accept(match)
        if accepted is None:
            return
        stamp = self.get_clock().now().to_msg()
        wake = WakeEvent()
        wake.stamp = stamp
        wake.kind = WakeEvent.KIND_WAKE
        wake.provider = accepted.provider
        wake.transcript = accepted.keyword
        self._wake_pub.publish(wake)
        self._event_pub.publish(kws_event_to_message(accepted, stamp=stamp))

    def _publish_scores_if_available(self):
        if not hasattr(self._detector, "last_scores"):
            return
        scores = self._detector.last_scores()
        threshold = getattr(self._detector, "threshold", 0.0)
        message = kws_score_to_message(
            provider=str(self.get_parameter("provider_name").value),
            scores=scores,
            threshold=float(threshold),
            stamp=self.get_clock().now().to_msg(),
        )
        if message is None:
            return
        now = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if now < self._next_score_publish_at:
            return
        self._next_score_publish_at = now + self._score_period_s
        self._score_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(KeywordWakeNode())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
