"""ROS 2 sidecar for keyword spotting wake events."""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String, UInt8MultiArray

from .keyword_wake import (
    KeywordWakeBridge,
    SherpaKeywordWakeDetector,
    TextKeywordWakeDetector,
)


class KeywordWakeNode(Node):
    """把 KWS detector 输出桥接到 `/agent/wake_event_input`。

    - `mock_text`：无外部依赖，订阅 `/agent/kws_text_input`，适合验收 sidecar 链路。
    - `sherpa`：订阅 `/audio/clean_pcm`，使用 sherpa-onnx KeywordSpotter。
    """

    def __init__(self):
        super().__init__("keyword_wake")
        self._declare_parameters()
        self._mode = str(self.get_parameter("mode").value)
        self._bridge = KeywordWakeBridge(
            cooldown_s=float(self.get_parameter("cooldown_s").value)
        )
        self._wake_pub = self.create_publisher(String, "/agent/wake_event_input", 10)
        self._event_pub = self.create_publisher(String, "/agent/kws_event", 10)

        if self._mode == "disabled":
            self.get_logger().info("keyword wake sidecar disabled")
            return
        if self._mode == "mock_text":
            self._detector = TextKeywordWakeDetector(
                self.get_parameter("keywords").value,
                aliases=self.get_parameter("aliases").value,
                provider_name=str(self.get_parameter("provider_name").value),
            )
            self.create_subscription(String, "/agent/kws_text_input", self._on_text, 10)
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
            audio_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=int(self.get_parameter("input_queue_depth").value),
                reliability=ReliabilityPolicy.BEST_EFFORT,
            )
            self.create_subscription(
                UInt8MultiArray, "/audio/clean_pcm", self._on_audio, audio_qos
            )
        else:
            raise ValueError("mode must be disabled, mock_text, or sherpa")

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
            "input_queue_depth": 20,
            "sherpa_tokens": "",
            "sherpa_encoder": "",
            "sherpa_decoder": "",
            "sherpa_joiner": "",
            "sherpa_keywords_file": "",
            "sherpa_num_threads": 1,
            "sherpa_provider": "cpu",
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)

    def _on_text(self, message: String):
        self._handle_match(self._detector.detect_text(message.data))

    def _on_audio(self, message: UInt8MultiArray):
        self._handle_match(self._detector.detect_audio(bytes(message.data)))

    def _handle_match(self, match):
        payload = self._bridge.wake_payload(match)
        if payload is None:
            return
        self._wake_pub.publish(String(data=payload))
        event = json.loads(payload)
        event["status"] = "detected"
        self._event_pub.publish(String(data=json.dumps(event, ensure_ascii=False)))


def main(args=None):
    rclpy.init(args=args)
    try:
        rclpy.spin(KeywordWakeNode())
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
