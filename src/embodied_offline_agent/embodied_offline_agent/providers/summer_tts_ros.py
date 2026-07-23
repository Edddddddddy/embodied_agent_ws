from __future__ import annotations

import time

from embodied_agent_interfaces.srv import SynthesizeSpeech


class SummerTtsRosError(RuntimeError):
    """Raised when the resident SummerTTS ROS service cannot synthesize audio."""


class SummerTtsRosClient:
    """Client for the resident C++ SummerTTS service.

    与命令行 provider 不同，这个 provider 不启动外部进程；模型由 C++ service 节点
    常驻加载。Python Offline Agent 每句只发一次 service request，适合后续冲低延迟。
    """

    def __init__(
        self,
        node,
        *,
        service_name: str = "/tts/synthesize",
        timeout_s: float = 10.0,
        speaker_id: int = -1,
        length_scale: float = 0.0,
    ):
        self._node = node
        self._service_name = service_name
        self._timeout_s = float(timeout_s)
        self._speaker_id = int(speaker_id)
        self._length_scale = float(length_scale)
        self._client = node.create_client(SynthesizeSpeech, service_name)
        self.sample_rate = 16000

    def synthesize(self, text: str) -> bytes:
        if self._client is None:
            raise SummerTtsRosError("SummerTTS ROS client is closed")
        if not self._client.wait_for_service(timeout_sec=self._timeout_s):
            raise SummerTtsRosError(
                f"SummerTTS service not available: {self._service_name}"
            )
        request = SynthesizeSpeech.Request()
        request.text = text
        request.speaker_id = self._speaker_id
        request.length_scale = self._length_scale
        future = self._client.call_async(request)
        deadline = time.monotonic() + self._timeout_s
        while not future.done():
            if time.monotonic() >= deadline:
                raise SummerTtsRosError(
                    f"SummerTTS service timeout after {self._timeout_s:.1f}s"
                )
            time.sleep(0.001)
        response = future.result()
        if response is None:
            raise SummerTtsRosError("SummerTTS service returned no response")
        self.sample_rate = int(response.sample_rate or self.sample_rate)
        if not response.ok:
            raise SummerTtsRosError(response.error or "SummerTTS service failed")
        return bytes(response.pcm)

    def close(self) -> None:
        """Lifecycle cleanup 时注销 ROS client，避免反复 configure 泄漏 entity。"""

        if self._client is None:
            return
        self._node.destroy_client(self._client)
        self._client = None
