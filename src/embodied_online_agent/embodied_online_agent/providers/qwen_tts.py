import base64
import os
import threading
from typing import Callable, Iterable

from .base import TtsProvider


class QwenRealtimeTts(TtsProvider):
    """Qwen3-TTS-Realtime server-commit streaming adapter."""

    def __init__(
        self,
        model: str = "qwen3-tts-flash-realtime",
        voice: str = "Cherry",
        url: str = "wss://dashscope.aliyuncs.com/api-ws/v1/realtime",
        language: str = "Chinese",
    ):
        self.model = model
        self.voice = voice
        self.url = url
        self.language = language

    def synthesize(self, text_chunks: Iterable[str], on_audio: Callable[[bytes], None]):
        if not os.getenv("DASHSCOPE_API_KEY"):
            raise RuntimeError("DASHSCOPE_API_KEY is required in online mode")
        try:
            import dashscope
            from dashscope.audio.qwen_tts_realtime import (
                AudioFormat,
                QwenTtsRealtime,
                QwenTtsRealtimeCallback,
            )
        except ImportError as exc:
            raise RuntimeError("install requirements.txt for Qwen realtime TTS") from exc

        dashscope.api_key = os.environ["DASHSCOPE_API_KEY"]
        completed = threading.Event()
        callback_error = []

        class Callback(QwenTtsRealtimeCallback):
            def on_open(self):
                return None

            def on_close(self, code, message):
                del code, message

            def on_event(self, response):
                try:
                    event_type = response.get("type")
                    if event_type == "response.audio.delta":
                        on_audio(base64.b64decode(response["delta"]))
                    elif event_type == "session.finished":
                        completed.set()
                    elif event_type == "error":
                        callback_error.append(str(response))
                        completed.set()
                except Exception as exc:  # callback thread must unblock the caller
                    callback_error.append(str(exc))
                    completed.set()

        synthesizer = QwenTtsRealtime(
            model=self.model,
            callback=Callback(),
            url=self.url,
        )
        synthesizer.connect()
        synthesizer.update_session(
            voice=self.voice,
            response_format=AudioFormat.PCM_24000HZ_MONO_16BIT,
            language_type=self.language,
            mode="server_commit",
        )
        sent = False
        for text in text_chunks:
            if text:
                sent = True
                synthesizer.append_text(text)
        if not sent:
            synthesizer.close()
            return
        synthesizer.finish()
        if not completed.wait(timeout=30.0):
            synthesizer.close()
            raise TimeoutError("Qwen TTS did not finish within 30 seconds")
        if callback_error:
            raise RuntimeError(callback_error[0])

