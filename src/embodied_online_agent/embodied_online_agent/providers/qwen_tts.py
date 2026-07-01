import base64
import os
import threading
from typing import Callable, Iterable

from .base import TtsProvider


class QwenRealtimeTts(TtsProvider):
    """Persistent Qwen3-TTS server-commit session with per-response commits."""

    def __init__(
        self,
        model: str = "qwen3-tts-flash-realtime",
        voice: str = "Cherry",
        url: str = "wss://dashscope.aliyuncs.com/api-ws/v1/realtime",
        language: str = "Chinese",
    ):
        self.model = model
        self.voice = voice
        self.url = os.getenv("DASHSCOPE_WS_URL", url)
        self.language = language
        self._synthesizer = None
        self._connected = False
        self._session_ready = threading.Event()
        self._response_done = threading.Event()
        self._connection_lock = threading.Lock()
        self._operation_lock = threading.Lock()
        self._callback_error = []
        self._on_audio = None

    def connect(self) -> None:
        """Open the WebSocket ahead of the first spoken response."""
        if not os.getenv("DASHSCOPE_API_KEY"):
            raise RuntimeError("DASHSCOPE_API_KEY is required in online mode")
        with self._connection_lock:
            if self._connected:
                return
            try:
                import dashscope
                from dashscope.audio.qwen_tts_realtime import (
                    AudioFormat,
                    QwenTtsRealtime,
                    QwenTtsRealtimeCallback,
                )
            except ImportError as exc:
                raise RuntimeError("install requirements.txt for Qwen realtime TTS") from exc

            owner = self

            class Callback(QwenTtsRealtimeCallback):
                def on_open(self):
                    return None

                def on_close(self, code, message):
                    del code, message
                    owner._connected = False
                    owner._response_done.set()

                def on_event(self, response):
                    try:
                        event_type = response.get("type")
                        if event_type == "session.updated":
                            owner._session_ready.set()
                        elif event_type == "response.audio.delta" and owner._on_audio:
                            owner._on_audio(base64.b64decode(response["delta"]))
                        elif event_type in {"response.done", "session.finished"}:
                            owner._response_done.set()
                        elif event_type == "error":
                            owner._callback_error.append(str(response))
                            owner._response_done.set()
                    except Exception as exc:  # callback thread must unblock the caller
                        owner._callback_error.append(str(exc))
                        owner._response_done.set()

            dashscope.api_key = os.environ["DASHSCOPE_API_KEY"]
            self._session_ready.clear()
            self._callback_error.clear()
            self._synthesizer = QwenTtsRealtime(
                model=self.model, callback=Callback(), url=self.url
            )
            self._synthesizer.connect()
            self._synthesizer.update_session(
                voice=self.voice,
                response_format=AudioFormat.PCM_24000HZ_MONO_16BIT,
                language_type=self.language,
                mode="server_commit",
            )
            if not self._session_ready.wait(timeout=5.0):
                self._synthesizer.close()
                self._synthesizer = None
                if self._callback_error:
                    raise RuntimeError(self._callback_error[0])
                raise TimeoutError("Qwen TTS session was not ready within 5 seconds")
            self._connected = True

    def synthesize(self, text_chunks: Iterable[str], on_audio: Callable[[bytes], None]):
        with self._operation_lock:
            self.connect()
            self._response_done.clear()
            self._callback_error.clear()
            self._on_audio = on_audio
            sent = False
            try:
                for text in text_chunks:
                    if text:
                        sent = True
                        self._synthesizer.append_text(text)
                if not sent:
                    return
                self._synthesizer.commit()
                if not self._response_done.wait(timeout=30.0):
                    raise TimeoutError("Qwen TTS response did not finish within 30 seconds")
                if self._callback_error:
                    raise RuntimeError(self._callback_error[0])
                if not self._connected:
                    raise RuntimeError("Qwen TTS connection closed before response completed")
            finally:
                self._on_audio = None

    def close(self) -> None:
        with self._connection_lock:
            if self._synthesizer is None:
                return
            try:
                self._synthesizer.finish()
            except Exception:
                pass
            finally:
                self._synthesizer.close()
                self._synthesizer = None
                self._connected = False
