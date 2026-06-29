import base64
import os
from typing import Callable

from .base import AsrProvider


class QwenRealtimeAsr(AsrProvider):
    """DashScope Qwen3-ASR-Realtime adapter using local 0.4 s VAD."""

    def __init__(
        self,
        model: str = "qwen3-asr-flash-realtime",
        url: str = "wss://dashscope.aliyuncs.com/api-ws/v1/realtime",
        sample_rate: int = 16000,
        language: str = "zh",
    ):
        self.model = model
        self.url = url
        self.sample_rate = sample_rate
        self.language = language
        self.conversation = None

    def start(self, on_partial: Callable[[str], None], on_final: Callable[[str], None]):
        if not os.getenv("DASHSCOPE_API_KEY"):
            raise RuntimeError("DASHSCOPE_API_KEY is required in online mode")
        try:
            import dashscope
            from dashscope.audio.qwen_omni import (
                MultiModality,
                OmniRealtimeCallback,
                OmniRealtimeConversation,
            )
            from dashscope.audio.qwen_omni.omni_realtime import TranscriptionParams
        except ImportError as exc:
            raise RuntimeError("install requirements.txt for Qwen realtime ASR") from exc

        dashscope.api_key = os.environ["DASHSCOPE_API_KEY"]

        class Callback(OmniRealtimeCallback):
            def on_open(self):
                return None

            def on_close(self, code, message):
                del code, message

            def on_event(self, response):
                event_type = response.get("type")
                if event_type == "conversation.item.input_audio_transcription.text":
                    text = response.get("stash", "")
                    if text:
                        on_partial(text)
                elif event_type == "conversation.item.input_audio_transcription.completed":
                    text = response.get("transcript", "")
                    if text:
                        on_final(text)

        self.conversation = OmniRealtimeConversation(
            model=self.model,
            url=self.url,
            callback=Callback(),
        )
        self.conversation.connect()
        self.conversation.update_session(
            output_modalities=[MultiModality.TEXT],
            enable_turn_detection=False,
            enable_input_audio_transcription=True,
            transcription_params=TranscriptionParams(
                language=self.language,
                sample_rate=self.sample_rate,
                input_audio_format="pcm",
            ),
        )

    def push_audio(self, pcm16: bytes):
        if self.conversation is not None:
            self.conversation.append_audio(base64.b64encode(pcm16).decode("ascii"))

    def commit(self):
        if self.conversation is not None:
            self.conversation.commit()

    def stop(self):
        if self.conversation is not None:
            try:
                self.conversation.end_session()
            finally:
                self.conversation.close()
                self.conversation = None
