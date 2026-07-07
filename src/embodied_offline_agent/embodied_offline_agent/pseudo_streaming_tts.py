import time
import threading
from dataclasses import asdict, dataclass
from typing import Callable

from .double_buffer import DoubleBuffer


@dataclass
class PseudoStreamingTtsMetrics:
    text_chunks: int = 0
    synth_calls: int = 0
    audio_chunks: int = 0
    synth_total_ms: float = 0.0
    first_text_to_first_audio_ms: float | None = None
    message_buffer_dropped: int = 0
    audio_buffer_dropped: int = 0
    message_buffer_high_watermark: int = 0
    audio_buffer_high_watermark: int = 0

    def as_dict(self) -> dict:
        return asdict(self)


class PseudoStreamingTtsPipeline:
    """Sentence-level pseudo streaming TTS with message/audio double buffers.

    Sherpa/SummerTTS 类离线 TTS 通常一次生成一整句 PCM，不像云端 TTS 那样天然逐帧流式。
    这里把 LLM 增量文本切成短句，TTS worker 负责合成，audio worker 负责按 PCM 小块发布。
    这样 LLM、TTS、音频发布三段可以并行推进，避免“整段回复生成完才开始说话”。
    """

    def __init__(
        self,
        *,
        synthesize: Callable[[str], bytes],
        publish_audio: Callable[[bytes], None],
        sample_rate: int,
        pcm_chunk_ms: int,
        on_first_audio: Callable[[], None] | None = None,
        put_timeout_s: float = 5.0,
    ):
        self._synthesize = synthesize
        self._publish_audio = publish_audio
        self._sample_rate = sample_rate
        self._pcm_chunk_ms = pcm_chunk_ms
        self._on_first_audio = on_first_audio
        self._put_timeout_s = put_timeout_s
        self._message_buffer = DoubleBuffer[str](drop_oldest=False)
        self._audio_buffer = DoubleBuffer[bytes](drop_oldest=False)
        self._errors: list[Exception] = []
        self._metrics = PseudoStreamingTtsMetrics()
        self._first_text_at: float | None = None
        self._first_audio_seen = False
        self._tts_thread = threading.Thread(target=self._tts_worker, daemon=True)
        self._audio_thread = threading.Thread(target=self._audio_worker, daemon=True)

    def start(self) -> None:
        self._tts_thread.start()
        self._audio_thread.start()

    def put_text(self, text: str) -> bool:
        if self._first_text_at is None:
            self._first_text_at = time.perf_counter()
        accepted = self._message_buffer.put(text, timeout=self._put_timeout_s)
        if accepted:
            self._metrics.text_chunks += 1
        return accepted

    def close_and_wait(self, timeout_s: float = 60.0) -> PseudoStreamingTtsMetrics:
        self._message_buffer.close()
        self._tts_thread.join(timeout=timeout_s)
        self._audio_thread.join(timeout=timeout_s)
        if self._tts_thread.is_alive() or self._audio_thread.is_alive():
            raise TimeoutError("pseudo-streaming TTS pipeline did not drain")
        if self._errors:
            raise self._errors[0]
        self._collect_buffer_stats()
        return self._metrics

    def abort(self) -> None:
        self._message_buffer.abort()
        self._audio_buffer.abort()

    @property
    def metrics(self) -> dict:
        self._collect_buffer_stats()
        return self._metrics.as_dict()

    def _tts_worker(self) -> None:
        try:
            while True:
                text = self._message_buffer.get()
                started = time.perf_counter()
                pcm = self._synthesize(text)
                self._metrics.synth_calls += 1
                self._metrics.synth_total_ms += (time.perf_counter() - started) * 1000.0
                bytes_per_chunk = max(
                    2,
                    int(self._sample_rate * self._pcm_chunk_ms / 1000) * 2,
                )
                for offset in range(0, len(pcm), bytes_per_chunk):
                    if not self._audio_buffer.put(
                        pcm[offset : offset + bytes_per_chunk],
                        timeout=self._put_timeout_s,
                    ):
                        raise TimeoutError("audio double buffer remained full")
        except StopIteration:
            pass
        except Exception as exc:
            self._errors.append(exc)
        finally:
            self._audio_buffer.close()

    def _audio_worker(self) -> None:
        try:
            while True:
                pcm = self._audio_buffer.get()
                if not self._first_audio_seen:
                    self._first_audio_seen = True
                    if self._first_text_at is not None:
                        self._metrics.first_text_to_first_audio_ms = (
                            time.perf_counter() - self._first_text_at
                        ) * 1000.0
                    if self._on_first_audio is not None:
                        self._on_first_audio()
                self._metrics.audio_chunks += 1
                self._publish_audio(pcm)
        except StopIteration:
            return

    def _collect_buffer_stats(self) -> None:
        message_stats = self._message_buffer.stats
        audio_stats = self._audio_buffer.stats
        self._metrics.message_buffer_dropped = message_stats.dropped
        self._metrics.audio_buffer_dropped = audio_stats.dropped
        self._metrics.message_buffer_high_watermark = message_stats.high_watermark
        self._metrics.audio_buffer_high_watermark = audio_stats.high_watermark
