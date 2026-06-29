import audioop
import threading
import time
from array import array
from typing import Callable, Optional


class EnergyVad:
    """Small RMS VAD used for local end-of-utterance detection."""

    def __init__(self, threshold: float = 0.018):
        self.threshold = threshold

    def is_speech(self, pcm16: bytes) -> bool:
        if not pcm16:
            return False
        rms = audioop.rms(pcm16, 2) / 32768.0
        return rms >= self.threshold


class SilenceTimeout:
    """Emits once when speech is followed by configured silence."""

    def __init__(self, timeout_s: float = 0.4, clock=time.monotonic):
        self.timeout_s = timeout_s
        self._clock = clock
        self._heard_speech = False
        self._last_speech_at: Optional[float] = None
        self._emitted = False

    def update(self, speech: bool) -> bool:
        now = self._clock()
        if speech:
            self._heard_speech = True
            self._last_speech_at = now
            self._emitted = False
            return False
        if (
            self._heard_speech
            and not self._emitted
            and self._last_speech_at is not None
            and now - self._last_speech_at >= self.timeout_s
        ):
            self._emitted = True
            self._heard_speech = False
            return True
        return False


class NlmsEchoCanceller:
    """Lightweight normalized-LMS acoustic echo canceller for mono PCM16.

    This is deliberately dependency-free and suitable as a baseline. Production
    hardware should tune delay/taps or replace it behind the same process/add_reference
    seam with its DSP/WebRTC AEC.
    """

    def __init__(
        self,
        mic_rate: int = 16000,
        reference_rate: int = 24000,
        taps: int = 64,
        step: float = 0.35,
    ):
        self.mic_rate = mic_rate
        self.reference_rate = reference_rate
        self.taps = max(8, taps)
        self.step = step
        self.weights = [0.0] * self.taps
        self.history = [0.0] * self.taps
        self._reference_buffer = bytearray()
        self._reference_lock = threading.Lock()
        self._rate_state = None

    def add_reference(self, pcm16: bytes) -> None:
        if not pcm16:
            return
        if self.reference_rate != self.mic_rate:
            pcm16, self._rate_state = audioop.ratecv(
                pcm16,
                2,
                1,
                self.reference_rate,
                self.mic_rate,
                self._rate_state,
            )
        with self._reference_lock:
            self._reference_buffer.extend(pcm16)
            max_bytes = self.mic_rate * 2 * 2  # cap queued reference at two seconds
            if len(self._reference_buffer) > max_bytes:
                del self._reference_buffer[:-max_bytes]

    def process(self, microphone_pcm16: bytes) -> bytes:
        with self._reference_lock:
            if len(self._reference_buffer) < len(microphone_pcm16):
                return microphone_pcm16
            reference_pcm = bytes(self._reference_buffer[: len(microphone_pcm16)])
            del self._reference_buffer[: len(microphone_pcm16)]

        mic = array("h")
        ref = array("h")
        mic.frombytes(microphone_pcm16)
        ref.frombytes(reference_pcm)
        output = array("h")
        for mic_value, ref_value in zip(mic, ref):
            normalized_ref = ref_value / 32768.0
            self.history.pop()
            self.history.insert(0, normalized_ref)
            predicted = sum(w * x for w, x in zip(self.weights, self.history))
            desired = mic_value / 32768.0
            error = desired - predicted
            energy = 1e-6 + sum(x * x for x in self.history)
            scale = self.step * error / energy
            self.weights = [
                w + scale * x for w, x in zip(self.weights, self.history)
            ]
            output.append(max(-32768, min(32767, int(error * 32768.0))))

        if len(mic) > len(ref):
            output.extend(mic[len(ref) :])
        return output.tobytes()


class PcmSpeaker:
    def __init__(self, sample_rate: int = 24000):
        self.sample_rate = sample_rate
        self._pyaudio = None
        self._stream = None
        self._lock = threading.Lock()

    def write(self, pcm16: bytes) -> None:
        if not pcm16:
            return
        with self._lock:
            if self._stream is None:
                try:
                    import pyaudio
                except ImportError as exc:
                    raise RuntimeError("PyAudio is required for speaker output") from exc
                self._pyaudio = pyaudio.PyAudio()
                self._stream = self._pyaudio.open(
                    format=pyaudio.paInt16,
                    channels=1,
                    rate=self.sample_rate,
                    output=True,
                )
            self._stream.write(pcm16)

    def close(self) -> None:
        with self._lock:
            if self._stream is not None:
                self._stream.stop_stream()
                self._stream.close()
                self._stream = None
            if self._pyaudio is not None:
                self._pyaudio.terminate()
                self._pyaudio = None


class MicrophoneFrontend:
    """Captures 20 ms PCM frames and runs AEC, VAD and silence timeout."""

    def __init__(
        self,
        on_audio: Callable[[bytes], None],
        on_silence: Callable[[], None],
        echo_canceller: NlmsEchoCanceller,
        sample_rate: int = 16000,
        frame_ms: int = 20,
        vad_threshold: float = 0.018,
        silence_timeout_s: float = 0.4,
        input_device_index: int | None = None,
    ):
        self.on_audio = on_audio
        self.on_silence = on_silence
        self.echo_canceller = echo_canceller
        self.sample_rate = sample_rate
        self.frames_per_buffer = sample_rate * frame_ms // 1000
        self.vad = EnergyVad(vad_threshold)
        self.silence = SilenceTimeout(silence_timeout_s)
        self.input_device_index = input_device_index
        self._pyaudio = None
        self._stream = None

    def start(self) -> None:
        try:
            import pyaudio
        except ImportError as exc:
            raise RuntimeError("PyAudio is required for microphone capture") from exc
        self._pyaudio = pyaudio.PyAudio()

        def callback(in_data, frame_count, time_info, status):
            del frame_count, time_info, status
            cleaned = self.echo_canceller.process(in_data)
            self.on_audio(cleaned)
            if self.silence.update(self.vad.is_speech(cleaned)):
                self.on_silence()
            return (None, pyaudio.paContinue)

        self._stream = self._pyaudio.open(
            format=pyaudio.paInt16,
            channels=1,
            rate=self.sample_rate,
            input=True,
            input_device_index=self.input_device_index,
            frames_per_buffer=self.frames_per_buffer,
            stream_callback=callback,
        )
        self._stream.start_stream()

    def stop(self) -> None:
        if self._stream is not None:
            self._stream.stop_stream()
            self._stream.close()
            self._stream = None
        if self._pyaudio is not None:
            self._pyaudio.terminate()
            self._pyaudio = None
