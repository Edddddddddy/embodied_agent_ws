from array import array

from embodied_online_agent.audio import EnergyVad, NlmsEchoCanceller, SilenceTimeout


class Clock:
    value = 0.0

    def __call__(self):
        return self.value


def test_silence_timeout_emits_once_after_speech():
    clock = Clock()
    timeout = SilenceTimeout(0.4, clock=clock)
    assert not timeout.update(True)
    clock.value = 0.39
    assert not timeout.update(False)
    clock.value = 0.41
    assert timeout.update(False)
    clock.value = 1.0
    assert not timeout.update(False)


def test_energy_vad_distinguishes_silence():
    vad = EnergyVad(0.01)
    assert not vad.is_speech(array("h", [0] * 100).tobytes())
    assert vad.is_speech(array("h", [1000] * 100).tobytes())


def test_echo_canceller_preserves_mic_without_reference():
    samples = array("h", [10, -20, 30]).tobytes()
    canceller = NlmsEchoCanceller(mic_rate=16000, reference_rate=16000)
    assert canceller.process(samples) == samples

