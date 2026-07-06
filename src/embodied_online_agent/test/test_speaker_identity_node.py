import struct

from embodied_online_agent.speaker_identity_node import AudioWindow


def test_audio_window_rms_and_bounded_append():
    window = AudioWindow(bytearray())
    frame = struct.pack("<hhhh", 1000, -1000, 1000, -1000)

    window.append(frame, max_bytes=len(frame))
    assert 0.02 < window.rms() < 0.04

    window.append(frame, max_bytes=len(frame))
    assert len(window.pcm) == len(frame)


def test_audio_window_clear():
    window = AudioWindow(bytearray(b"\x01\x00"))
    window.clear()
    assert window.rms() == 0.0
