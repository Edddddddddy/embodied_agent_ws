import struct

from embodied_voice_frontend.speaker_identity_node import (
    AudioWindow,
    classify_speaker_scores,
)


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


def test_speaker_score_uses_real_top_score_and_margin():
    decision = classify_speaker_scores(
        {"alice": 0.86, "bob": 0.61}, threshold=0.6, min_margin=0.08
    )
    assert decision.matched is True
    assert decision.speaker_id == "alice"
    assert decision.confidence == 0.86
    assert round(decision.margin, 2) == 0.25


def test_speaker_score_rejects_ambiguous_identity_to_protect_memory():
    decision = classify_speaker_scores(
        {"alice": 0.82, "bob": 0.79}, threshold=0.6, min_margin=0.05
    )
    assert decision.matched is False
    assert decision.speaker_id == "unknown"
    assert decision.reason == "ambiguous_match"
    assert decision.confidence == 0.82


def test_speaker_score_rejects_low_or_missing_scores():
    low = classify_speaker_scores(
        {"alice": 0.42}, threshold=0.6, min_margin=0.05
    )
    empty = classify_speaker_scores({}, threshold=0.6, min_margin=0.05)
    assert low.reason == "below_threshold"
    assert empty.reason == "no_scores"
