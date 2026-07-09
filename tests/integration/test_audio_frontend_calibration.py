import importlib.util
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "audio_frontend_calibration.py"
spec = importlib.util.spec_from_file_location("audio_frontend_calibration", SCRIPT)
audio_calibration = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = audio_calibration
spec.loader.exec_module(audio_calibration)


def test_parse_audio_metrics_accepts_frontend_json():
    sample = audio_calibration.parse_audio_metrics(
        '{"rms": 0.023, "peak": 1234, "speech": true, '
        '"dropped_input_frames": 2, "dropped_playback_chunks": 1, '
        '"vad_provider": "energy", '
        '"audio_enhancer_requested": "webrtc", "audio_enhancer_active": "nlms", '
        '"aec_active": true, '
        '"noise_suppression_requested": true, "noise_suppression_active": false, '
        '"auto_gain_requested": true, "auto_gain_active": false}'
    )

    assert sample == audio_calibration.AudioMetricSample(
        rms=0.023,
        peak=1234.0,
        speech=True,
        dropped_input_frames=2,
        dropped_playback_chunks=1,
        vad_provider="energy",
        audio_enhancer_requested="webrtc",
        audio_enhancer_active="nlms",
        aec_active=True,
        noise_suppression_requested=True,
        noise_suppression_active=False,
        auto_gain_requested=True,
        auto_gain_active=False,
    )


def test_quiet_microphone_produces_actionable_warning():
    samples = [
        audio_calibration.AudioMetricSample(rms=0.0001, peak=3, speech=False),
        audio_calibration.AudioMetricSample(rms=0.0002, peak=6, speech=False),
    ]

    report = audio_calibration.analyze_audio_health(samples)

    assert report.ok is False
    assert "microphone_too_quiet_or_disconnected" in report.warnings
    assert report.suggested_vad_threshold < 0.006


def test_low_gain_microphone_recommends_low_gain_profile_and_low_threshold():
    samples = [
        audio_calibration.AudioMetricSample(rms=0.0003, peak=23, speech=False),
        audio_calibration.AudioMetricSample(rms=0.0023, peak=180, speech=False),
    ]

    report = audio_calibration.analyze_audio_health(samples)

    assert "microphone_low_gain" in report.warnings
    assert report.recommended_voice_profile == "low_gain"
    assert report.profile_reason == "low_gain_input_detected_but_below_default_vad"
    assert 0.0008 <= report.suggested_vad_threshold <= 0.002


def test_loud_audio_without_speech_suggests_vad_threshold_is_high():
    samples = [
        audio_calibration.AudioMetricSample(rms=0.025, peak=1200, speech=False),
        audio_calibration.AudioMetricSample(rms=0.032, peak=1500, speech=False),
    ]

    report = audio_calibration.analyze_audio_health(samples)

    assert "vad_threshold_may_be_too_high" in report.warnings
    assert report.suggested_vad_threshold > 0.01
    assert report.recommended_voice_profile == "quiet"


def test_stable_speech_samples_pass_health_check():
    samples = [
        audio_calibration.AudioMetricSample(rms=0.006, peak=100, speech=False),
        audio_calibration.AudioMetricSample(rms=0.026, peak=1400, speech=True),
        audio_calibration.AudioMetricSample(rms=0.03, peak=1600, speech=True),
        audio_calibration.AudioMetricSample(rms=0.007, peak=120, speech=False),
    ]

    report = audio_calibration.analyze_audio_health(samples)

    assert report.ok is True
    assert report.speech_ratio == 0.5
    assert report.recommended_voice_profile == "normal"


def test_noisy_room_samples_recommend_noisy_room_profile():
    samples = [
        audio_calibration.AudioMetricSample(rms=0.019, peak=900, speech=True),
        audio_calibration.AudioMetricSample(rms=0.021, peak=950, speech=True),
        audio_calibration.AudioMetricSample(rms=0.018, peak=850, speech=True),
        audio_calibration.AudioMetricSample(rms=0.02, peak=920, speech=True),
    ]

    report = audio_calibration.analyze_audio_health(samples)

    assert "vad_threshold_may_be_too_low_or_environment_noisy" in report.warnings
    assert report.recommended_voice_profile == "noisy_room"
    assert "persistent_speech_or_noise" in report.profile_reason


def test_low_mid_input_without_warning_keeps_normal_profile():
    samples = [
        audio_calibration.AudioMetricSample(rms=0.006, peak=220, speech=False),
        audio_calibration.AudioMetricSample(rms=0.009, peak=360, speech=False),
        audio_calibration.AudioMetricSample(rms=0.008, peak=300, speech=False),
    ]

    report = audio_calibration.analyze_audio_health(samples)

    assert report.recommended_voice_profile == "normal"
    assert "balanced_audio_frontend" in report.profile_reason


def test_dropped_frames_are_reported_from_counter_delta():
    samples = [
        audio_calibration.AudioMetricSample(
            rms=0.006, peak=80, speech=False, dropped_input_frames=3
        ),
        audio_calibration.AudioMetricSample(
            rms=0.018, peak=800, speech=True, dropped_input_frames=5
        ),
    ]

    report = audio_calibration.analyze_audio_health(samples)

    assert report.dropped_input_delta == 2
    assert "audio_input_overrun" in report.warnings


def test_audio_enhancer_fallbacks_are_reported():
    samples = [
        audio_calibration.AudioMetricSample(
            rms=0.02,
            peak=1000,
            speech=True,
            audio_enhancer_requested="webrtc",
            audio_enhancer_active="nlms",
            noise_suppression_requested=True,
            noise_suppression_active=False,
            auto_gain_requested=True,
            auto_gain_active=False,
        )
    ]

    report = audio_calibration.analyze_audio_health(samples)

    assert report.audio_enhancer_requested == "webrtc"
    assert report.audio_enhancer_active == "nlms"
    assert "audio_enhancer_fallback" in report.warnings
    assert "noise_suppression_unavailable" in report.warnings
    assert "auto_gain_unavailable" in report.warnings


def test_format_report_explains_warnings_in_chinese():
    report = audio_calibration.analyze_audio_health([])

    rendered = audio_calibration.format_report(report)

    assert "WARN: audio frontend calibration" in rendered
    assert "没有收到 /audio/frontend_metrics" in rendered
    assert "recommended VOICE_CONTROL_PROFILE: normal" in rendered
    assert "export VOICE_CONTROL_PROFILE=normal" in rendered
