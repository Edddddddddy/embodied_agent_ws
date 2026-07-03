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
        '"dropped_input_frames": 2, "dropped_playback_chunks": 1}'
    )

    assert sample == audio_calibration.AudioMetricSample(
        rms=0.023,
        peak=1234.0,
        speech=True,
        dropped_input_frames=2,
        dropped_playback_chunks=1,
    )


def test_quiet_microphone_produces_actionable_warning():
    samples = [
        audio_calibration.AudioMetricSample(rms=0.001, peak=10, speech=False),
        audio_calibration.AudioMetricSample(rms=0.002, peak=20, speech=False),
    ]

    report = audio_calibration.analyze_audio_health(samples)

    assert report.ok is False
    assert "microphone_too_quiet_or_disconnected" in report.warnings
    assert report.suggested_vad_threshold >= 0.006


def test_loud_audio_without_speech_suggests_vad_threshold_is_high():
    samples = [
        audio_calibration.AudioMetricSample(rms=0.025, peak=1200, speech=False),
        audio_calibration.AudioMetricSample(rms=0.032, peak=1500, speech=False),
    ]

    report = audio_calibration.analyze_audio_health(samples)

    assert "vad_threshold_may_be_too_high" in report.warnings
    assert report.suggested_vad_threshold > 0.01


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


def test_format_report_explains_warnings_in_chinese():
    report = audio_calibration.analyze_audio_health([])

    rendered = audio_calibration.format_report(report)

    assert "WARN: audio frontend calibration" in rendered
    assert "没有收到 /audio/frontend_metrics" in rendered
