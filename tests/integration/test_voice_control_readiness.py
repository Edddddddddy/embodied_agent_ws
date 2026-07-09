import importlib.util
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "voice_control_readiness_check.py"
spec = importlib.util.spec_from_file_location("voice_control_readiness_check", SCRIPT)
readiness = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = readiness
spec.loader.exec_module(readiness)


def test_readiness_passes_when_audio_and_kws_are_healthy():
    audio = readiness.AudioHealthReport(
        sample_count=4,
        max_rms=0.03,
        mean_rms=0.018,
        speech_ratio=0.5,
        dropped_input_delta=0,
        dropped_playback_delta=0,
        suggested_vad_threshold=0.014,
        warnings=(),
    )
    kws = readiness.KwsScoreReport(
        sample_count=3,
        provider="livekit",
        top_keyword="小智",
        max_top_score=0.91,
        mean_top_score=0.63,
        trigger_ratio=0.667,
        current_threshold=0.5,
        suggested_threshold=0.68,
        warnings=(),
    )

    report = readiness.build_readiness_report(audio, kws, require_kws=True)

    assert report.ok is True
    assert report.blockers == ()
    assert report.warnings == ()
    assert readiness.readiness_exit_code(report) == 0


def test_readiness_blocks_when_audio_metrics_are_missing():
    audio = readiness.analyze_audio_health([])
    kws = readiness.analyze_kws_scores([])

    report = readiness.build_readiness_report(audio, kws, require_kws=False)

    assert report.ok is False
    assert "audio:no_audio_metrics" in report.blockers
    assert "kws:not_required_or_not_running" in report.warnings
    assert readiness.readiness_exit_code(report) == 1


def test_require_kws_blocks_when_kws_scores_are_missing():
    audio = readiness.AudioHealthReport(
        sample_count=3,
        max_rms=0.02,
        mean_rms=0.01,
        speech_ratio=0.5,
        dropped_input_delta=0,
        dropped_playback_delta=0,
        suggested_vad_threshold=0.01,
        warnings=(),
    )
    kws = readiness.analyze_kws_scores([])

    report = readiness.build_readiness_report(audio, kws, require_kws=True)

    assert report.ok is False
    assert "kws:no_kws_scores" in report.blockers


def test_format_readiness_report_summarizes_next_actions():
    audio = readiness.analyze_audio_health([])
    kws = readiness.analyze_kws_scores([])
    report = readiness.build_readiness_report(audio, kws, require_kws=True)

    rendered = readiness.format_readiness_report(report)

    assert "BLOCKED: voice control readiness" in rendered
    assert "audio:no_audio_metrics" in rendered
    assert "kws:no_kws_scores" in rendered
    assert "recommended_voice_profile: normal" in rendered
    assert "quick_apply: export VOICE_CONTROL_PROFILE=normal" in rendered
    assert "quick_apply_threshold: export SPEECH_START_THRESHOLD=0.0180" in rendered


def test_readiness_recommends_low_gain_for_user_reported_low_peak_audio():
    audio = readiness.analyze_audio_health(
        [
            readiness.AudioMetricSample(rms=0.0003, peak=23, speech=False),
            readiness.AudioMetricSample(rms=0.0023, peak=180, speech=False),
        ]
    )
    kws = readiness.analyze_kws_scores([])

    report = readiness.build_readiness_report(audio, kws, require_kws=False)

    rendered = readiness.format_readiness_report(report)

    assert report.ok is True
    assert report.audio.recommended_voice_profile == "low_gain"
    assert "audio:microphone_low_gain" in report.warnings
    assert "quick_apply: export VOICE_CONTROL_PROFILE=low_gain" in rendered
    assert "quick_apply_threshold: export SPEECH_START_THRESHOLD=" in rendered


def test_readiness_report_preserves_audio_profile_advice_for_json_output():
    audio = readiness.analyze_audio_health(
        [
            readiness.AudioMetricSample(rms=0.02, peak=900, speech=True),
            readiness.AudioMetricSample(rms=0.021, peak=950, speech=True),
        ]
    )
    kws = readiness.analyze_kws_scores([])

    report = readiness.build_readiness_report(audio, kws, require_kws=False)

    assert report.audio.recommended_voice_profile == "noisy_room"
    assert report.audio.profile_reason == "persistent_speech_or_noise"
