import importlib.util
import sys
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[2] / "scripts" / "kws_score_calibration.py"
spec = importlib.util.spec_from_file_location("kws_score_calibration", SCRIPT)
kws_calibration = importlib.util.module_from_spec(spec)
assert spec.loader is not None
sys.modules[spec.name] = kws_calibration
spec.loader.exec_module(kws_calibration)


def test_parse_kws_score_accepts_score_json():
    sample = kws_calibration.parse_kws_score(
        '{"provider":"livekit","top_keyword":"小智","top_score":0.82,'
        '"threshold":0.5,"above_threshold":true,'
        '"scores":{"小智":0.82,"background":0.01}}'
    )

    assert sample == kws_calibration.KwsScoreSample(
        provider="livekit",
        top_keyword="小智",
        top_score=0.82,
        threshold=0.5,
        above_threshold=True,
        scores={"小智": 0.82, "background": 0.01},
    )


def test_no_scores_warns_that_kws_is_not_running():
    report = kws_calibration.analyze_kws_scores([])

    assert report.ok is False
    assert report.sample_count == 0
    assert report.suggested_threshold == 0.5
    assert "no_kws_scores" in report.warnings


def test_strong_wake_samples_pass_and_suggest_threshold():
    samples = [
        kws_calibration.KwsScoreSample("livekit", "小智", 0.12, 0.5, False, {}),
        kws_calibration.KwsScoreSample("livekit", "小智", 0.88, 0.5, True, {}),
        kws_calibration.KwsScoreSample("livekit", "小智", 0.91, 0.5, True, {}),
    ]

    report = kws_calibration.analyze_kws_scores(samples)

    assert report.ok is True
    assert report.max_top_score == 0.91
    assert report.trigger_ratio == 0.667
    assert 0.6 <= report.suggested_threshold <= 0.75


def test_high_threshold_warns_when_scores_are_strong_but_not_triggering():
    samples = [
        kws_calibration.KwsScoreSample("openwakeword", "hey", 0.61, 0.8, False, {}),
        kws_calibration.KwsScoreSample("openwakeword", "hey", 0.64, 0.8, False, {}),
    ]

    report = kws_calibration.analyze_kws_scores(samples)

    assert "threshold_may_be_too_high" in report.warnings
    assert report.suggested_threshold < 0.8


def test_low_threshold_or_noise_warns_when_almost_everything_triggers():
    samples = [
        kws_calibration.KwsScoreSample("livekit", "noise", 0.55, 0.5, True, {}),
        kws_calibration.KwsScoreSample("livekit", "noise", 0.58, 0.5, True, {}),
        kws_calibration.KwsScoreSample("livekit", "noise", 0.6, 0.5, True, {}),
    ]

    report = kws_calibration.analyze_kws_scores(samples)

    assert "threshold_may_be_too_low_or_environment_noisy" in report.warnings


def test_format_report_explains_threshold_advice_in_chinese():
    report = kws_calibration.analyze_kws_scores([])

    rendered = kws_calibration.format_report(report)

    assert "WARN: KWS score calibration" in rendered
    assert "没有收到 /agent/kws_score" in rendered
