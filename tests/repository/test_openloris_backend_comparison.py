import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[2] / "scripts" / "compare_openloris_backends.py"
SPEC = importlib.util.spec_from_file_location("compare_openloris_backends", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(MODULE)


def _report(*, matches=100, coverage=0.95, ate=0.2, rpe=0.05):
    return {
        "passed": True,
        "association": {
            "matched_poses": matches,
            "reference_temporal_coverage_ratio": coverage,
        },
        "ate_xy_m": {"rmse": ate},
        "rpe": {"translation_m": {"rmse": rpe}},
        "loop": {"recall": 0.8},
    }


def test_compare_accepts_comparable_runs_without_preselecting_winner():
    result = MODULE.compare(
        _report(ate=0.18, rpe=0.04), _report(ate=0.20, rpe=0.03)
    )
    assert result["passed"] is True
    assert result["lower_ate_backend"] == "ceres"
    assert result["delta"]["rpe_gtsam_minus_ceres_m"] == -0.01


def test_compare_rejects_different_replay_windows():
    result = MODULE.compare(_report(matches=100), _report(matches=80, coverage=0.70))
    assert result["passed"] is False
    assert result["checks"]["same_sample_window"] is False
    assert result["checks"]["same_temporal_coverage"] is False
