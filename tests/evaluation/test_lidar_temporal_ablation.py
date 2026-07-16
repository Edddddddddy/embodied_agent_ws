import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]


def _load_module():
    path = ROOT / "tools" / "evaluation" / "compare_lidar_temporal_ablation.py"
    spec = importlib.util.spec_from_file_location("compare_lidar_temporal_ablation", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _report(sequence: str, config: dict[str, object]) -> dict[str, object]:
    return {
        "passed": True,
        "sequence": sequence,
        "ground_truth": {"true_candidate_pairs": 20},
        "profiles": {
            "cpp_ranked_single": {
                "accepted_pairs": 10,
                "true_accepted_pairs": 2,
                "false_accepted_pairs": 8,
                "conditional_pair_recall": 0.1,
            },
            "cpp_temporal": {
                "accepted_pairs": 3,
                "true_accepted_pairs": 1,
                "false_accepted_pairs": 2,
                "conditional_pair_recall": 0.05,
            },
        },
        "diagnostics": {"temporal_consistency": {"config": config}},
    }


def _write(path: Path, value: dict[str, object]) -> Path:
    path.write_text(json.dumps(value), encoding="utf-8")
    return path


def test_multisequence_temporal_precision_gate(tmp_path: Path) -> None:
    module = _load_module()
    config = {"minimum_confirmations": 4}
    first = _write(tmp_path / "first.json", _report("first", config))
    second = _write(tmp_path / "second.json", _report("second", config))
    result = module.compare([("first", first), ("second", second)])
    assert result["passed"] is True
    assert result["aggregate"]["ranked_single"]["micro_precision"] == pytest.approx(0.2)
    assert result["aggregate"]["temporal"]["micro_precision"] == pytest.approx(1 / 3)
    assert result["aggregate"]["pair_recall_delta"] < 0.0


def test_temporal_ablation_rejects_per_sequence_parameter_tuning(tmp_path: Path) -> None:
    module = _load_module()
    first = _write(
        tmp_path / "first.json", _report("first", {"minimum_confirmations": 3})
    )
    second = _write(
        tmp_path / "second.json", _report("second", {"minimum_confirmations": 4})
    )
    with pytest.raises(ValueError, match="fixed configuration"):
        module.compare([("first", first), ("second", second)])
