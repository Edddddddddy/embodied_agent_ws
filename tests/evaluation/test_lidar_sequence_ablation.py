from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "compare_lidar_sequence_ablation",
    ROOT / "tools" / "evaluation" / "compare_lidar_sequence_ablation.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _profile(accepted: int, true: int) -> dict[str, object]:
    return {
        "accepted_pairs": accepted,
        "true_accepted_pairs": true,
        "false_accepted_pairs": accepted - true,
        "precision": true / accepted,
        "conditional_pair_recall": true / 100,
        "eligible_query_recall": true / 50,
        "relative_translation_error_m": {"median": 0.2},
    }


def _write_report(path: Path, sequence: str) -> None:
    path.write_text(
        json.dumps(
            {
                "passed": True,
                "sequence": sequence,
                "ground_truth": {"true_candidate_pairs": 100},
                "profiles": {
                    "cpp_ranked_single": _profile(20, 4),
                    "cpp_temporal": _profile(10, 4),
                    "cpp_sequence_multi_hypothesis": _profile(6, 4),
                },
                "diagnostics": {
                    "sequence_consistency": {
                        "config": {"mode": "multi_hypothesis", "minimum_confirmations": 3}
                    }
                },
            }
        ),
        encoding="utf-8",
    )


def test_multi_hypothesis_comparison_requires_cross_sequence_precision_gain(tmp_path: Path):
    first = tmp_path / "first.json"
    second = tmp_path / "second.json"
    _write_report(first, "first")
    _write_report(second, "second")

    report = MODULE.compare([("first", first), ("second", second)])

    assert report["passed"] is True
    assert report["aggregate"]["single_track"]["micro_precision"] == 0.4
    assert report["aggregate"]["multi_hypothesis"]["micro_precision"] == 2 / 3
    assert report["release_decision"]["direct_graph_edge_insertion_enabled"] is False


def test_multi_hypothesis_comparison_rejects_mismatched_configuration(tmp_path: Path):
    first = tmp_path / "first.json"
    second = tmp_path / "second.json"
    _write_report(first, "first")
    _write_report(second, "second")
    data = json.loads(second.read_text(encoding="utf-8"))
    data["diagnostics"]["sequence_consistency"]["config"][
        "minimum_confirmations"
    ] = 4
    second.write_text(json.dumps(data), encoding="utf-8")

    try:
        MODULE.compare([("first", first), ("second", second)])
    except ValueError as error:
        assert "fixed configuration" in str(error)
    else:
        raise AssertionError("mismatched configuration should fail")
