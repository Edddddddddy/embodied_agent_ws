from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]


def _load(name: str):
    path = ROOT / "scripts" / name
    spec = importlib.util.spec_from_file_location(path.stem, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PREPARE = _load("prepare_lidar_shadow_pairs.py")
EVALUATE = _load("evaluate_lidar_shadow_matches.py")


def test_pair_contract_carries_runtime_odometry_relative_pose(tmp_path: Path):
    candidates = tmp_path / "candidates.jsonl"
    candidates.write_text(
        '{"query_id":2,"query_stamp_s":2.0,"candidates":['
        '{"rank":1,"scan_id":1,"stamp_s":1.0,"yaw_offset_rad":0.1,'
        '"similarity":0.8,"ring_key_distance":0.2}]}\n',
        encoding="utf-8",
    )
    odometry = tmp_path / "odometry.txt"
    odometry.write_text(
        "# odometry\nO 1 1.0 1.0 0.0 0.0\nO 2 2.0 2.0 0.0 0.0\n",
        encoding="utf-8",
    )
    output = tmp_path / "pairs.txt"

    summary = PREPARE.prepare(candidates, output, 10, odometry)

    assert summary["odometry_prior"] is True
    # candidate pose expressed in query frame: x=1 - 2 = -1 m.
    assert output.read_text(encoding="utf-8").splitlines()[1].endswith(
        "-1 0 0"
    )


def test_pair_contract_rejects_candidate_corpus_hash_mismatch(tmp_path: Path):
    metadata = tmp_path / "corpus.json"
    evidence = tmp_path / "candidate.json"
    metadata.write_text('{"corpus":{"sha256":"actual"}}', encoding="utf-8")
    evidence.write_text(
        '{"source":{"corpus_sha256":"different"}}', encoding="utf-8"
    )

    try:
        PREPARE.prepare(
            tmp_path / "unused.jsonl",
            tmp_path / "unused.txt",
            10,
            corpus_metadata_path=metadata,
            candidate_evidence_path=evidence,
        )
    except ValueError as error:
        assert "SHA256" in str(error)
    else:
        raise AssertionError("mixed candidate/corpus sources should fail")


def test_shadow_evaluation_separates_data_contract_from_edge_release():
    reference = [
        EVALUATE.EVALUATOR.Pose2(float(index), float(index % 60), 0.0, 0.0)
        for index in range(121)
    ]
    rows = [
        {
            "query_id": 60,
            "candidate_id": 0,
            "query_stamp_s": 60.0,
            "candidate_stamp_s": 0.0,
            "available": True,
            "converged": True,
            "accepted": True,
            "yaw_ambiguous": False,
            "odometry_prior_used": True,
            "odometry_prior_consistent": True,
            "target_to_source": {"x_m": 0.0, "y_m": 0.0, "yaw_rad": 0.0},
            "inlier_ratio": 0.8,
            "bidirectional_overlap_ratio": 0.8,
            "rmse_m": 0.05,
            "observability_ratio": 0.1,
            "odometry_prior_yaw_error_rad": 0.0,
            "rejection_reason": "accepted",
        }
    ]

    report = EVALUATE.evaluate(
        rows,
        reference,
        sequence="synthetic",
        maximum_time_diff_s=0.01,
        revisit_radius_m=1.0,
        minimum_temporal_separation_s=60.0,
        event_gap_s=2.0,
    )

    assert report["passed"] is True
    assert report["profiles"]["cpp_default"]["precision"] == 1.0
    assert "never inserted" in report["methodology"]["boundary"]


def test_invalid_pair_time_coverage_threshold_is_rejected():
    try:
        EVALUATE.evaluate(
            [],
            [],
            sequence="invalid",
            maximum_time_diff_s=0.01,
            revisit_radius_m=1.0,
            minimum_temporal_separation_s=60.0,
            event_gap_s=2.0,
            minimum_pair_time_coverage=0.0,
        )
    except ValueError as error:
        assert "coverage" in str(error)
    else:
        raise AssertionError("invalid coverage threshold should fail")


def test_match_loader_normalizes_legacy_mode_and_rejects_mixed_modes(tmp_path: Path):
    base = {
        "query_id": 2,
        "candidate_id": 1,
        "query_stamp_s": 2.0,
        "candidate_stamp_s": 1.0,
        "inlier_ratio": 0.8,
        "bidirectional_overlap_ratio": 0.8,
        "rmse_m": 0.05,
        "observability_ratio": 0.1,
    }
    legacy = tmp_path / "legacy.jsonl"
    import json

    legacy.write_text(json.dumps(base) + "\n", encoding="utf-8")
    assert EVALUATE.load_rows(legacy)[0]["matching_mode"] == "scan_to_scan"

    mixed = tmp_path / "mixed.jsonl"
    second = dict(base, query_id=3, matching_mode="scan_to_submap")
    mixed.write_text(json.dumps(base) + "\n" + json.dumps(second) + "\n", encoding="utf-8")
    try:
        EVALUATE.load_rows(mixed)
    except ValueError as error:
        assert "matching_mode" in str(error)
    else:
        raise AssertionError("mixed geometry modes must not share one evidence report")
