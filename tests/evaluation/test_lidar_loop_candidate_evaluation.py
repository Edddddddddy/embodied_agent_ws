from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "evaluate_lidar_loop_candidates",
    ROOT / "tools" / "evaluation" / "evaluate_lidar_loop_candidates.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_ring_key_retrieval_beats_similarity_rerank_on_repeated_route():
    reference = [MODULE.EVALUATOR.Pose2(float(index), float(index % 60), 0.0, 0.0) for index in range(130)]
    rows = []
    for index in range(130):
        candidates = []
        if index >= 60:
            candidates.append(
                {
                    "rank": 1,
                    "scan_id": index - 60,
                    "stamp_s": float(index - 60),
                    "similarity": 0.80,
                    "ring_key_distance": 0.10,
                }
            )
        if index >= 61:
            candidates.append(
                {
                    "rank": 2,
                    "scan_id": index - 61,
                    "stamp_s": float(index - 61),
                    "similarity": 0.95,
                    "ring_key_distance": 0.20,
                }
            )
        rows.append(
            {
                "query_id": index,
                "query_stamp_s": float(index),
                "descriptor_valid": True,
                "candidates": candidates,
            }
        )
    metadata = {
        "passed": True,
        "corpus": {"nodes": 130, "sha256": "a" * 64},
        "source_graph": {"sha256": "b" * 64},
        "source_bag": {"sha256": "c" * 64},
    }

    report = MODULE.evaluate(
        rows,
        reference,
        sequence="synthetic",
        corpus_metadata=metadata,
        top_ks=[1, 2],
        minimum_temporal_separation_s=60.0,
        revisit_radius_m=0.05,
        maximum_time_diff_s=0.01,
        event_gap_s=2.0,
    )

    ring = report["retrieval"]["ring_key_retrieval"]["top_k_metrics"]["at_1"]
    rerank = report["retrieval"]["similarity_rerank_ablation"]["top_k_metrics"]["at_1"]
    assert report["passed"] is True
    assert ring["query_recall"] == 1.0
    assert rerank["query_recall"] < ring["query_recall"]
    assert report["methodology"]["boundary"].startswith("This evaluates candidate")


def test_load_candidate_rows_rejects_duplicate_query_ids(tmp_path: Path):
    path = tmp_path / "candidates.jsonl"
    path.write_text(
        '{"query_id":1,"query_stamp_s":1,"candidates":[]}\n'
        '{"query_id":1,"query_stamp_s":2,"candidates":[]}\n',
        encoding="utf-8",
    )

    try:
        MODULE.load_candidate_rows(path)
    except ValueError as error:
        assert "duplicate query id" in str(error)
    else:
        raise AssertionError("duplicate query id should fail")
