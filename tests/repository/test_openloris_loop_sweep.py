from __future__ import annotations

import importlib.util
import json
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[2]


def _load(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec and spec.loader
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CONFIGS = _load(
    "build_openloris_loop_sweep_configs",
    ROOT / "scripts" / "build_openloris_loop_sweep_configs.py",
)
COMPARE = _load(
    "compare_openloris_loop_sweep",
    ROOT / "scripts" / "compare_openloris_loop_sweep.py",
)
COMPACT = _load(
    "compact_openloris_rosbag",
    ROOT / "scripts" / "compact_openloris_rosbag.py",
)


def test_config_matrix_changes_only_declared_frontend_keys(tmp_path):
    base = ROOT / "src" / "embodied_slam" / "config" / "openloris_mapping_gtsam.yaml"
    matrix = ROOT / "src" / "embodied_slam" / "config" / "openloris_loop_sweep.json"
    manifest = CONFIGS.build_configs(base, matrix, tmp_path)
    assert [item["id"] for item in manifest["profiles"]] == [
        "baseline",
        "short_chain",
        "lower_response",
        "wider_search",
        "permissive_combined",
        "diagnostic_extreme",
    ]
    original = yaml.safe_load(base.read_text(encoding="utf-8"))
    baseline = yaml.safe_load((tmp_path / "baseline.yaml").read_text(encoding="utf-8"))
    assert baseline == original
    combined = yaml.safe_load(
        (tmp_path / "permissive_combined.yaml").read_text(encoding="utf-8")
    )
    parameters = combined["slam_toolbox"]["ros__parameters"]
    assert parameters["loop_match_minimum_chain_size"] == 5
    assert parameters["loop_match_minimum_response_fine"] == 0.3
    assert parameters["solver_plugin"] == "embodied_slam::GtsamScanSolver"
    diagnostic = yaml.safe_load(
        (tmp_path / "diagnostic_extreme.yaml").read_text(encoding="utf-8")
    )["slam_toolbox"]["ros__parameters"]
    assert diagnostic["loop_match_minimum_chain_size"] == 1
    assert diagnostic["loop_match_maximum_variance_coarse"] == 100.0


def test_derived_source_preserves_public_bag_hash_and_binds_subset(tmp_path, monkeypatch):
    raw = tmp_path / "raw.bag"
    derived = tmp_path / "derived.bag"
    raw.write_bytes(b"public-raw")
    derived.write_bytes(b"slam-only")
    monkeypatch.setattr(COMPACT, "_git_commit", lambda _workspace: "abc123")
    monkeypatch.setattr(COMPACT, "_git_dirty", lambda _workspace: True)
    source = {
        "dataset": "OpenLORIS-Scene",
        "sequence": "office1-7",
        "archive_sha256": "a" * 64,
        "archive_verification": "full_size_and_sha256",
        "bag": str(raw.resolve()),
        "bag_sha256": COMPACT.sha256(raw),
        "bag_size_bytes": raw.stat().st_size,
    }
    COMPACT.validate_source(source, raw)
    result = COMPACT.build_derived_source(
        source=source,
        raw_bag=raw,
        output_bag=derived,
        counts={"/odom": 2, "/scan": 2, "/tf_static": 1},
        first_timestamp_ns=1_000_000_000,
        last_timestamp_ns=2_000_000_000,
        workspace=tmp_path,
    )
    assert result["bag_sha256"] == COMPACT.sha256(derived)
    assert result["derived"]["source_bag_sha256"] == COMPACT.sha256(raw)
    assert result["derived"]["duration_s"] == 1.0
    assert result["derived"]["tool_git_commit"] == "abc123"
    assert result["derived"]["tool_git_dirty"] is True
    assert len(result["derived"]["tool_sha256"]) == 64


def _write_result(root: Path, profile: dict, *, loops: int, recall: float, ate: float):
    directory = root / profile["id"]
    directory.mkdir(parents=True)
    (directory / "gtsam_manifest.json").write_text(
        json.dumps(
            {
                "passed": True,
                "dataset": {"bag_sha256": "b" * 64},
                "configuration": {
                    "params": {"sha256": profile["sha256"]},
                    "replay_wall_clock_s": 40,
                },
            }
        ),
        encoding="utf-8",
    )
    (directory / "gtsam_loop_constraints.json").write_text(
        json.dumps(
            {
                "passed": True,
                "accepted_constraints": {
                    "associated": loops,
                    "true_positive": loops,
                    "false_positive": 0,
                    "precision": 1.0 if loops else None,
                },
                "event_recovery": {
                    "recovered_events": int(2 * recall),
                    "event_recall": recall,
                },
            }
        ),
        encoding="utf-8",
    )
    (directory / "gtsam_report.json").write_text(
        json.dumps(
            {
                "association": {"matched_poses": 449},
                "ate_xy_m": {"rmse": ate},
                "rpe": {"translation_m": {"rmse": 0.02}},
            }
        ),
        encoding="utf-8",
    )
    (directory / "gtsam_frontend_report.json").write_text(
        json.dumps(
            {
                "passed": True,
                "failure_boundary": "candidate_generation" if loops == 0 else "accepted_loop",
                "candidate_primary_reasons": {
                    "all_geometric_neighbors_near_linked": 1
                },
                "topology_maxima": {"maximum_eligible_chain_size": loops},
                "events": {
                    "processed_topology_scans": 4,
                    "candidate_scans": loops,
                    "coarse_checks": loops,
                    "fine_checks": loops,
                    "closure_ends": loops,
                },
            }
        ),
        encoding="utf-8",
    )
    edges = [
        {
            "source_id": index,
            "target_id": index + 1,
            "constraint_kind": "sequential",
        }
        for index in range(3)
    ]
    (directory / "gtsam_constraints.jsonl").write_text(
        "\n".join(json.dumps(edge) for edge in edges) + "\n", encoding="utf-8"
    )


def test_comparison_requires_same_bag_and_reports_zero_loop_as_evidence(tmp_path):
    config = {
        "claim_boundary": "accepted edges only",
        "profiles": [
            {"id": "baseline", "overrides": {}, "sha256": "1" * 64},
            {"id": "permissive", "overrides": {"x": 1}, "sha256": "2" * 64},
        ],
    }
    _write_result(tmp_path, config["profiles"][0], loops=0, recall=0.0, ate=0.10)
    _write_result(tmp_path, config["profiles"][1], loops=1, recall=0.5, ate=0.11)
    report = COMPARE.compare(config, tmp_path)
    assert report["passed"] is True
    assert report["profiles"][0]["precision"] is None
    assert report["profiles"][0]["graph_topology"]["maximum_node_id_separation"] == 1
    assert report["summary"]["best_event_recall_profiles"] == ["permissive"]
    assert report["summary"]["failure_boundaries"] == {
        "accepted_loop": 1,
        "candidate_generation": 1,
    }
    assert report["profiles"][0]["loop_frontend"]["failure_boundary"] == "candidate_generation"
    assert report["methodology"]["boundary"] == "accepted edges only"
