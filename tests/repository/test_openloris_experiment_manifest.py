from __future__ import annotations

import importlib.util
import json
import subprocess
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location(
    "build_openloris_experiment_manifest",
    ROOT / "scripts" / "build_openloris_experiment_manifest.py",
)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def _write(path: Path, content: str) -> Path:
    path.write_text(content, encoding="utf-8")
    return path


def test_manifest_binds_dataset_commit_config_metrics_and_artifacts(tmp_path):
    subprocess.run(["git", "init", "-q", tmp_path], check=True)
    subprocess.run(["git", "-C", tmp_path, "config", "user.email", "test@example.com"], check=True)
    subprocess.run(["git", "-C", tmp_path, "config", "user.name", "Test"], check=True)
    _write(tmp_path / "tracked", "x")
    subprocess.run(["git", "-C", tmp_path, "add", "tracked"], check=True)
    subprocess.run(["git", "-C", tmp_path, "commit", "-qm", "fixture"], check=True)

    bag = _write(tmp_path / "office1-1.bag", "bag")
    source = _write(
        tmp_path / "source.json",
        json.dumps(
            {
                "dataset": "OpenLORIS-Scene",
                "sequence": "office1-1",
                "dataset_commit": "dataset-commit",
                "archive_sha256": "a" * 64,
                "archive_verification": "full_size_and_sha256",
                "bag_sha256": MODULE.sha256(bag),
                "bag_size_bytes": 3,
                "bag": str(bag.resolve()),
                "derived": {
                    "kind": "lossless_topic_subset",
                    "selected_topics": ["/odom", "/scan", "/tf_static"],
                    "message_counts": {"/odom": 10, "/scan": 10, "/tf_static": 1},
                    "source_bag_sha256": "b" * 64,
                    "tool_sha256": "c" * 64,
                },
            }
        ),
    )
    contract = _write(tmp_path / "contract.json", json.dumps({"passed": True}))
    params = _write(
        tmp_path / "params.yaml", "solver_plugin: solver_plugins::CeresSolver\n"
    )
    estimate = _write(
        tmp_path / "estimate.tum",
        "# tum\n" + "\n".join(f"{i} 0 0 0 0 0 0 1" for i in range(100)),
    )
    report_payload = {
        "passed": True,
        "methodology": {"alignment": "SE2"},
        "association": {"matched_poses": 100},
        "ate_xy_m": {"rmse": 0.1},
        "rpe": {"translation_m": {"rmse": 0.01}},
        "path": {"length_ratio": 1.0},
        "closure": {"final_pose_error_m": 0.1},
        "loop": {"recall": 1.0},
        "worst_segment": {"available": True},
    }
    report = _write(tmp_path / "report.json", json.dumps(report_payload))
    degradation = _write(
        tmp_path / "degradation.json",
        json.dumps(
            {
                "passed": True,
                "motion_classes": {"straight": {"ate_rmse_m": 0.1}},
                "labelled_intervals": [],
            }
        ),
    )
    launch_log = _write(tmp_path / "launch.log", "replay complete")
    annotations = _write(
        tmp_path / "annotations.json",
        json.dumps({"intervals": [{"label": "dynamic_occlusion", "start_s": 1, "end_s": 2}]}),
    )
    result = MODULE.build_manifest(
        workspace=tmp_path,
        sequence="office1-1",
        backend="ceres",
        bag=bag,
        bag_source=source,
        contract_path=contract,
        params_path=params,
        estimate_path=estimate,
        report_path=report,
        degradation_path=degradation,
        launch_log_path=launch_log,
        replay_rate=1.0,
        wall_clock_s=42.0,
        annotations_path=annotations,
    )
    assert result["passed"] is True
    assert result["evidence_scope"]["fixture"] is False
    assert result["artifacts"]["estimate"]["pose_count"] == 100
    assert result["configuration"]["solver_plugin"] == "solver_plugins::CeresSolver"
    assert result["checks"]["source_verification_declared"] is True
    assert result["configuration"]["replay_wall_clock_s"] == 42.0
    assert result["checks"]["derived_topic_subset_bound"] is True
    assert result["evidence_scope"]["lossless_topic_subset"] is True
    assert result["configuration"]["semantic_annotations"]["sha256"] == MODULE.sha256(
        annotations
    )


def test_manifest_rejects_crashed_replay_and_wrong_backend_config(tmp_path, monkeypatch):
    monkeypatch.setattr(MODULE, "_git", lambda _workspace, *_args: "fixture")
    bag = _write(tmp_path / "office1-1.bag", "bag")
    source = _write(
        tmp_path / "source.json",
        json.dumps(
            {
                "dataset": "OpenLORIS-Scene",
                "sequence": "office1-1",
                "archive_sha256": "a" * 64,
                "bag_sha256": "b" * 64,
                "bag_size_bytes": 3,
                "bag": str(bag.resolve()),
            }
        ),
    )
    contract = _write(tmp_path / "contract.json", json.dumps({"passed": True}))
    params = _write(
        tmp_path / "params.yaml", "solver_plugin: solver_plugins::CeresSolver\n"
    )
    estimate = _write(
        tmp_path / "estimate.tum",
        "\n".join(f"{i} 0 0 0 0 0 0 1" for i in range(100)),
    )
    report = _write(
        tmp_path / "report.json",
        json.dumps(
            {
                "passed": True,
                "methodology": {},
                "association": {},
                "ate_xy_m": {},
                "rpe": {},
                "path": {},
                "closure": {},
                "loop": {},
                "worst_segment": {},
            }
        ),
    )
    degradation = _write(
        tmp_path / "degradation.json",
        json.dumps(
            {"passed": True, "motion_classes": {}, "labelled_intervals": []}
        ),
    )
    launch_log = _write(
        tmp_path / "launch.log", "replay complete\nprocess has died"
    )
    result = MODULE.build_manifest(
        workspace=tmp_path,
        sequence="office1-1",
        backend="gtsam",
        bag=bag,
        bag_source=source,
        contract_path=contract,
        params_path=params,
        estimate_path=estimate,
        report_path=report,
        degradation_path=degradation,
        launch_log_path=launch_log,
        replay_rate=1.0,
    )
    assert result["passed"] is False
    assert result["checks"]["backend_config_matches"] is False
    assert result["checks"]["replay_completed_cleanly"] is False
