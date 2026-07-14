#!/usr/bin/env python3
"""Build a reproducible manifest for one real OpenLORIS SLAM run."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path


def sha256(path: Path) -> str:
    """Return a streaming SHA256 for an evidence artifact."""

    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _git(workspace: Path, *args: str) -> str:
    completed = subprocess.run(
        ["git", *args], cwd=workspace, check=True, capture_output=True, text=True
    )
    return completed.stdout.strip()


def _artifact(path: Path) -> dict[str, object]:
    return {
        "path": str(path.resolve()),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def build_manifest(
    *,
    workspace: Path,
    sequence: str,
    backend: str,
    bag: Path,
    bag_source: Path,
    contract_path: Path,
    params_path: Path,
    estimate_path: Path,
    report_path: Path,
    degradation_path: Path,
    launch_log_path: Path,
    replay_rate: float,
    wall_clock_s: float | None = None,
    constraint_log_path: Path | None = None,
    loop_report_path: Path | None = None,
    frontend_log_path: Path | None = None,
    frontend_report_path: Path | None = None,
    annotations_path: Path | None = None,
) -> dict[str, object]:
    """Validate evidence relationships and freeze configuration plus provenance."""

    if backend not in {"ceres", "gtsam"}:
        raise ValueError(f"unsupported backend: {backend}")
    for path in (
        bag,
        bag_source,
        contract_path,
        params_path,
        estimate_path,
        report_path,
        degradation_path,
        launch_log_path,
    ):
        if not path.is_file() or path.stat().st_size == 0:
            raise FileNotFoundError(path)
    if (constraint_log_path is None) != (loop_report_path is None):
        raise ValueError("constraint log and loop report must be provided together")
    if constraint_log_path is not None and loop_report_path is not None:
        for path in (constraint_log_path, loop_report_path):
            if not path.is_file() or path.stat().st_size == 0:
                raise FileNotFoundError(path)
    if (frontend_log_path is None) != (frontend_report_path is None):
        raise ValueError("frontend log and frontend report must be provided together")
    if frontend_log_path is not None and frontend_report_path is not None:
        for path in (frontend_log_path, frontend_report_path):
            if not path.is_file() or path.stat().st_size == 0:
                raise FileNotFoundError(path)
    if annotations_path is not None and (
        not annotations_path.is_file() or annotations_path.stat().st_size == 0
    ):
        raise FileNotFoundError(annotations_path)

    source = json.loads(bag_source.read_text(encoding="utf-8"))
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    report = json.loads(report_path.read_text(encoding="utf-8"))
    degradation = json.loads(degradation_path.read_text(encoding="utf-8"))
    loop_report = (
        json.loads(loop_report_path.read_text(encoding="utf-8"))
        if loop_report_path is not None
        else None
    )
    frontend_report = (
        json.loads(frontend_report_path.read_text(encoding="utf-8"))
        if frontend_report_path is not None
        else None
    )
    launch_log = launch_log_path.read_text(encoding="utf-8", errors="replace")
    params_text = params_path.read_text(encoding="utf-8")
    pose_count = sum(
        1
        for line in estimate_path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    )
    expected_solver = (
        "solver_plugins::CeresSolver"
        if backend == "ceres"
        else "embodied_slam::GtsamScanSolver"
    )
    source_bag = Path(str(source.get("bag", "")))
    source_verification = source.get("archive_verification")
    derived = source.get("derived")
    source_bag_sha256 = str(source.get("bag_sha256", ""))
    checks = {
        "bag_contract_passed": bool(contract.get("passed")),
        "trajectory_report_passed": bool(report.get("passed")),
        "degradation_report_passed": bool(degradation.get("passed")),
        "minimum_pose_count": pose_count >= 100,
        "backend_config_matches": expected_solver in params_text,
        "replay_completed_cleanly": (
            "replay complete" in launch_log
            and "Traceback" not in launch_log
            and "process has died" not in launch_log
        ),
        # manifest 只信任 setup 脚本写出的来源记录，不能把任意同名 .bag 当成公开数据证据。
        "bag_provenance_matches": (
            source.get("dataset") == "OpenLORIS-Scene"
            and source.get("sequence") == sequence
            and source_bag.resolve() == bag.resolve()
            and len(source_bag_sha256) == 64
            and sha256(bag) == source_bag_sha256
        ),
        "source_verification_declared": source_verification
        in {"full_size_and_sha256", "pinned_https_range_not_full_hash"},
    }
    if loop_report is not None:
        checks["loop_constraint_report_passed"] = bool(loop_report.get("passed"))
    if frontend_report is not None:
        checks["loop_frontend_report_passed"] = bool(frontend_report.get("passed"))
    if derived is not None:
        if not isinstance(derived, dict):
            raise ValueError("derived provenance must be an object")
        counts = derived.get("message_counts", {})
        checks["derived_topic_subset_bound"] = (
            derived.get("kind") == "lossless_topic_subset"
            and derived.get("selected_topics") == ["/odom", "/scan", "/tf_static"]
            and isinstance(counts, dict)
            and all(
                int(counts.get(topic, 0)) > 0
                for topic in ("/odom", "/scan", "/tf_static")
            )
            and len(str(derived.get("source_bag_sha256", ""))) == 64
            and len(str(derived.get("tool_sha256", ""))) == 64
        )
    return {
        "schema_version": 3,
        "passed": all(checks.values()),
        "checks": checks,
        "evidence_scope": {
            "dataset": "real_public_rosbag",
            "ground_truth": "office OptiTrack",
            "fixture": False,
            "lossless_topic_subset": derived is not None,
            "claim_boundary": (
                "metrics apply only to this sequence, bag hash, commit and parameter file"
            ),
        },
        "dataset": {
            "name": source["dataset"],
            "sequence": sequence,
            "archive_sha256": source["archive_sha256"],
            "archive_verification": source_verification,
            "bag_sha256": source["bag_sha256"],
            "bag_size_bytes": source["bag_size_bytes"],
            "dataset_commit": source.get("dataset_commit"),
            "derived_from": derived,
        },
        "software": {
            "git_commit": _git(workspace, "rev-parse", "HEAD"),
            "git_branch": _git(workspace, "branch", "--show-current"),
            "git_dirty": bool(_git(workspace, "status", "--porcelain")),
        },
        "configuration": {
            "backend": backend,
            "solver_plugin": expected_solver,
            "replay_rate": replay_rate,
            "replay_wall_clock_s": wall_clock_s,
            "params": _artifact(params_path),
            "evaluation": report["methodology"],
            "accepted_loop_constraint_evaluation": loop_report is not None,
            "loop_frontend_instrumentation": frontend_report is not None,
            "semantic_annotations": (
                _artifact(annotations_path) if annotations_path is not None else None
            ),
        },
        "metrics": {
            "association": report["association"],
            "ate_xy_m": report["ate_xy_m"],
            "rpe": report["rpe"],
            "path": report["path"],
            "closure": report["closure"],
            "loop": report["loop"],
            "worst_segment": report["worst_segment"],
            "motion_classes": degradation["motion_classes"],
            "labelled_intervals": degradation["labelled_intervals"],
            **(
                {
                    "accepted_loop_constraints": loop_report["accepted_constraints"],
                    "loop_event_recovery": loop_report["event_recovery"],
                }
                if loop_report is not None
                else {}
            ),
            **(
                {"loop_frontend": frontend_report}
                if frontend_report is not None
                else {}
            ),
        },
        "artifacts": {
            "bag_contract": _artifact(contract_path),
            "estimate": {**_artifact(estimate_path), "pose_count": pose_count},
            "report": _artifact(report_path),
            "degradation_report": _artifact(degradation_path),
            "launch_log": _artifact(launch_log_path),
            **(
                {
                    "accepted_constraint_log": _artifact(constraint_log_path),
                    "loop_constraint_report": _artifact(loop_report_path),
                }
                if constraint_log_path is not None and loop_report_path is not None
                else {}
            ),
            **(
                {
                    "loop_frontend_log": _artifact(frontend_log_path),
                    "loop_frontend_report": _artifact(frontend_report_path),
                }
                if frontend_log_path is not None and frontend_report_path is not None
                else {}
            ),
        },
    }


def main() -> int:
    """CLI entry point."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", type=Path, default=Path.cwd())
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--backend", choices=("ceres", "gtsam"), required=True)
    parser.add_argument("--bag", type=Path, required=True)
    parser.add_argument("--bag-source", type=Path, required=True)
    parser.add_argument("--contract", type=Path, required=True)
    parser.add_argument("--params", type=Path, required=True)
    parser.add_argument("--estimate", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--degradation", type=Path, required=True)
    parser.add_argument("--launch-log", type=Path, required=True)
    parser.add_argument("--replay-rate", type=float, required=True)
    parser.add_argument("--wall-clock-s", type=float)
    parser.add_argument("--constraint-log", type=Path)
    parser.add_argument("--loop-report", type=Path)
    parser.add_argument("--frontend-log", type=Path)
    parser.add_argument("--frontend-report", type=Path)
    parser.add_argument("--annotations", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    manifest = build_manifest(
        workspace=args.workspace,
        sequence=args.sequence,
        backend=args.backend,
        bag=args.bag,
        bag_source=args.bag_source,
        contract_path=args.contract,
        params_path=args.params,
        estimate_path=args.estimate,
        report_path=args.report,
        degradation_path=args.degradation,
        launch_log_path=args.launch_log,
        replay_rate=args.replay_rate,
        wall_clock_s=args.wall_clock_s,
        constraint_log_path=args.constraint_log,
        loop_report_path=args.loop_report,
        frontend_log_path=args.frontend_log,
        frontend_report_path=args.frontend_report,
        annotations_path=args.annotations,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    print(f"{'PASS' if manifest['passed'] else 'FAIL'}: OpenLORIS experiment manifest")
    return 0 if manifest["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
