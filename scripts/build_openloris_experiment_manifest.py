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

    source = json.loads(bag_source.read_text(encoding="utf-8"))
    contract = json.loads(contract_path.read_text(encoding="utf-8"))
    report = json.loads(report_path.read_text(encoding="utf-8"))
    degradation = json.loads(degradation_path.read_text(encoding="utf-8"))
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
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "evidence_scope": {
            "dataset": "real_public_rosbag",
            "ground_truth": "office OptiTrack",
            "fixture": False,
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
            "params": _artifact(params_path),
            "evaluation": report["methodology"],
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
        },
        "artifacts": {
            "bag_contract": _artifact(contract_path),
            "estimate": {**_artifact(estimate_path), "pose_count": pose_count},
            "report": _artifact(report_path),
            "degradation_report": _artifact(degradation_path),
            "launch_log": _artifact(launch_log_path),
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
