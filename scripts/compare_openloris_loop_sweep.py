#!/usr/bin/env python3
"""Compare accepted-edge loop evidence from a controlled OpenLORIS sweep."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def _load(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def _graph_topology(path: Path) -> dict[str, int]:
    edges = [
        json.loads(line)
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    gaps = [abs(int(edge["source_id"]) - int(edge["target_id"])) for edge in edges]
    return {
        "accepted_graph_edges": len(edges),
        "sequential_edges": sum(
            edge.get("constraint_kind") == "sequential" for edge in edges
        ),
        "nonlocal_edges": sum(edge.get("constraint_kind") == "loop" for edge in edges),
        "maximum_node_id_separation": max(gaps, default=0),
    }


def compare(config_manifest: dict[str, object], result_root: Path) -> dict[str, object]:
    rows: list[dict[str, object]] = []
    bag_hashes: set[str] = set()
    matched_counts: list[int] = []
    checks: dict[str, bool] = {}
    for profile in config_manifest["profiles"]:  # type: ignore[index]
        profile_id = str(profile["id"])
        directory = result_root / profile_id
        manifest = _load(directory / "gtsam_manifest.json")
        loop = _load(directory / "gtsam_loop_constraints.json")
        report = _load(directory / "gtsam_report.json")
        topology = _graph_topology(directory / "gtsam_constraints.jsonl")
        bag_hashes.add(str(manifest["dataset"]["bag_sha256"]))  # type: ignore[index]
        matched = int(report["association"]["matched_poses"])  # type: ignore[index]
        matched_counts.append(matched)
        accepted = loop["accepted_constraints"]  # type: ignore[index]
        recovery = loop["event_recovery"]  # type: ignore[index]
        checks[f"{profile_id}:manifest"] = bool(manifest.get("passed"))
        checks[f"{profile_id}:loop_report"] = bool(loop.get("passed"))
        checks[f"{profile_id}:config_hash"] = (
            manifest["configuration"]["params"]["sha256"] == profile["sha256"]  # type: ignore[index]
        )
        rows.append(
            {
                "id": profile_id,
                "overrides": profile["overrides"],
                "accepted_loop_edges": int(accepted["associated"]),
                "graph_topology": topology,
                "true_positive": int(accepted["true_positive"]),
                "false_positive": int(accepted["false_positive"]),
                "precision": accepted["precision"],
                "recovered_events": int(recovery["recovered_events"]),
                "event_recall": recovery["event_recall"],
                "ate_rmse_m": float(report["ate_xy_m"]["rmse"]),  # type: ignore[index]
                "rpe_translation_rmse_m": float(
                    report["rpe"]["translation_m"]["rmse"]  # type: ignore[index]
                ),
                "matched_poses": matched,
                "replay_wall_clock_s": manifest["configuration"].get(  # type: ignore[index]
                    "replay_wall_clock_s"
                ),
            }
        )

    checks["same_derived_bag"] = len(bag_hashes) == 1
    checks["comparable_pose_coverage"] = (
        max(matched_counts) - min(matched_counts)
    ) / max(max(matched_counts), 1) <= 0.02
    recalls = [
        float(row["event_recall"])
        for row in rows
        if row["event_recall"] is not None and math.isfinite(float(row["event_recall"]))
    ]
    best_recall = max(recalls) if recalls else None
    best_recall_profiles = [
        row["id"] for row in rows if row["event_recall"] == best_recall
    ]
    lowest_ate = min(rows, key=lambda row: float(row["ate_rmse_m"]))
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "profiles": rows,
        "summary": {
            "best_event_recall": best_recall,
            "best_event_recall_profiles": best_recall_profiles,
            "lowest_ate_profile": lowest_ate["id"],
            "lowest_ate_rmse_m": lowest_ate["ate_rmse_m"],
        },
        "methodology": {
            "controlled": "same lossless topic-subset bag, GTSAM backend and evaluator",
            "changed": "declared slam_toolbox loop-front-end parameters only",
            "precision": "ground-truth-consistent accepted non-local graph edges",
            "event_recall": "ground-truth revisit events recovered by a true accepted edge",
            "boundary": config_manifest.get("claim_boundary"),
            "winner_policy": "descriptive only; no profile is preselected to win",
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--configs", type=Path, required=True)
    parser.add_argument("--result-root", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    report = compare(_load(args.configs), args.result_root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: OpenLORIS loop sweep")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
