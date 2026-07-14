#!/usr/bin/env python3
"""Run GTSAM robust-kernel variants on one immutable pose-graph snapshot."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


VARIANTS = (
    ("gaussian", "none", 0.0, False, False),
    ("huber_all", "huber", 1.345, False, False),
    ("huber_loop", "huber", 1.345, True, False),
    ("cauchy_loop", "cauchy", 1.0, True, False),
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_comparison(variants: list[dict[str, Any]], graph_sha256: str) -> dict[str, Any]:
    """Summarize an ablation without assuming that a robust kernel must win.

    Robust losses cannot repair false closure detection. The experiment passes when every
    optimizer saw the same graph and produced temporally comparable trajectories; ATE decides
    the winner only after measurement.
    """

    if not variants:
        raise ValueError("at least one variant is required")
    node_counts = {item["optimizer"]["nodes"] for item in variants}
    edge_counts = {item["optimizer"]["constraints"] for item in variants}
    matched_counts = {item["evaluation"]["association"]["matched_poses"] for item in variants}
    checks = {
        "same_graph_sha256": all(item["graph_sha256"] == graph_sha256 for item in variants),
        "same_node_count": len(node_counts) == 1,
        "same_constraint_count": len(edge_counts) == 1,
        "same_matched_pose_count": len(matched_counts) == 1,
        "all_trajectory_evaluations_passed": all(
            item["evaluation"]["passed"] for item in variants
        ),
    }
    metric_rows = []
    for item in variants:
        evaluation = item["evaluation"]
        metric_rows.append(
            {
                "name": item["name"],
                "kernel": item["optimizer"]["kernel"],
                "kernel_k": item["optimizer"]["kernel_k"],
                "loop_only": item["optimizer"]["loop_only"],
                "constraints_used": item["optimizer"].get(
                    "constraints_used", item["optimizer"]["constraints"]
                ),
                "robustified_constraints": item["optimizer"]["robustified_constraints"],
                "consistency_gate": item["optimizer"].get("consistency_gate", False),
                "consistency_rejected_constraints": item["optimizer"].get(
                    "consistency_rejected_constraints", 0
                ),
                "ate_rmse_m": evaluation["ate_xy_m"]["rmse"],
                "ate_p95_m": evaluation["ate_xy_m"]["p95"],
                "rpe_translation_rmse_m": evaluation["rpe"]["translation_m"]["rmse"],
                "final_pose_error_m": evaluation["closure"]["final_pose_error_m"],
            }
        )
    best = min(metric_rows, key=lambda item: item["ate_rmse_m"])
    baseline = next(item for item in metric_rows if item["name"] == "gaussian")
    for item in metric_rows:
        item["ate_change_vs_gaussian_pct"] = round(
            100.0 * (item["ate_rmse_m"] - baseline["ate_rmse_m"])
            / max(baseline["ate_rmse_m"], 1e-12),
            6,
        )
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "graph_sha256": graph_sha256,
        "graph_nodes": next(iter(node_counts)),
        "graph_constraints": next(iter(edge_counts)),
        "matched_poses_per_variant": next(iter(matched_counts)),
        "best_ate_variant": best["name"],
        "variants": metric_rows,
        "interpretation_boundary": (
            "This fixed-graph ablation measures backend sensitivity to already accepted "
            "constraints. It does not improve or prove loop-closure front-end precision."
        ),
    }


def render_markdown(report: dict[str, Any], graph: Path) -> str:
    rows = "\n".join(
        "| {name} | {kernel} | {loop_only} | {consistency_gate} | {constraints_used} | "
        "{robustified_constraints} | {consistency_rejected_constraints} | "
        "{ate_rmse_m:.4f} | {ate_p95_m:.4f} | {rpe_translation_rmse_m:.4f} | "
        "{final_pose_error_m:.4f} | {ate_change_vs_gaussian_pct:+.2f}% |".format(**item)
        for item in report["variants"]
    )
    return f"""# GTSAM 鲁棒核固定图消融

- 位姿图：`{graph}`
- SHA256：`{report['graph_sha256']}`
- 图规模：{report['graph_nodes']} nodes / {report['graph_constraints']} constraints
- 每组匹配位姿：{report['matched_poses_per_variant']}
- ATE 最优组：**{report['best_ate_variant']}**
- 公平性检查：**{'PASS' if report['passed'] else 'FAIL'}**

| variant | kernel | loop only | gate | used edges | robust edges | rejected | ATE RMSE m | ATE P95 m | RPE RMSE m | final m | ATE vs Gaussian |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
{rows}

> 结论边界：所有变体复用同一份前端已接受的节点和约束。鲁棒核衡量后端对离群边的
> 敏感性；一致性门控只使用入图前估计，不使用真值，但可能在累计漂移严重时误拒绝真回环。
> 该实验不能证明回环前端 precision 得到改善。
"""


def _run(command: list[str]) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    if result.returncode != 0:
        raise RuntimeError(
            f"command failed ({result.returncode}): {' '.join(command)}\n"
            f"stdout:\n{result.stdout[-4000:]}\nstderr:\n{result.stderr[-4000:]}"
        )
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--graph", type=Path, required=True)
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--optimizer", type=Path, required=True)
    parser.add_argument("--evaluator", type=Path, default=Path("scripts/evaluate_slam_trajectory.py"))
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--loop-id-separation", type=int, default=20)
    parser.add_argument("--max-iterations", type=int, default=50)
    parser.add_argument("--max-time-diff", type=float, default=0.05)
    parser.add_argument("--rpe-delta", type=float, default=1.0)
    parser.add_argument("--min-match-ratio", type=float, default=0.80)
    parser.add_argument("--loop-radius", type=float, default=1.0)
    parser.add_argument("--loop-yaw-tolerance-deg", type=float, default=180.0)
    parser.add_argument("--loop-min-separation", type=float, default=60.0)
    parser.add_argument("--include-consistency-gate", action="store_true")
    parser.add_argument("--max-consistency-translation", type=float, default=2.0)
    parser.add_argument("--max-consistency-yaw-rad", type=float, default=0.7853981633974483)
    args = parser.parse_args()

    for path in (args.graph, args.reference, args.optimizer, args.evaluator):
        if not path.is_file():
            parser.error(f"missing required file: {path}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    graph_digest = sha256(args.graph)
    records: list[dict[str, Any]] = []
    variants = list(VARIANTS)
    if args.include_consistency_gate:
        variants.append(("cauchy_loop_gated", "cauchy", 1.0, True, True))
    for name, kernel, kernel_k, loop_only, consistency_gate in variants:
        trajectory = args.output_dir / f"{name}.tum"
        optimizer_result = _run(
            [
                str(args.optimizer),
                str(args.graph),
                str(trajectory),
                kernel,
                str(kernel_k),
                str(loop_only).lower(),
                str(args.loop_id_separation),
                str(args.max_iterations),
                str(consistency_gate).lower(),
                str(args.max_consistency_translation),
                str(args.max_consistency_yaw_rad),
            ]
        )
        optimizer_summary = json.loads(optimizer_result.stdout.strip().splitlines()[-1])
        optimizer_report_path = args.output_dir / f"{name}_optimizer.json"
        optimizer_report_path.write_text(
            json.dumps(optimizer_summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        evaluation_path = args.output_dir / f"{name}_evaluation.json"
        _run(
            [
                sys.executable,
                str(args.evaluator),
                "--reference", str(args.reference),
                "--estimate", str(trajectory),
                "--output", str(evaluation_path),
                "--max-time-diff", str(args.max_time_diff),
                "--rpe-delta", str(args.rpe_delta),
                "--min-match-ratio", str(args.min_match_ratio),
                "--loop-radius", str(args.loop_radius),
                "--loop-yaw-tolerance-deg", str(args.loop_yaw_tolerance_deg),
                "--loop-min-separation", str(args.loop_min_separation),
            ]
        )
        records.append(
            {
                "name": name,
                "graph_sha256": graph_digest,
                "optimizer": optimizer_summary,
                "evaluation": json.loads(evaluation_path.read_text(encoding="utf-8")),
            }
        )

    report = build_comparison(records, graph_digest)
    json_path = args.output_dir / "comparison.json"
    markdown_path = args.output_dir / "comparison.md"
    json_path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    markdown_path.write_text(render_markdown(report, args.graph), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: fixed-graph robust-kernel ablation")
    print(f"Reports: {json_path} {markdown_path}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
