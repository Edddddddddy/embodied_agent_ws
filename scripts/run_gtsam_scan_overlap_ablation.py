#!/usr/bin/env python3
"""Ablate no-GT LaserScan overlap evidence on one immutable GTSAM pose graph."""

from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from typing import Any


# naive_overlap 是有意保留的反例：只看单帧重叠会伤害 Karto 的 chain-matching 约束。
def variant_specs(minimum_overlap: float, minimum_innovation: float):
    return (
        ("cauchy_baseline", False, False, minimum_overlap, minimum_innovation),
        ("naive_overlap", False, True, minimum_overlap, 0.0),
        ("innovation_gate", True, False, minimum_overlap, minimum_innovation),
        ("dual_evidence", True, True, minimum_overlap, minimum_innovation),
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def build_comparison(records: list[dict[str, Any]], graph_sha256: str) -> dict[str, Any]:
    if not records:
        raise ValueError("at least one variant is required")
    node_counts = {item["optimizer"]["nodes"] for item in records}
    edge_counts = {item["optimizer"]["constraints"] for item in records}
    matched_counts = {
        item["evaluation"]["association"]["matched_poses"] for item in records
    }
    checks = {
        "same_graph_sha256": all(item["graph_sha256"] == graph_sha256 for item in records),
        "same_node_count": len(node_counts) == 1,
        "same_constraint_count": len(edge_counts) == 1,
        "same_matched_pose_count": len(matched_counts) == 1,
        "all_trajectory_evaluations_passed": all(
            item["evaluation"]["passed"] for item in records
        ),
        "all_nonlocal_edges_have_overlap_evidence": all(
            item["optimizer"].get("scan_overlap_unavailable_constraints", 0) == 0
            for item in records
            if item["optimizer"].get("scan_overlap_gate", False)
        ),
    }
    variants = []
    for item in records:
        optimizer = item["optimizer"]
        evaluation = item["evaluation"]
        variants.append(
            {
                "name": item["name"],
                "consistency_gate": optimizer["consistency_gate"],
                "scan_overlap_gate": optimizer["scan_overlap_gate"],
                "minimum_scan_overlap_ratio": optimizer["minimum_scan_overlap_ratio"],
                "minimum_translation_residual_m": optimizer[
                    "scan_overlap_gate_min_translation_residual_m"
                ],
                "constraints_used": optimizer["constraints_used"],
                "consistency_rejected_constraints": optimizer[
                    "consistency_rejected_constraints"
                ],
                "scan_overlap_evaluated_constraints": optimizer[
                    "scan_overlap_evaluated_constraints"
                ],
                "scan_overlap_rejected_constraints": optimizer[
                    "scan_overlap_rejected_constraints"
                ],
                "ate_rmse_m": evaluation["ate_xy_m"]["rmse"],
                "ate_p95_m": evaluation["ate_xy_m"]["p95"],
                "rpe_translation_rmse_m": evaluation["rpe"]["translation_m"]["rmse"],
                "final_pose_error_m": evaluation["closure"]["final_pose_error_m"],
            }
        )
    baseline = next(item for item in variants if item["name"] == "cauchy_baseline")
    for item in variants:
        item["ate_change_vs_baseline_pct"] = round(
            100.0
            * (item["ate_rmse_m"] - baseline["ate_rmse_m"])
            / max(baseline["ate_rmse_m"], 1e-12),
            6,
        )
    best = min(variants, key=lambda item: item["ate_rmse_m"])
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "graph_sha256": graph_sha256,
        "graph_nodes": next(iter(node_counts)),
        "graph_constraints": next(iter(edge_counts)),
        "matched_poses_per_variant": next(iter(matched_counts)),
        "best_ate_variant": best["name"],
        "variants": variants,
        "interpretation_boundary": (
            "Overlap is computed from timestamp-associated raw LaserScan and static TF, without "
            "ground truth. This validates Karto-accepted constraints at the frontend/backend "
            "boundary; it neither generates candidates nor proves frontend precision."
        ),
    }


def render_markdown(report: dict[str, Any], graph: Path) -> str:
    try:
        graph_label = graph.resolve().relative_to(Path.cwd().resolve())
    except ValueError:
        graph_label = graph
    rows = "\n".join(
        "| {name} | {consistency_gate} | {scan_overlap_gate} | {minimum_scan_overlap_ratio:.2f} | "
        "{minimum_translation_residual_m:.1f} | {constraints_used} | "
        "{consistency_rejected_constraints} | {scan_overlap_rejected_constraints} | "
        "{ate_rmse_m:.4f} | {ate_p95_m:.4f} | {rpe_translation_rmse_m:.4f} | "
        "{final_pose_error_m:.4f} | {ate_change_vs_baseline_pct:+.2f}% |".format(**item)
        for item in report["variants"]
    )
    evidence = report.get("scan_evidence", {})
    evidence_summary = ""
    if evidence:
        overlap = evidence["overlap_ratio"]
        evidence_summary = (
            f"- 原始扫描证据：{evidence['scored_nonlocal_constraints']} 条非局部边，"
            f"unavailable={evidence['unavailable_nonlocal_constraints']}\n"
            f"- 重叠率分布：min={overlap['minimum']:.4f} / median={overlap['median']:.4f} "
            f"/ P95={overlap['p95']:.4f} / max={overlap['maximum']:.4f}\n"
        )
    return f"""# GTSAM 扫描重叠双证据固定图消融

- 位姿图：`{graph_label}`
- SHA256：`{report['graph_sha256']}`
- 图规模：{report['graph_nodes']} nodes / {report['graph_constraints']} constraints
- 每组匹配位姿：{report['matched_poses_per_variant']}
- ATE 最优组：**{report['best_ate_variant']}**
- 公平性检查：**{'PASS' if report['passed'] else 'FAIL'}**
{evidence_summary}

| variant | innovation gate | overlap gate | min overlap | min innovation m | used edges | innovation reject | overlap reject | ATE RMSE m | ATE P95 m | RPE RMSE m | final m | ATE vs baseline |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
{rows}

> 边界：重叠率来自原始 LaserScan、时间戳关联和静态 TF，不使用真值。该门控位于
> Karto 已接受约束与 GTSAM 后端之间，不生成候选边，也不能证明前端 precision 提升。
> 单帧低重叠不能独立否决 chain-matching 约束，因此保留 naive 组作为反例。
> 当前 0.65 阈值只在 corridor1-1 做过敏感性检查，门控默认关闭；多序列验证前不作为
> 通用参数发布。
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
    parser.add_argument(
        "--evaluator", type=Path, default=Path("scripts/evaluate_slam_trajectory.py")
    )
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--loop-id-separation", type=int, default=20)
    parser.add_argument("--max-iterations", type=int, default=50)
    parser.add_argument("--max-time-diff", type=float, default=0.05)
    parser.add_argument("--rpe-delta", type=float, default=1.0)
    parser.add_argument("--min-match-ratio", type=float, default=0.80)
    parser.add_argument("--max-consistency-translation", type=float, default=2.0)
    parser.add_argument("--max-consistency-yaw-rad", type=float, default=0.7853981633974483)
    parser.add_argument("--minimum-overlap", type=float, default=0.65)
    parser.add_argument("--minimum-overlap-innovation", type=float, default=1.0)
    parser.add_argument("--augmentation-metadata", type=Path)
    parser.add_argument("--published-json", type=Path)
    parser.add_argument("--published-markdown", type=Path)
    args = parser.parse_args()

    if not 0.0 <= args.minimum_overlap <= 1.0:
        parser.error("minimum overlap must be within [0, 1]")
    if args.minimum_overlap_innovation < 0.0:
        parser.error("minimum overlap innovation must be non-negative")

    for path in (args.graph, args.reference, args.optimizer, args.evaluator):
        if not path.is_file():
            parser.error(f"missing required file: {path}")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    graph_digest = sha256(args.graph)
    records: list[dict[str, Any]] = []
    for name, consistency_gate, overlap_gate, minimum_overlap, minimum_innovation in variant_specs(
        args.minimum_overlap, args.minimum_overlap_innovation
    ):
        trajectory = args.output_dir / f"{name}.tum"
        optimizer_result = _run(
            [
                str(args.optimizer),
                str(args.graph),
                str(trajectory),
                "cauchy",
                "1.0",
                "true",
                str(args.loop_id_separation),
                str(args.max_iterations),
                str(consistency_gate).lower(),
                str(args.max_consistency_translation),
                str(args.max_consistency_yaw_rad),
                str(overlap_gate).lower(),
                str(minimum_overlap),
                str(minimum_innovation),
            ]
        )
        optimizer_summary = json.loads(optimizer_result.stdout.strip().splitlines()[-1])
        (args.output_dir / f"{name}_optimizer.json").write_text(
            json.dumps(optimizer_summary, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        evaluation_path = args.output_dir / f"{name}_evaluation.json"
        _run(
            [
                sys.executable,
                str(args.evaluator),
                "--reference",
                str(args.reference),
                "--estimate",
                str(trajectory),
                "--output",
                str(evaluation_path),
                "--max-time-diff",
                str(args.max_time_diff),
                "--rpe-delta",
                str(args.rpe_delta),
                "--min-match-ratio",
                str(args.min_match_ratio),
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
    report["fixed_graph_contract"] = {
        "sha256": graph_digest,
        "nodes": report["graph_nodes"],
        "constraints": report["graph_constraints"],
        "source_is_immutable_across_variants": True,
    }
    report["gate_contract"] = {
        "uses_ground_truth_at_runtime": False,
        "default_enabled": False,
        "minimum_overlap_ratio": args.minimum_overlap,
        "minimum_translation_innovation_m": args.minimum_overlap_innovation,
        "threshold_policy": (
            "0.65 is a conservative corridor1-1 calibration point; keep disabled until "
            "multi-sequence validation"
        ),
    }
    if args.augmentation_metadata:
        report["scan_evidence"] = json.loads(
            args.augmentation_metadata.read_text(encoding="utf-8")
        )
    json_path = args.output_dir / "comparison.json"
    markdown_path = args.output_dir / "comparison.md"
    json_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    markdown_path.write_text(render_markdown(report, args.graph), encoding="utf-8")
    if args.published_json:
        args.published_json.parent.mkdir(parents=True, exist_ok=True)
        args.published_json.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    if args.published_markdown:
        args.published_markdown.parent.mkdir(parents=True, exist_ok=True)
        args.published_markdown.write_text(
            render_markdown(report, args.graph), encoding="utf-8"
        )
    print(json.dumps(report, ensure_ascii=False, indent=2))
    print(f"{'PASS' if report['passed'] else 'FAIL'}: fixed-graph scan-overlap ablation")
    print(f"Reports: {json_path} {markdown_path}")
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
