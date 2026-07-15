#!/usr/bin/env python3
"""Aggregate one-candidate and C++ temporal loop-gate evidence across sequences."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _assignment(value: str) -> tuple[str, Path]:
    sequence, separator, raw_path = value.partition("=")
    if not separator or not sequence or not raw_path:
        raise argparse.ArgumentTypeError("report must use SEQUENCE=PATH")
    return sequence, Path(raw_path)


def _precision(profile: dict[str, object]) -> float:
    accepted = int(profile["accepted_pairs"])
    return int(profile["true_accepted_pairs"]) / accepted if accepted else 0.0


def compare(reports: list[tuple[str, Path]]) -> dict[str, object]:
    if len(reports) < 2:
        raise ValueError("temporal ablation requires at least two independent sequences")
    per_sequence = []
    baseline_accepted = baseline_true = temporal_accepted = temporal_true = 0
    true_pairs = 0
    configs: list[dict[str, object]] = []
    sources = []
    for sequence, path in reports:
        report = json.loads(path.read_text(encoding="utf-8"))
        if report.get("sequence") != sequence or not report.get("passed"):
            raise ValueError(f"invalid or failed sequence report: {sequence}")
        profiles = report["profiles"]
        baseline = profiles["cpp_ranked_single"]
        temporal = profiles["cpp_temporal"]
        diagnostics = report["diagnostics"]["temporal_consistency"]
        configs.append(diagnostics["config"])
        baseline_accepted += int(baseline["accepted_pairs"])
        baseline_true += int(baseline["true_accepted_pairs"])
        temporal_accepted += int(temporal["accepted_pairs"])
        temporal_true += int(temporal["true_accepted_pairs"])
        true_pairs += int(report["ground_truth"]["true_candidate_pairs"])
        baseline_precision = _precision(baseline)
        temporal_precision = _precision(temporal)
        per_sequence.append(
            {
                "sequence": sequence,
                "ranked_single": baseline,
                "temporal": temporal,
                "precision_delta": temporal_precision - baseline_precision,
                "pair_recall_delta": float(temporal["conditional_pair_recall"])
                - float(baseline["conditional_pair_recall"]),
                "precision_non_decreasing": temporal_precision >= baseline_precision,
            }
        )
        # 报告进入仓库后不能绑定某台机器的绝对工作区；哈希负责内容溯源。
        sources.append(
            {"sequence": sequence, "report": path.name, "sha256": _sha256(path)}
        )
    if any(config != configs[0] for config in configs[1:]):
        raise ValueError("all temporal reports must use one fixed configuration")

    baseline_precision = baseline_true / baseline_accepted if baseline_accepted else 0.0
    temporal_precision = temporal_true / temporal_accepted if temporal_accepted else 0.0
    checks = {
        "at_least_two_sequences": len(per_sequence) >= 2,
        "fixed_configuration": True,
        "precision_non_decreasing_each_sequence": all(
            bool(item["precision_non_decreasing"]) for item in per_sequence
        ),
        "micro_precision_improved": temporal_precision > baseline_precision,
        "false_acceptances_reduced": (temporal_accepted - temporal_true)
        < (baseline_accepted - baseline_true),
        "retains_true_constraints": temporal_true > 0,
    }
    return {
        "schema_version": 1,
        "passed": all(checks.values()),
        "checks": checks,
        "config": configs[0],
        "aggregate": {
            "ranked_single": {
                "accepted_pairs": baseline_accepted,
                "true_accepted_pairs": baseline_true,
                "false_accepted_pairs": baseline_accepted - baseline_true,
                "micro_precision": baseline_precision,
                "conditional_pair_recall": baseline_true / true_pairs,
            },
            "temporal": {
                "accepted_pairs": temporal_accepted,
                "true_accepted_pairs": temporal_true,
                "false_accepted_pairs": temporal_accepted - temporal_true,
                "micro_precision": temporal_precision,
                "conditional_pair_recall": temporal_true / true_pairs,
            },
            "precision_delta": temporal_precision - baseline_precision,
            "pair_recall_delta": (temporal_true - baseline_true) / true_pairs,
        },
        "sequences": per_sequence,
        "sources": sources,
        "boundary": (
            "The temporal policy improves precision by rejecting constraints and therefore "
            "reduces recall. Results remain shadow-only; no constraint is inserted into Karto."
        ),
    }


def render_markdown(report: dict[str, object]) -> str:
    aggregate = report["aggregate"]
    lines = [
        "# Multi-sequence LiDAR temporal-consistency ablation",
        "",
        f"- 数据与策略门禁：**{'PASS' if report['passed'] else 'FAIL'}**",
        f"- 固定参数：`{json.dumps(report['config'], ensure_ascii=False)}`",
        "",
        "| Sequence | Ranked precision | Temporal precision | Precision delta | Recall delta |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for item in report["sequences"]:
        lines.append(
            f"| {item['sequence']} | {_precision(item['ranked_single']):.2%} | "
            f"{_precision(item['temporal']):.2%} | {item['precision_delta']:+.2%} | "
            f"{item['pair_recall_delta']:+.2%} |"
        )
    lines.extend(
        [
            "",
            f"- 聚合精度：{aggregate['ranked_single']['micro_precision']:.2%} → "
            f"{aggregate['temporal']['micro_precision']:.2%}",
            f"- 聚合条件召回：{aggregate['ranked_single']['conditional_pair_recall']:.2%} → "
            f"{aggregate['temporal']['conditional_pair_recall']:.2%}",
            "",
            "> 边界：这是 shadow-only 精度/召回权衡证据；尚未证明写入 Karto 后地图改善。",
            "",
        ]
    )
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", action="append", type=_assignment, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--markdown", type=Path, required=True)
    args = parser.parse_args()
    result = compare(args.report)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    args.markdown.write_text(render_markdown(result), encoding="utf-8")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"{'PASS' if result['passed'] else 'FAIL'}: temporal consistency ablation")
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
