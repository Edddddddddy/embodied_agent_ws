#!/usr/bin/env python3
"""Evaluate a saved continuous microphone report against a fixed command scenario."""

from __future__ import annotations

import argparse
import difflib
import json
import math
import re
from pathlib import Path
from statistics import median
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[2]


def _normalize(text: str) -> str:
    return re.sub(r"[^0-9a-zA-Z\u4e00-\u9fff]", "", str(text)).lower()


def _monotonic_text_matches(
    expected: list[str], observed: list[str], threshold: float
) -> list[dict[str, Any]]:
    """按时间顺序为期望话术与 ASR final 建立全局一对一对齐。

    现场可能先产生截断 final，随后又给出完整 final。局部贪心会让前一条命令
    抢走后一条唯一证据，因此这里用动态规划先最大化匹配条数，再最大化相似度。
    """

    normalized_expected = [_normalize(item) for item in expected]
    normalized_observed = [_normalize(item) for item in observed]
    scores = [
        [
            difflib.SequenceMatcher(None, wanted, actual).ratio()
            for actual in normalized_observed
        ]
        for wanted in normalized_expected
    ]
    expected_count = len(expected)
    observed_count = len(observed)

    # 每个状态保存“匹配条数、相似度总和”，确保匹配保持单调且一对一。
    dp = [
        [(0, 0.0) for _ in range(observed_count + 1)]
        for _ in range(expected_count + 1)
    ]
    choice = [
        ["done" for _ in range(observed_count + 1)]
        for _ in range(expected_count + 1)
    ]

    def rank(value: tuple[int, float], priority: int) -> tuple[int, float, int]:
        return value[0], round(value[1], 12), priority

    for expected_index in range(expected_count, -1, -1):
        for observed_index in range(observed_count, -1, -1):
            if expected_index == expected_count and observed_index == observed_count:
                continue
            options: list[tuple[tuple[int, float], int, str]] = []
            if expected_index < expected_count:
                options.append(
                    (dp[expected_index + 1][observed_index], 0, "skip_expected")
                )
            if observed_index < observed_count:
                options.append(
                    (dp[expected_index][observed_index + 1], 1, "skip_observed")
                )
            if expected_index < expected_count and observed_index < observed_count:
                score = scores[expected_index][observed_index]
                if score >= threshold and not _has_unsafe_control_slot_mismatch(
                    normalized_expected[expected_index],
                    normalized_observed[observed_index],
                ):
                    tail = dp[expected_index + 1][observed_index + 1]
                    options.append(((tail[0] + 1, tail[1] + score), 2, "match"))
            best_value, _, best_choice = max(
                options, key=lambda item: rank(item[0], item[1])
            )
            dp[expected_index][observed_index] = best_value
            choice[expected_index][observed_index] = best_choice

    assignments: dict[int, int] = {}
    expected_index = 0
    observed_index = 0
    while expected_index < expected_count or observed_index < observed_count:
        decision = choice[expected_index][observed_index]
        if decision == "match":
            assignments[expected_index] = observed_index
            expected_index += 1
            observed_index += 1
        elif decision == "skip_expected":
            expected_index += 1
        elif decision == "skip_observed":
            observed_index += 1
        else:
            break

    matches: list[dict[str, Any]] = []
    for index, wanted in enumerate(expected):
        assigned = assignments.get(index)
        best_score = max(scores[index], default=0.0)
        matches.append(
            {
                "expected": wanted,
                "observed": observed[assigned] if assigned is not None else "",
                # 未匹配项保留最高文本相似度，便于区分阈值问题与槽位安全拒绝。
                "similarity": round(
                    scores[index][assigned] if assigned is not None else best_score,
                    3,
                ),
                "matched": assigned is not None,
            }
        )
    return matches


def _has_unsafe_control_slot_mismatch(expected: str, observed: str) -> bool:
    """拒绝方向/颜色相反，或 ASR 漏掉期望关键槽位的伪高相似匹配。"""

    mutually_exclusive_slots = (
        ("左", "右"),
        ("前", "后"),
        ("蓝", "红", "绿", "黄"),
    )
    for slots in mutually_exclusive_slots:
        expected_values = {slot for slot in slots if slot in expected}
        if not expected_values:
            continue
        observed_values = {slot for slot in slots if slot in observed}
        # 编辑距离无法保证控制安全：缺失、替换或混入相反槽位都不能作为通过证据。
        if observed_values != expected_values:
            return True
    return False


def _percentile(values: list[float], percentile: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(percentile * len(ordered)) - 1))
    return round(ordered[index], 3)


def _monotonic_action_match_count(expected: list[str], observed: list[str]) -> int:
    """返回两个动作序列的最长公共子序列长度。

    动作集合里会重复出现 move/turn。旧实现按 expected 贪心查找，漏掉一次左转后
    会把右转对应的 ``turn`` 错配给左转，并连带错扣下一次 ``move``。LCS 仍严格
    保持执行顺序，但允许单个漏识别独立计错，报告才能忠实反映 ROS 实际执行证据。
    """
    previous = [0] * (len(observed) + 1)
    for wanted in expected:
        current = [0]
        for index, actual in enumerate(observed, start=1):
            if wanted == actual:
                current.append(previous[index - 1] + 1)
            else:
                current.append(max(previous[index], current[index - 1]))
        previous = current
    return previous[-1]


def _metric_values(rows: list[dict[str, Any]], *keys: str) -> list[float]:
    """读取在线/离线 Agent 的同义延迟字段，不伪造缺失的测量值。"""
    values: list[float] = []
    for row in rows:
        for key in keys:
            value = row.get(key)
            if isinstance(value, (int, float)):
                values.append(float(value))
                break
    return values


def evaluate(
    report: dict[str, Any], scenario: dict[str, Any], *, text_threshold: float = 0.65
) -> dict[str, Any]:
    commands = list(scenario.get("commands") or [])
    expected_text = [str(item["text"]) for item in commands]
    expected_actions = [str(item["action"]) for item in commands]
    observed_asr = [str(item) for item in report.get("asr_samples") or []]
    observed_candidates = [
        str(item.get("name") or "")
        for item in report.get("action_candidate_samples") or []
        if isinstance(item, dict) and item.get("name")
    ]
    text_matches = _monotonic_text_matches(expected_text, observed_asr, text_threshold)
    matched_text = sum(1 for item in text_matches if item["matched"])
    matched_actions = _monotonic_action_match_count(
        expected_actions, observed_candidates
    )
    # 误触发只统计“超过计划数量的额外 candidate”；动作类型错误单独计入
    # action_accuracy，避免同一个错误被重复惩罚两次。
    unexpected_candidates = max(0, len(observed_candidates) - len(expected_actions))
    successful_results = [
        item
        for item in report.get("successful_action_samples") or []
        if isinstance(item, dict) and item.get("success") is True
    ]
    successful_action_sequence = [
        str(item.get("action_name") or "")
        for item in successful_results
        if item.get("action_name")
    ]
    matched_successes = _monotonic_action_match_count(
        expected_actions, successful_action_sequence
    )
    # 旧报告只有 success 总数，无法证明具体是哪条 action 成功；评测器宁可标记
    # 证据不足，也不以不相关的成功结果冒充固定话术执行成功。
    success_count = matched_successes
    metric_rows = [
        item for item in report.get("metrics_samples") or [] if isinstance(item, dict)
    ]
    latency_values = {}
    metric_aliases = {
        "asr_finalize_ms": ("asr_finalize_ms",),
        "asr_to_first_token_ms": ("asr_to_first_token_ms",),
        "llm_first_token_ms": ("llm_first_token_ms",),
        # 离线链路从语音端点计时，字段名为 end_to_first_audio_ms；在线链路
        # 从 TTS request 计时。报告保留统一展示名，同时记录实际来源字段。
        "tts_first_audio_ms": ("tts_first_audio_ms", "end_to_first_audio_ms"),
        "turn_complete_ms": ("turn_complete_ms",),
    }
    for key, aliases in metric_aliases.items():
        values = _metric_values(metric_rows, *aliases)
        latency_values[key] = {
            "count": len(values),
            "median_ms": None if not values else round(median(values), 3),
            "p95_ms": _percentile(values, 0.95),
            "source_fields": list(aliases),
        }
    e2e_values = [
        float(value)
        for value in report.get("action_e2e_latency_ms") or []
        if isinstance(value, (int, float))
    ]
    latency_values["asr_final_to_action_result_ms"] = {
        "count": len(e2e_values),
        "median_ms": None if not e2e_values else round(median(e2e_values), 3),
        "p95_ms": _percentile(e2e_values, 0.95),
    }
    count = max(1, len(commands))
    recognition_rate = matched_text / count
    action_accuracy = matched_actions / count
    action_success_rate = min(success_count, len(commands)) / count
    false_trigger_rate = unexpected_candidates / max(1, len(observed_candidates))
    queue_rejected_count = int(report.get("queue_rejected_count", 0))
    # enqueue_count 只统计已接收项；拒绝项属于额外尝试，分母必须相加，不能用 max 低估拒绝率。
    queue_attempts = (
        int(report.get("command_enqueue_count", 0)) + queue_rejected_count
    )
    queue_reject_rate = queue_rejected_count / max(1, queue_attempts)
    required_duration_s = float(scenario.get("recommended_duration_s") or 300.0)
    checks = {
        "expected_command_count": len(commands) >= 10,
        f"duration_at_least_{int(required_duration_s)}s": (
            float(report.get("duration_s", 0.0)) >= required_duration_s
        ),
        "recognition_rate_at_least_80pct": recognition_rate >= 0.8,
        "action_accuracy_at_least_80pct": action_accuracy >= 0.8,
        "action_success_rate_at_least_80pct": action_success_rate >= 0.8,
        "false_trigger_rate_at_most_10pct": false_trigger_rate <= 0.1,
        "session_awake_and_sleeping": bool(report.get("saw_awake"))
        and bool(report.get("saw_sleeping")),
        "final_cmd_vel_zero": bool(report.get("final_cmd_vel_zero")),
    }
    capture_source = str(report.get("capture_source", "unspecified"))
    long_enough = float(report.get("duration_s", 0.0)) >= required_duration_s
    evidence_scope = (
        "operator_declared_real_microphone"
        if long_enough and capture_source == "real_microphone"
        else "synthetic_short_or_unspecified"
    )
    return {
        "schema_version": 1,
        "scenario": scenario.get("name", ""),
        "source_report_duration_s": report.get("duration_s", 0.0),
        "required_duration_s": required_duration_s,
        "agent_mode": str(report.get("agent_mode", "unspecified")),
        "expected_commands": len(commands),
        "matched_asr_commands": matched_text,
        "recognition_rate": round(recognition_rate, 4),
        "matched_action_sequence": matched_actions,
        "action_accuracy": round(action_accuracy, 4),
        "successful_action_results": success_count,
        "successful_action_sequence": successful_action_sequence,
        "action_success_rate": round(action_success_rate, 4),
        "unexpected_candidates": unexpected_candidates,
        "false_trigger_rate": round(false_trigger_rate, 4),
        "queue_rejected_count": queue_rejected_count,
        "queue_reject_rate": round(queue_reject_rate, 4),
        "ignored_transcript_count": int(report.get("ignored_transcript_count", 0)),
        "recognition_retry_count": int(report.get("recognition_retry_count", 0)),
        # 单独展示 partial 恢复次数：它是鲁棒性证据，不计作额外 ASR 或动作。
        "asr_final_recovery_count": int(report.get("asr_final_recovery_count", 0)),
        "latency": latency_values,
        "text_matches": text_matches,
        "observed_candidate_sequence": observed_candidates,
        "checks": checks,
        "passed": all(checks.values()),
        "capture_source": capture_source,
        "evidence_scope": evidence_scope,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument(
        "--scenario",
        type=Path,
        default=WORKSPACE / "training" / "voice_command_benchmark_zh.json",
    )
    parser.add_argument("--output", type=Path)
    parser.add_argument("--text-threshold", type=float, default=0.65)
    args = parser.parse_args()
    report = json.loads(args.report.read_text(encoding="utf-8"))
    scenario = json.loads(args.scenario.read_text(encoding="utf-8"))
    result = evaluate(report, scenario, text_threshold=args.text_threshold)
    output = args.output
    if output is None:
        mode = str(result.get("agent_mode") or "unspecified")
        output = WORKSPACE / "logs" / f"voice_benchmark_{mode}_report.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
