#!/usr/bin/env python3
"""汇总求职展示版的自动证据、真实语音证据和可视化证据。

这个脚本不启动 Gazebo/Nav2，也不冒充真实麦克风测试；它只做一件事：
把已经生成的报告文件和可选录屏/截图资产整理成一份 checklist。

默认模式用于演示前排障：即使缺少真实证据也退出 0，并在 JSON/Markdown 里写清楚缺口。
严格模式用于发布/演示前门禁：任一 required evidence 缺失或失败都会退出 1。
"""

from __future__ import annotations

import argparse
import json
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class EvidenceItem:
    id: str
    title: str
    required: bool
    status: str
    evidence_kind: str
    path: str
    command: str
    summary: str
    next_action: str


@dataclass(frozen=True)
class ChecklistReport:
    schema_version: int
    scenario: str
    generated_at: str
    workspace: str
    strict: bool
    require_nav2: bool
    require_visual_evidence: bool
    ok: bool
    required_passed: int
    required_total: int
    optional_passed: int
    optional_total: int
    missing_required: list[str]
    items: list[EvidenceItem] = field(default_factory=list)


def _load_json(path: Path) -> tuple[dict[str, Any] | None, str]:
    if not path.is_file():
        return None, "file_missing"
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        return None, f"invalid_json: {exc}"
    if not isinstance(payload, dict):
        return None, "json_root_not_object"
    return payload, "ok"


def _status_from_report(path: Path, *, required: bool, expected_scenario: str | None = None) -> tuple[str, str]:
    payload, reason = _load_json(path)
    if payload is None:
        return ("missing" if required else "optional_missing"), reason
    if expected_scenario and payload.get("scenario") != expected_scenario:
        return "invalid", f"scenario={payload.get('scenario')!r}"
    if payload.get("ok") is True:
        return "passed", "ok=true"
    return "failed", f"ok={payload.get('ok')!r}"


def _automatic_summary(path: Path) -> str:
    payload, reason = _load_json(path)
    if payload is None:
        return f"未找到自动 demo gate 报告：{reason}"
    command_count = payload.get("command_count", "?")
    evidence_summary = payload.get("evidence_summary", {})
    manual_followups = (
        payload.get("evidence_policy", {}).get("manual_followups", [])
        if isinstance(payload.get("evidence_policy"), dict)
        else []
    )
    return (
        f"自动 gate ok={payload.get('ok')!r}，commands={command_count}，"
        f"evidence_summary={evidence_summary}，manual_followups={manual_followups}"
    )


def _live_summary(path: Path) -> str:
    payload, reason = _load_json(path)
    if payload is None:
        return f"未找到 live-check 报告：{reason}"
    return (
        f"ok={payload.get('ok')!r}，asr={payload.get('asr_count')}，"
        f"candidate={payload.get('action_candidate_count')}，"
        f"success={payload.get('action_success_count')}，"
        f"final_cmd_vel_zero={payload.get('final_cmd_vel_zero')}，"
        f"missing={payload.get('missing')}"
    )


def _nav2_summary(path: Path) -> str:
    payload, reason = _load_json(path)
    if payload is None:
        return f"未找到 Nav2 live-check 报告：{reason}"
    names = payload.get("action_candidate_names", {})
    failures = payload.get("navigation_failure_reasons", [])
    return (
        f"ok={payload.get('ok')!r}，action_candidate_names={names}，"
        f"navigation_failure_reasons={failures}"
    )


def _file_status(path: Path, *, required: bool) -> tuple[str, str]:
    if path.is_file() and path.stat().st_size > 0:
        return "passed", f"size={path.stat().st_size}"
    return ("missing" if required else "optional_missing"), "file_missing_or_empty"


def build_report(args: argparse.Namespace) -> ChecklistReport:
    workspace = Path(args.workspace).expanduser().resolve()
    automatic_report = Path(args.automatic_report).expanduser()
    voice_report = Path(args.voice_report).expanduser()
    nav2_report = Path(args.nav2_report).expanduser()
    recording = Path(args.recording).expanduser()
    screenshot = Path(args.screenshot).expanduser()

    items: list[EvidenceItem] = []

    status, detail = _status_from_report(
        automatic_report,
        required=True,
        expected_scenario="job_showcase_release_gate",
    )
    items.append(
        EvidenceItem(
            id="automatic_demo_gate",
            title="自动 demo gate 报告",
            required=True,
            status=status,
            evidence_kind="automatic_report",
            path=str(automatic_report),
            command="bash scripts/acceptance_test.sh demo-gate",
            summary=f"{_automatic_summary(automatic_report)}；detail={detail}",
            next_action="先运行 demo-gate，生成 logs/demo_acceptance_report.json。",
        )
    )

    status, detail = _status_from_report(voice_report, required=True)
    items.append(
        EvidenceItem(
            id="continuous_voice_live",
            title="真实麦克风连续语音 live-check",
            required=True,
            status=status,
            evidence_kind="manual_live_report",
            path=str(voice_report),
            command=(
                "CONTINUOUS_LIVE_CHECK_REPORT=logs/continuous-live-check.json "
                "bash scripts/acceptance_test.sh continuous-live-check offline"
            ),
            summary=f"{_live_summary(voice_report)}；detail={detail}",
            next_action=(
                "终端 1 跑 continuous-offline，终端 2 跑 continuous-live-check offline，"
                "按固定话术完成 3~5 分钟真实语音演示。"
            ),
        )
    )

    status, detail = _status_from_report(nav2_report, required=args.require_nav2)
    items.append(
        EvidenceItem(
            id="nav2_voice_live",
            title="Nav2 语音目标点导航 live-check",
            required=bool(args.require_nav2),
            status=status,
            evidence_kind="manual_nav2_report",
            path=str(nav2_report),
            command=(
                "CONTINUOUS_LIVE_CHECK_REPORT=logs/nav2-live-check.json "
                "bash scripts/acceptance_test.sh continuous-nav2-evidence offline"
            ),
            summary=f"{_nav2_summary(nav2_report)}；detail={detail}",
            next_action="演示语音目标点导航时运行 continuous-nav2-evidence offline 留证。",
        )
    )

    for evidence_id, title, path, command in (
        (
            "demo_recording",
            "演示录屏",
            recording,
            "Windows Xbox Game Bar / OBS / PowerPoint 录屏，保存到 logs/demo_recording.mp4",
        ),
        (
            "rviz_or_gazebo_screenshot",
            "RViz/Gazebo 截图",
            screenshot,
            "演示时截图保存到 logs/demo_screenshot.png",
        ),
    ):
        status, detail = _file_status(path, required=args.require_visual_evidence)
        items.append(
            EvidenceItem(
                id=evidence_id,
                title=title,
                required=bool(args.require_visual_evidence),
                status=status,
                evidence_kind="visual_artifact",
                path=str(path),
                command=command,
                summary=detail,
                next_action="演示前后保留视觉证据，方便复盘和项目汇报。",
            )
        )

    required_items = [item for item in items if item.required]
    optional_items = [item for item in items if not item.required]
    missing_required = [
        item.id for item in required_items if item.status not in {"passed"}
    ]
    return ChecklistReport(
        schema_version=1,
        scenario="job_showcase_demo_evidence_checklist",
        generated_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        workspace=str(workspace),
        strict=bool(args.strict),
        require_nav2=bool(args.require_nav2),
        require_visual_evidence=bool(args.require_visual_evidence),
        ok=not missing_required,
        required_passed=sum(1 for item in required_items if item.status == "passed"),
        required_total=len(required_items),
        optional_passed=sum(1 for item in optional_items if item.status == "passed"),
        optional_total=len(optional_items),
        missing_required=missing_required,
        items=items,
    )


def write_json(path: Path, report: ChecklistReport) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(asdict(report), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def write_markdown(path: Path, report: ChecklistReport) -> None:
    lines = [
        "# 求职展示演示证据 Checklist",
        "",
        f"- 生成时间：`{report.generated_at}`",
        f"- strict：`{report.strict}`",
        f"- 总体结果：`{'PASS' if report.ok else 'INCOMPLETE'}`",
        f"- required：`{report.required_passed}/{report.required_total}`",
        f"- optional：`{report.optional_passed}/{report.optional_total}`",
        "",
        "| 证据 | 必需 | 状态 | 路径 | 下一步 |",
        "| --- | --- | --- | --- | --- |",
    ]
    for item in report.items:
        lines.append(
            f"| {item.title} | {'是' if item.required else '否'} | `{item.status}` | "
            f"`{item.path}` | {item.next_action} |"
        )
    lines.extend(
        [
            "",
            "## 现场演示推荐顺序",
            "",
            "1. `bash scripts/acceptance_test.sh demo-gate`",
            "2. `bash scripts/acceptance_test.sh continuous-offline`",
            "3. 另一个终端运行 `continuous-live-check offline` 并保存报告。",
            "4. 如果展示 Nav2，运行 `continuous-nav2-evidence offline`。",
            "5. 保存录屏和 Gazebo/RViz 截图。",
            "6. 最后运行 `DEMO_EVIDENCE_STRICT=true bash scripts/acceptance_test.sh demo-evidence-checklist`。",
            "",
            "## 详细摘要",
            "",
        ]
    )
    for item in report.items:
        lines.extend(
            [
                f"### {item.title}",
                "",
                f"- command: `{item.command}`",
                f"- summary: {item.summary}",
                "",
            ]
        )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default="/home/ubuntu/embodied_agent_ws")
    parser.add_argument("--automatic-report", default="logs/demo_acceptance_report.json")
    parser.add_argument("--voice-report", default="logs/continuous-live-check.json")
    parser.add_argument("--nav2-report", default="logs/nav2-live-check.json")
    parser.add_argument("--recording", default="logs/demo_recording.mp4")
    parser.add_argument("--screenshot", default="logs/demo_screenshot.png")
    parser.add_argument("--output", default="logs/demo_evidence_checklist.json")
    parser.add_argument("--markdown", default="logs/demo_evidence_checklist.md")
    parser.add_argument("--require-nav2", action="store_true")
    parser.add_argument("--require-visual-evidence", action="store_true")
    parser.add_argument("--strict", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    report = build_report(args)
    write_json(Path(args.output).expanduser(), report)
    write_markdown(Path(args.markdown).expanduser(), report)
    print(json.dumps(asdict(report), ensure_ascii=False, indent=2), flush=True)
    if args.strict and not report.ok:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
