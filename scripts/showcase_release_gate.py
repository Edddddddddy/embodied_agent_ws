#!/usr/bin/env python3
"""Run the job-showcase release gate and write a machine-readable report.

这个脚本解决两个痛点：

1. README 里散落了很多验收命令，新读者不知道“提交前到底跑哪一组”。
2. 真实演示前需要一份可复查证据，而不是只靠终端滚动日志。

默认会执行一组适合求职展示版的本地 gate，并把每条命令的耗时、退出码和尾部日志
写入 JSON。CI/单测只使用 --dry-run，不会启动模型、Gazebo 或长时间 ROS graph。
"""

from __future__ import annotations

import argparse
import json
import shlex
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path


CORE_COMMANDS: tuple[tuple[str, str], ...] = (
    (
        "python_repository_and_agent_units",
        "pytest -q tests/repository src/embodied_online_agent/test src/embodied_offline_agent/test",
    ),
    (
        "cli_and_instruction_parser",
        "bash tests/integration/test_acceptance_cli.sh && "
        "bash scripts/acceptance_test.sh instruction-eval-dataset && "
        "bash scripts/acceptance_test.sh instruction-parser-eval",
    ),
    (
        "continuous_voice_queue",
        "bash scripts/acceptance_test.sh continuous-mock && "
        "bash scripts/acceptance_test.sh continuous-multi-command",
    ),
    (
        "voice_navigation_demo",
        "bash scripts/acceptance_test.sh navigation-demo && "
        "bash scripts/acceptance_test.sh nav2-bridge",
    ),
    (
        "offline_runtime_and_cpp_ros",
        "bash scripts/acceptance_test.sh offline-latency && "
        "bash scripts/acceptance_test.sh summer-tts-service && "
        "colcon test --packages-select embodied_agent_cpp embodied_simulation "
        "--event-handlers console_direct+ && colcon test-result --verbose",
    ),
)

FULL_COMMANDS: tuple[tuple[str, str], ...] = (
    (
        "repository_and_offline_unit",
        "pytest -q tests/repository src/embodied_offline_agent/test",
    ),
    ("acceptance_cli", "bash tests/integration/test_acceptance_cli.sh"),
    ("instruction_parser_eval", "bash scripts/acceptance_test.sh instruction-parser-eval"),
    ("continuous_mock", "bash scripts/acceptance_test.sh continuous-mock"),
    ("continuous_multi_command", "bash scripts/acceptance_test.sh continuous-multi-command"),
    ("navigation_demo", "bash scripts/acceptance_test.sh navigation-demo"),
    ("offline_latency", "bash scripts/acceptance_test.sh offline-latency"),
    ("summer_tts_service", "bash scripts/acceptance_test.sh summer-tts-service"),
    (
        "cpp_ros_unit",
        "colcon test --packages-select embodied_agent_cpp embodied_simulation "
        "--event-handlers console_direct+ && colcon test-result --verbose",
    ),
)

DEMO_COMMANDS: tuple[tuple[str, str], ...] = (
    (
        "demo_cli_readiness",
        "bash tests/integration/test_acceptance_cli.sh",
    ),
    (
        "voice_provider_readiness",
        "bash scripts/acceptance_test.sh provider-preflight && "
        "bash scripts/acceptance_test.sh voice-readiness",
    ),
    (
        "speaker_memory_preferences",
        "bash scripts/acceptance_test.sh speaker-memory-mock",
    ),
    (
        "continuous_voice_demo",
        "bash scripts/acceptance_test.sh continuous-mock && "
        "bash scripts/acceptance_test.sh continuous-multi-command",
    ),
    (
        "navigation_and_offline_evidence",
        "bash scripts/acceptance_test.sh navigation-demo && "
        "bash scripts/acceptance_test.sh offline-showcase-report",
    ),
)


PROFILE_COMMANDS = {
    "core": CORE_COMMANDS,
    "demo": DEMO_COMMANDS,
    "full": FULL_COMMANDS,
}


@dataclass(frozen=True)
class GateCommand:
    name: str
    command: str


def _tail(text: str, max_lines: int) -> list[str]:
    lines = text.splitlines()
    return lines[-max_lines:]


def _run_command(command: GateCommand, *, root: Path, timeout_s: float, tail_lines: int) -> dict:
    started = time.perf_counter()
    completed = subprocess.run(
        command.command,
        cwd=root,
        shell=True,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        timeout=timeout_s,
        check=False,
    )
    elapsed = time.perf_counter() - started
    return {
        "name": command.name,
        "command": command.command,
        "returncode": completed.returncode,
        "ok": completed.returncode == 0,
        "duration_s": round(elapsed, 2),
        "output_tail": _tail(completed.stdout, tail_lines),
    }


def _default_report_path(root: Path, profile: str) -> Path:
    if profile == "demo":
        return root / "logs" / "demo_acceptance_report.json"
    return root / "logs" / "acceptance_report.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", default="/home/ubuntu/embodied_agent_ws")
    parser.add_argument("--output", default="")
    parser.add_argument("--timeout-s", type=float, default=900.0)
    parser.add_argument("--tail-lines", type=int, default=80)
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILE_COMMANDS),
        default="core",
        help="core 固定 5 条求职展示门禁；full 保留更完整但更慢的本地验收。",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Only print/write the command plan; do not execute gate commands.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    root = Path(args.workspace).expanduser().resolve()
    output_path = (
        Path(args.output).expanduser()
        if args.output
        else _default_report_path(root, args.profile)
    )
    commands = [GateCommand(name, command) for name, command in PROFILE_COMMANDS[args.profile]]
    started = time.strftime("%Y-%m-%dT%H:%M:%S%z")

    if args.dry_run:
        report = {
            "schema_version": 1,
            "scenario": "job_showcase_release_gate",
            "profile": args.profile,
            "workspace": str(root),
            "started_at": started,
            "dry_run": True,
            "ok": True,
            "command_count": len(commands),
            "commands": [
                {
                    "name": command.name,
                    "command": command.command,
                    "argv_preview": shlex.split(command.command),
                }
                for command in commands
            ],
        }
    else:
        results = []
        for command in commands:
            print(f"[release-gate] RUN {command.name}: {command.command}", flush=True)
            result = _run_command(
                command,
                root=root,
                timeout_s=args.timeout_s,
                tail_lines=max(1, args.tail_lines),
            )
            results.append(result)
            status = "PASS" if result["ok"] else "FAIL"
            print(
                f"[release-gate] {status} {command.name} "
                f"duration={result['duration_s']}s",
                flush=True,
            )
            if not result["ok"]:
                break
        report = {
            "schema_version": 1,
            "scenario": "job_showcase_release_gate",
            "profile": args.profile,
            "workspace": str(root),
            "started_at": started,
            "finished_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "dry_run": False,
            "ok": all(item["ok"] for item in results) and len(results) == len(commands),
            "command_count": len(commands),
            "passed_count": sum(1 for item in results if item["ok"]),
            "failed_count": sum(1 for item in results if not item["ok"]),
            "commands": results,
        }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
