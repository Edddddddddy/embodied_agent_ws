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
import os
import shlex
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path


def _owned_colcon_test_command(*packages: str) -> str:
    """只汇总本次自有包结果，隔离 vendor 与历史 build 的 xUnit 文件。"""

    selected = " ".join(packages)
    reports = " && ".join(
        f"colcon test-result --test-result-base build/{package} --verbose"
        for package in packages
    )
    return (
        f"colcon test --packages-select {selected} --event-handlers console_direct+ && "
        f"{reports}"
    )


CORE_COMMANDS: tuple[tuple[str, str], ...] = (
    (
        "python_repository_and_agent_units",
        "pytest -q tests/repository src/embodied_agent_core/test "
        "src/embodied_voice_frontend/test src/embodied_offline_agent/test",
    ),
    (
        "cli_and_instruction_parser",
        "bash tests/integration/control/test_acceptance_cli.sh && "
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
        + _owned_colcon_test_command("embodied_agent_cpp", "embodied_simulation"),
    ),
)

FULL_COMMANDS: tuple[tuple[str, str], ...] = (
    (
        "repository_and_offline_unit",
        "pytest -q tests/repository src/embodied_offline_agent/test",
    ),
    ("acceptance_cli", "bash tests/integration/control/test_acceptance_cli.sh"),
    ("instruction_parser_eval", "bash scripts/acceptance_test.sh instruction-parser-eval"),
    ("continuous_mock", "bash scripts/acceptance_test.sh continuous-mock"),
    ("continuous_multi_command", "bash scripts/acceptance_test.sh continuous-multi-command"),
    ("navigation_demo", "bash scripts/acceptance_test.sh navigation-demo"),
    ("offline_latency", "bash scripts/acceptance_test.sh offline-latency"),
    ("summer_tts_service", "bash scripts/acceptance_test.sh summer-tts-service"),
    (
        "cpp_ros_unit",
        _owned_colcon_test_command("embodied_agent_cpp", "embodied_simulation"),
    ),
)

DEMO_COMMANDS: tuple[tuple[str, str], ...] = (
    (
        "demo_cli_readiness",
        "bash tests/integration/control/test_acceptance_cli.sh",
    ),
    (
        "voice_provider_readiness",
        "bash scripts/acceptance_test.sh provider-preflight && "
        "bash scripts/acceptance_test.sh voice-readiness && "
        "bash scripts/acceptance_test.sh voice-calibration-report",
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

ROBOTICS_COMMANDS: tuple[tuple[str, str], ...] = (
    (
        "robotics_repository_and_agent_units",
        "bash scripts/acceptance_test.sh architecture-facts && "
        "pytest -q tests/repository src/embodied_online_agent/test "
        "src/embodied_offline_agent/test",
    ),
    (
        "robotics_cpp_packages",
        _owned_colcon_test_command(
            "embodied_agent_bringup",
            "embodied_agent_cpp",
            "embodied_simulation",
            "embodied_navigation",
            "embodied_slam",
        ),
    ),
    (
        "robotics_continuous_multi_command",
        "bash scripts/acceptance_test.sh continuous-multi-command",
    ),
    (
        "robotics_control_authority",
        "bash scripts/acceptance_test.sh control-authority-stage",
    ),
    (
        "robotics_nav2_stage",
        "bash scripts/acceptance_test.sh nav2-stage",
    ),
    (
        "robotics_slam_public_bag",
        "bash scripts/acceptance_test.sh slam-evaluation-stage && "
        "bash scripts/acceptance_test.sh openloris-replay-stage",
    ),
    (
        "robotics_dynamic_obstacle",
        "bash scripts/acceptance_test.sh dynamic-obstacle-stage",
    ),
)


PROFILE_COMMANDS = {
    "core": CORE_COMMANDS,
    "demo": DEMO_COMMANDS,
    "full": FULL_COMMANDS,
    "robotics": ROBOTICS_COMMANDS,
}

EVIDENCE_KIND_BY_COMMAND = {
    "python_repository_and_agent_units": "ci_compatible",
    "cli_and_instruction_parser": "ci_compatible",
    "continuous_voice_queue": "mock_ros",
    "voice_navigation_demo": "mock_ros",
    "offline_runtime_and_cpp_ros": "local_runtime",
    "repository_and_offline_unit": "ci_compatible",
    "acceptance_cli": "ci_compatible",
    "instruction_parser_eval": "ci_compatible",
    "continuous_mock": "mock_ros",
    "continuous_multi_command": "mock_ros",
    "navigation_demo": "mock_ros",
    "offline_latency": "local_runtime",
    "summer_tts_service": "local_runtime",
    "cpp_ros_unit": "cpp_ros",
    "demo_cli_readiness": "ci_compatible",
    "voice_provider_readiness": "local_preflight",
    "speaker_memory_preferences": "mock_ros",
    "continuous_voice_demo": "mock_ros",
    "navigation_and_offline_evidence": "mixed_evidence",
    "robotics_repository_and_agent_units": "ci_compatible",
    "robotics_cpp_packages": "cpp_ros",
    "robotics_continuous_multi_command": "mock_ros",
    "robotics_control_authority": "ros_stage",
    "robotics_nav2_stage": "mock_ros",
    "robotics_slam_public_bag": "public_bag",
    "robotics_dynamic_obstacle": "mock_ros",
}

MANUAL_FOLLOWUPS_BY_PROFILE = {
    "core": (
        "continuous-offline",
        "continuous-online",
        "gazebo",
        "nav2-turtlebot3",
    ),
    "demo": (
        "continuous-offline",
        "gazebo",
        "nav2-stage",
        "continuous-nav2-evidence offline",
    ),
    "full": (
        "continuous-offline",
        "continuous-online",
        "gazebo",
        "nav2-turtlebot3",
        "continuous-nav2-evidence offline",
    ),
    "robotics": (
        "continuous-offline",
        "continuous-online",
        "nav2-turtlebot3",
        "slam-benchmark",
        "dynamic-obstacle-navigation",
    ),
}


@dataclass(frozen=True)
class GateCommand:
    name: str
    command: str


def _tail(text: str, max_lines: int) -> list[str]:
    lines = text.splitlines()
    return lines[-max_lines:]


def _evidence_kind(command_name: str) -> str:
    return EVIDENCE_KIND_BY_COMMAND.get(command_name, "unknown")


def _evidence_summary(commands: list[GateCommand]) -> dict[str, int]:
    summary: dict[str, int] = {}
    for command in commands:
        kind = _evidence_kind(command.name)
        summary[kind] = summary.get(kind, 0) + 1
    return dict(sorted(summary.items()))


def _evidence_policy(profile: str) -> dict:
    """Describe what the gate proves and what still needs human/live validation.

    release/demo gate 主要用于固定自动证据。真实麦克风、Gazebo 图形和 Nav2 重型链路
    仍受本机音频、图形和模型资产影响，不能被 dry-run 或 mock smoke 冒充。
    """

    return {
        "profile": profile,
        "automatic_report": {
            "demo": "logs/demo_acceptance_report.json",
            "robotics": "logs/robotics_acceptance_report.json",
        }.get(profile, "logs/acceptance_report.json"),
        "evidence_kinds": {
            "ci_compatible": "纯仓库/解析/单元测试，适合 CI 或快速本地门禁。",
            "mock_ros": "不依赖真实麦克风或 Gazebo 图形的 ROS/mock 链路证据。",
            "local_preflight": "本机 provider、音频、KWS 或模型依赖预检证据。",
            "local_runtime": "依赖本机模型/ROS2 runtime 的本地证据，不默认放入 CI。",
            "cpp_ros": "C++/ROS2 单测或组件测试证据。",
            "mixed_evidence": "组合报告，可能混合 mock、preflight 和本地 runtime 证据。",
            "public_bag": "公开 rosbag/fixture 的可复现实验，不等同于真实机器人现场数据。",
            "real_model": "依赖本机真实 ASR/LLM/TTS 模型的运行证据。",
            "gazebo": "依赖 Gazebo 物理仿真的运动、里程计或导航证据。",
        },
        "requires_human_demo": True,
        "manual_followups": list(MANUAL_FOLLOWUPS_BY_PROFILE[profile]),
        "note": "自动 gate 不能替代真实麦克风、Gazebo/RViz 或 Nav2 TurtleBot3 人工演示证据。",
    }


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
        "evidence_kind": _evidence_kind(command.name),
        "returncode": completed.returncode,
        "ok": completed.returncode == 0,
        "duration_s": round(elapsed, 2),
        "output_tail": _tail(completed.stdout, tail_lines),
    }


def _default_report_path(root: Path, profile: str) -> Path:
    if profile == "demo":
        return root / "logs" / "demo_acceptance_report.json"
    if profile == "robotics":
        return root / "logs" / "robotics_acceptance_report.json"
    return root / "logs" / "acceptance_report.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--workspace",
        default=os.environ.get("WORKSPACE", "/home/ubuntu/embodied_agent_ws"),
    )
    parser.add_argument("--output", default="")
    parser.add_argument("--timeout-s", type=float, default=900.0)
    parser.add_argument("--tail-lines", type=int, default=80)
    parser.add_argument(
        "--profile",
        choices=sorted(PROFILE_COMMANDS),
        default="core",
        help=(
            "core 是快速求职展示门禁；robotics 覆盖连续命令、Nav2、SLAM、"
            "公开 bag 与动态障碍；full 保留更慢的通用本地验收。"
        ),
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
            "evidence_summary": _evidence_summary(commands),
            "evidence_policy": _evidence_policy(args.profile),
            "commands": [
                {
                    "name": command.name,
                    "command": command.command,
                    "evidence_kind": _evidence_kind(command.name),
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
            "evidence_summary": _evidence_summary(commands),
            "evidence_policy": _evidence_policy(args.profile),
            "commands": results,
        }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if not report["ok"]:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
