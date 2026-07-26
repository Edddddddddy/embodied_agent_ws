#!/usr/bin/env python3
"""统一运行项目关键功能验收，并生成机器可读报告。"""

from __future__ import annotations

import argparse
from collections import deque
import json
import os
import signal
import subprocess
import threading
import time
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Sequence


ROOT = Path(__file__).resolve().parents[3]


@dataclass(frozen=True, slots=True)
class VerificationCheck:
    """一个验收步骤只描述入口和预算，具体实现继续由原 mode 负责。"""

    name: str
    mode: str
    timeout_s: float
    evidence: str
    requires_local_models: bool = False


CHECKS = {
    "core": VerificationCheck(
        "core",
        "core",
        1200.0,
        "repository, Python and owned C++ unit tests",
    ),
    "voice_frontend_readiness": VerificationCheck(
        "voice_frontend_readiness",
        "voice-readiness",
        90.0,
        "synthetic frontend metrics and KWS diagnostic aggregation",
    ),
    "voice_endpoint": VerificationCheck(
        "voice_endpoint",
        "continuous-endpoint",
        180.0,
        "speech endpoint -> ASR commit -> online/offline Agent action",
    ),
    "voice_multi_command": VerificationCheck(
        "voice_multi_command",
        "continuous-multi-command",
        180.0,
        "one ASR final -> ordered multi-command queue and correlated results",
    ),
    "voice_offline_model": VerificationCheck(
        "voice_offline_model",
        "offline-sherpa-typed",
        300.0,
        "deterministic synthesized PCM -> Sherpa ASR -> llama.cpp -> typed Action -> TTS",
        requires_local_models=True,
    ),
    "cpp_action_client": VerificationCheck(
        "cpp_action_client",
        "cpp-action-client",
        180.0,
        "typed ROS 2 Action feedback, result, cancel and timeout",
    ),
    "cpp_action_scheduler": VerificationCheck(
        "cpp_action_scheduler",
        "cpp-action-scheduler",
        180.0,
        "FIFO queue, priority stop and result correlation",
    ),
    "control_authority": VerificationCheck(
        "control_authority",
        "control-authority-stage",
        240.0,
        "ActionGuard, control authority lease and fail-closed zero velocity",
    ),
    "gazebo": VerificationCheck(
        "gazebo",
        "gazebo",
        180.0,
        "typed Action -> BT/plugin executor -> Gazebo odometry -> final stop",
    ),
    "slam_nav": VerificationCheck(
        "slam_nav",
        "unknown-world-slam-e2e",
        4200.0,
        "unknown-world exploration -> return -> map save -> AMCL/Nav2 -> dynamic replan",
    ),
}

PROFILE_CHECK_NAMES = {
    "core": ("core",),
    "voice": (
        "voice_frontend_readiness",
        "voice_endpoint",
        "voice_multi_command",
        "voice_offline_model",
    ),
    "control": (
        "cpp_action_client",
        "cpp_action_scheduler",
        "control_authority",
    ),
    "gazebo": ("gazebo",),
    "slam-nav": ("slam_nav",),
    "all": (
        "core",
        "voice_frontend_readiness",
        "voice_endpoint",
        "voice_multi_command",
        "voice_offline_model",
        "cpp_action_client",
        "cpp_action_scheduler",
        "control_authority",
        "gazebo",
        "slam_nav",
    ),
}


def _terminate_process_group(process: subprocess.Popen[str]) -> None:
    """终止一个验收 mode 及其 launch 子树，避免中断后留下 ROS/Gazebo。"""

    if process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            return
        process.wait(timeout=5.0)


def checks_for_profile(
    profile: str,
    *,
    skip_local_models: bool = False,
) -> tuple[VerificationCheck, ...]:
    selected = tuple(CHECKS[name] for name in PROFILE_CHECK_NAMES[profile])
    if skip_local_models:
        selected = tuple(
            check for check in selected if not check.requires_local_models
        )
    return selected


def _execute_check(
    check: VerificationCheck,
    workspace: Path,
    tail_lines: int,
) -> dict:
    command = [
        "bash",
        str(workspace / "scripts/acceptance_test.sh"),
        check.mode,
    ]
    started = time.perf_counter()
    output_tail: deque[str] = deque(maxlen=tail_lines)
    error = None
    process = subprocess.Popen(
        command,
        cwd=workspace,
        env={**os.environ, "WORKSPACE": str(workspace)},
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        bufsize=1,
        start_new_session=True,
    )

    def forward_output() -> None:
        """长时 SLAM 心跳必须即时可见，同时只在报告里保留有限尾部。"""

        assert process.stdout is not None
        for line in process.stdout:
            print(line, end="", flush=True)
            output_tail.append(line.rstrip("\r\n"))

    reader = threading.Thread(target=forward_output, daemon=True)
    reader.start()
    try:
        returncode = process.wait(timeout=check.timeout_s)
    except subprocess.TimeoutExpired:
        returncode = 124
        error = f"timeout after {check.timeout_s:.0f}s"
        # 一个 mode 可能拥有 launch 的多级子进程；只终止父进程会留下 Gazebo
        # 和 DDS graph，污染下一项验收，因此以独立进程组为边界收口。
        _terminate_process_group(process)
    except BaseException:
        # start_new_session 隔离了 mode 的信号域；若用户 Ctrl-C，父进程必须显式
        # 回收整棵子树，否则长时 Gazebo 会在后台继续运行并占用内存。
        _terminate_process_group(process)
        raise
    finally:
        reader.join(timeout=5.0)
        if process.stdout is not None:
            process.stdout.close()

    return {
        **asdict(check),
        "command": command,
        "passed": returncode == 0,
        "returncode": returncode,
        "duration_s": round(time.perf_counter() - started, 2),
        "output_tail": list(output_tail),
        "output_streamed": True,
        "error": error,
    }


CheckExecutor = Callable[[VerificationCheck, Path, int], dict]


def run(
    profile: str,
    *,
    workspace: Path = ROOT,
    report_path: Path | None = None,
    skip_local_models: bool = False,
    keep_going: bool = False,
    tail_lines: int = 40,
    execute_check: CheckExecutor = _execute_check,
) -> int:
    """运行一个验收 profile；默认失败即停，防止残留 ROS 图污染后续步骤。"""

    workspace = workspace.resolve()
    selected = checks_for_profile(
        profile,
        skip_local_models=skip_local_models,
    )
    session_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    session_id = f"{session_id}-{os.getpid()}"
    if report_path is None:
        report_path = (
            workspace
            / "logs/acceptance/project_verification"
            / session_id
            / "report.json"
        )
    report_path = report_path.resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)

    started_at = datetime.now(timezone.utc).isoformat()
    results: list[dict] = []
    print(
        f"[verify] profile={profile} checks={len(selected)} "
        "live_microphone=false",
        flush=True,
    )
    for index, check in enumerate(selected, start=1):
        print(
            f"[verify][{index}/{len(selected)}] RUN {check.name}: {check.evidence}",
            flush=True,
        )
        result = execute_check(check, workspace, tail_lines)
        results.append(result)
        state = "PASS" if result["passed"] else "FAIL"
        print(
            f"[verify][{index}/{len(selected)}] {state} {check.name} "
            f"elapsed={result['duration_s']}s",
            flush=True,
        )
        if not result["passed"]:
            if not result.get("output_streamed", False):
                for line in result["output_tail"]:
                    print(line, flush=True)
            if not keep_going:
                break

    completed_names = {result["name"] for result in results}
    not_run = [check.name for check in selected if check.name not in completed_names]
    passed = len(results) == len(selected) and all(
        result["passed"] for result in results
    )
    report = {
        "schema_version": 1,
        "scenario": "project_key_feature_verification",
        "profile": profile,
        "session_id": session_id,
        "started_at": started_at,
        "finished_at": datetime.now(timezone.utc).isoformat(),
        "live_microphone_used": False,
        "local_model_checks_skipped": skip_local_models,
        "passed": passed,
        "checks": results,
        "not_run": not_run,
    }
    report_path.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(f"[verify] report={report_path}", flush=True)
    print("PASS: project key features" if passed else "FAIL: project key features")
    return 0 if passed else 1


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "验收项目关键功能；真人麦克风不在本命令范围内。"
        )
    )
    parser.add_argument(
        "profile",
        nargs="?",
        default="all",
        choices=tuple(PROFILE_CHECK_NAMES),
    )
    parser.add_argument("--report", type=Path)
    parser.add_argument(
        "--skip-local-models",
        action="store_true",
        help="跳过需要本地模型的确定性音频离线链路。",
    )
    parser.add_argument(
        "--keep-going",
        action="store_true",
        help="失败后继续后续步骤；默认失败即停以防 ROS 残留污染。",
    )
    parser.add_argument("--tail-lines", type=int, default=40)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    return run(
        args.profile,
        report_path=args.report,
        skip_local_models=args.skip_local_models,
        keep_going=args.keep_going,
        tail_lines=max(1, args.tail_lines),
    )


if __name__ == "__main__":
    raise SystemExit(main())
