#!/usr/bin/env python3
"""Validate one deployable voice runtime profile without running inference."""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
CORE = ROOT / "src" / "embodied_agent_core"
if str(CORE) not in sys.path:
    sys.path.insert(0, str(CORE))

from embodied_agent_core.voice_runtime_deployment import (  # noqa: E402
    VoiceRuntimeDeploymentChecker,
    VoiceRuntimeProfileError,
)


def _runtime_root(workspace: Path) -> Path:
    """Linked worktree 复用主工作区的大模型目录，避免每个 feature 分支复制权重。"""

    try:
        completed = subprocess.run(
            ["git", "-C", str(workspace), "rev-parse", "--git-common-dir"],
            text=True,
            capture_output=True,
            check=True,
        )
        common = Path(completed.stdout.strip()).expanduser().resolve()
        if common.name == ".git":
            return common.parent
    except (OSError, subprocess.CalledProcessError):
        pass
    return workspace


def load_profiles(path: Path) -> dict:
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    profiles = raw.get("profiles")
    if not isinstance(profiles, dict):
        raise VoiceRuntimeProfileError("profile file must contain a profiles mapping")
    return profiles


def format_report(report) -> str:
    lines = [
        f"{report.status.upper()}: voice runtime preflight",
        f"  profile: {report.profile}",
        f"  contract_only: {str(report.contract_only).lower()}",
    ]
    for component in report.components:
        provider = component.selected_provider or "-"
        lines.append(
            f"  {component.name}: {component.status} provider={provider}"
        )
        for warning in component.warnings:
            lines.append(f"    WARN {warning}")
        for blocker in component.blockers:
            lines.append(f"    BLOCKER {blocker}")
    lines.append(
        "  note: no model inference, download, or paid API request was performed"
    )
    return "\n".join(lines)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description="Preflight VAD/ASR/LLM/RAG/TTS deployment profiles"
    )
    parser.add_argument(
        "--profile",
        required=True,
        choices=("offline-edge", "online-cloud"),
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=ROOT / "config" / "voice_runtime_profiles.yaml",
    )
    parser.add_argument("--workspace", type=Path, default=ROOT)
    parser.add_argument("--runtime-root", type=Path)
    parser.add_argument(
        "--probe-endpoints",
        action="store_true",
        help="Perform read-only health GETs; never sends an inference request.",
    )
    parser.add_argument(
        "--contract-only",
        action="store_true",
        help="Validate profile schema without requiring local models or API keys.",
    )
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    try:
        profiles = load_profiles(args.config)
        profile = profiles.get(args.profile)
        if not isinstance(profile, dict):
            raise VoiceRuntimeProfileError(
                f"profile not found in {args.config}: {args.profile}"
            )
        workspace = args.workspace.expanduser().resolve()
        runtime_root = (
            args.runtime_root.expanduser().resolve()
            if args.runtime_root
            else _runtime_root(workspace)
        )
        report = VoiceRuntimeDeploymentChecker().check(
            args.profile,
            profile,
            variables={
                "WORKSPACE": str(workspace),
                "RUNTIME_ROOT": str(runtime_root),
            },
            probe_endpoints=args.probe_endpoints,
            contract_only=args.contract_only,
        )
    except (OSError, ValueError, VoiceRuntimeProfileError) as exc:
        parser.error(str(exc))

    print(
        json.dumps(report.as_dict(), ensure_ascii=False, indent=2)
        if args.json
        else format_report(report)
    )
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
