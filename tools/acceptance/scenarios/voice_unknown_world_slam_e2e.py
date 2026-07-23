#!/usr/bin/env python3
"""真人语音触发 full-evidence unknown-world SLAM/Nav2 联合验收入口。"""

from __future__ import annotations

import argparse
from collections.abc import Sequence

from tools.acceptance.scenarios.unknown_world_slam_e2e import (
    UnknownWorldRunProfile,
    run,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Live microphone -> ASR -> full-evidence unknown-world SLAM/Nav2 gate"
        )
    )
    parser.add_argument(
        "agent_mode",
        choices=("offline", "online"),
        help="Agent/ASR provider used by the live microphone stage",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    # wrapper 只选择在线/离线 profile；所有 SLAM/Nav2 安全门禁复用同一 runner，
    # 避免真人链路逐渐演变成一套更宽松、无法对照的验收实现。
    return run(UnknownWorldRunProfile.live_voice(args.agent_mode))


if __name__ == "__main__":
    raise SystemExit(main())
