"""SLAM/Nav2 重型探针的 ROS-free 命令行契约。"""

from __future__ import annotations

import argparse
from pathlib import Path


def build_parser() -> argparse.ArgumentParser:
    """集中拥有稳定 CLI；仓库测试无需安装 ROS 也能验证参数契约。"""

    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--transition-timeout", type=float, default=10.0)
    parser.add_argument("--evidence-kind", default="dry_run_process_adapter")
    parser.add_argument("--session-id", default="")
    parser.add_argument("--session-start-ns", type=int, default=0)
    parser.add_argument("--world-file", type=Path)
    parser.add_argument("--mission-plan", type=Path)
    parser.add_argument("--dynamic-scenario", type=Path)
    parser.add_argument("--dynamic-navigation-timeout", type=float, default=180.0)
    parser.add_argument("--runtime-log", type=Path)
    parser.add_argument("--gate-timeout-s", type=float, default=900.0)
    parser.add_argument("--progress-heartbeat-s", type=float, default=15.0)
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="终端打印验收摘要，完整字段仅写入 --output",
    )
    parser.add_argument("--explore-before-save", action="store_true")
    parser.add_argument(
        "--automatic-mission",
        action="store_true",
        help="一句系统意图触发自动探索、存图、定位切换和语义导航",
    )
    parser.add_argument(
        "--cancel-automatic-mission",
        action="store_true",
        help="在 dry-run 自动探索期间发送急停并验证恢复到 MAPPING",
    )
    parser.add_argument(
        "--survey-plan",
        type=Path,
        help="通过 Agent 顺序执行工作场景 mapping_route，并验证地图覆盖与里程",
    )
    return parser
