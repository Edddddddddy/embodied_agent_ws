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
    parser.add_argument(
        "--unknown-world",
        action="store_true",
        help="启用无场景先验的 schema v4 地图/定位/动态目标证据门禁",
    )
    parser.add_argument(
        "--scene-spec",
        type=Path,
        help="仅供 evaluator 读取出生位姿与区域边界，不能传给机器人策略",
    )
    parser.add_argument(
        "--truth-map",
        type=Path,
        help="仅供 evaluator 计算覆盖率的静态真值地图",
    )
    parser.add_argument(
        "--gazebo-robot-entity",
        default="turtlebot3_waffle",
        help="Gazebo SceneBroadcaster 中的顶层机器人实体名",
    )
    parser.add_argument("--dynamic-navigation-timeout", type=float, default=180.0)
    parser.add_argument("--runtime-log", type=Path)
    parser.add_argument("--gate-timeout-s", type=float, default=900.0)
    parser.add_argument("--progress-heartbeat-s", type=float, default=15.0)
    parser.add_argument(
        "--summary-only",
        action="store_true",
        help="终端打印验收摘要，完整字段仅写入 --output",
    )
    parser.add_argument(
        "--require-runtime-continuity",
        action="store_true",
        help=(
            "要求 Gazebo、robot_state_publisher 与所选 online/offline Agent "
            "在建图、导航阶段保持同一 Linux 进程身份"
        ),
    )
    parser.add_argument(
        "--require-rviz-continuity",
        action="store_true",
        help="可视化演示同时要求 RViz 在阶段切换时不重启",
    )
    parser.add_argument("--explore-before-save", action="store_true")
    parser.add_argument(
        "--automatic-mission",
        action="store_true",
        help="一句系统意图触发自动探索、存图、定位切换和语义导航",
    )
    parser.add_argument(
        "--automatic-trigger-source",
        choices=("synthetic", "live_voice"),
        default="synthetic",
        help="自动任务由确定性测试文本或真人麦克风证据触发",
    )
    parser.add_argument(
        "--agent-mode",
        choices=("offline", "online"),
        default="offline",
        help="联合语音证据所对应的 Agent provider 模式",
    )
    parser.add_argument(
        "--voice-trigger-timeout",
        type=float,
        default=90.0,
        help="进入 MAPPING 后等待真人自动建图口令的秒数",
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
