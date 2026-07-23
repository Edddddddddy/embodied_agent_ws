#!/usr/bin/env python3
"""Export the lightweight command NLU prototype model.

当前 NLU 使用字符 n-gram 原型分类器，训练产物是一个 JSON 原型表。
脚本保留为后续加入真实 ASR 样本时的固定入口，避免把数据直接散落在代码里。
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path


DEFAULT_INTENTS = {
    "move_forward": ["向前走一秒", "前进一秒", "往前走", "向前", "前进"],
    "move_backward": ["后退一秒", "向后退", "往后走", "后退", "向后"],
    "turn_left": ["左转九十度", "向左转", "左转", "左转90度"],
    "turn_right": ["右转九十度", "向右转", "右转", "右转90度"],
    "stop": ["停下", "停止", "急停", "刹车", "别动"],
    "arc": ["绕圈", "画圆", "转圈"],
    "spin": ["原地转一圈", "旋转一圈", "转一圈"],
    "square": ["走正方形", "正方形", "方形巡游"],
    "demo": ["演示一下", "做个演示", "展示一下"],
    "wave": ["挥手", "挥手三次"],
    "set_led": ["开蓝灯", "把灯设为蓝色", "关灯"],
    "set_mode": ["开启自动避障", "开始沿墙行走", "退出自动模式"],
}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--output",
        default="src/embodied_agent_core/config/command_nlu_zh.json",
        help="Path to write the command NLU JSON model.",
    )
    args = parser.parse_args()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            {
                "version": 1,
                "description": "Character n-gram prototypes for lightweight Chinese robot command NLU.",
                "intents": DEFAULT_INTENTS,
            },
            ensure_ascii=False,
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    print(f"WROTE: {output}")


if __name__ == "__main__":
    main()
