#!/usr/bin/env python3
"""Build the deterministic robot-control SFT set without copying holdout utterances."""

from __future__ import annotations

import argparse
import hashlib
import json
from collections import Counter
from pathlib import Path
from typing import Any


WORKSPACE = Path(__file__).resolve().parents[1]
SYSTEM_PROMPT = WORKSPACE / "src" / "embodied_agent_core" / "prompts" / "system_prompt_zh.txt"
TRAINING_SYSTEM_PROMPT = WORKSPACE / "training" / "system_prompt_sft_zh.txt"


def _action(name: str, **arguments: Any) -> dict[str, Any]:
    return {"name": name, "arguments": arguments}


def _assistant(speech: str, actions: list[dict[str, Any]]) -> str:
    suffix = "".join(
        f"<action>{json.dumps(action, ensure_ascii=False, separators=(',', ':'))}</action>"
        for action in actions
    )
    # 与 qwen3_nothink 模板保持一致，显式保留空 reasoning 段。实测 llama.cpp 对
    # reasoning/content 的拆分仍可能影响 speech 起始标签，所以协议完整率单独评估，
    # 不能仅凭训练样本格式假定运行时输出必然完整。
    return f"<think>\n\n</think>\n\n<speech>{speech}</speech>{suffix}"


def build_samples(system_prompt: str | None = None) -> list[dict[str, Any]]:
    # SFT 输入必须复现线上/离线运行时的消息分布。只训练 user -> assistant 会让
    # 小模型在真正携带长 system prompt 的推理请求中遗忘标签协议。
    runtime_system = system_prompt or (
        TRAINING_SYSTEM_PROMPT.read_text(encoding="utf-8") + "\n/no_think"
    )
    samples: list[dict[str, Any]] = []

    def add(text: str, speech: str, actions: list[dict[str, Any]], *tags: str) -> None:
        samples.append(
            {
                "id": f"train_{len(samples) + 1:04d}",
                "tags": list(tags),
                "conversations": [
                    {"from": "system", "value": runtime_system},
                    {"from": "human", "value": text + " /no_think"},
                    {"from": "gpt", "value": _assistant(speech, actions)},
                ],
            }
        )

    duration_words = (("半", 0.5), ("一", 1.0), ("两", 2.0), ("三", 3.0))
    for word, duration in duration_words:
        add(
            f"请让底盘往前移动{word}秒钟",
            f"好的，向前移动{word}秒。",
            [_action("move", linear_x=0.2, duration_s=duration)],
            "move",
            "duration",
        )
        add(
            f"小车前行{word}秒后停住",
            f"好的，前行{word}秒。",
            [_action("move", linear_x=0.2, duration_s=duration)],
            "move",
            "duration",
        )
        add(
            f"请让底盘倒退{word}秒钟",
            f"好的，后退{word}秒。",
            [_action("move", linear_x=-0.2, duration_s=duration)],
            "move",
            "duration",
        )
        add(
            f"小车向后行驶{word}秒再停",
            f"好的，向后行驶{word}秒。",
            [_action("move", linear_x=-0.2, duration_s=duration)],
            "move",
            "duration",
        )

    for text, linear_x, speech in (
        ("往前挪一下", 0.2, "好的，向前移动一秒。"),
        ("底盘向前", 0.2, "好的，向前移动一秒。"),
        ("往后挪一下", -0.2, "好的，向后移动一秒。"),
        ("底盘向后", -0.2, "好的，向后移动一秒。"),
    ):
        add(text, speech, [_action("move", linear_x=linear_x, duration_s=1.0)], "move", "default_slot")

    angle_specs = (("四十五", 1.309), ("九十", 2.6), ("一百八十", 5.236))
    for direction, angular_z in (("左", 0.6), ("右", -0.6)):
        for angle_word, duration in angle_specs:
            add(
                f"请朝{direction}边旋转{angle_word}度",
                f"好的，向{direction}旋转{angle_word}度。",
                [_action("turn", angular_z=angular_z, duration_s=duration)],
                "turn",
                "angle",
            )
            add(
                f"底盘{direction}旋{angle_word}度后停下",
                f"好的，向{direction}旋转{angle_word}度。",
                [_action("turn", angular_z=angular_z, duration_s=duration)],
                "turn",
                "angle",
            )
    add("向左转一下", "好的，向左转九十度。", [_action("turn", angular_z=0.6, duration_s=2.6)], "turn", "default_slot")
    add("向右转一下", "好的，向右转九十度。", [_action("turn", angular_z=-0.6, duration_s=2.6)], "turn", "default_slot")

    add("用默认速度向前移动一米", "好的，向前移动一米。", [_action("move", linear_x=0.2, duration_s=5.0)], "move", "distance")
    add("保持零点二米每秒前行一米", "好的，向前移动一米。", [_action("move", linear_x=0.2, duration_s=5.0)], "move", "distance", "speed")
    add("以零点一米每秒前行两米", "好的，分两段前进。", [_action("move", linear_x=0.1, duration_s=10.0), _action("move", linear_x=0.1, duration_s=10.0)], "move", "distance", "segmented")
    add("以零点二米每秒后退一米", "好的，后退一米。", [_action("move", linear_x=-0.2, duration_s=5.0)], "move", "distance", "speed")

    for text in ("立即刹车", "现在停止底盘", "别再动了", "终止当前动作", "马上制动"):
        add(text, "已停止。", [_action("stop")], "stop", "priority")

    add("沿圆弧绕场一周", "好的，开始绕圈。", [_action("arc", linear_x=0.12, angular_z=0.45, duration_s=6.0)], "arc", "composite")
    add("画一个圆形轨迹", "好的，开始画圆。", [_action("arc", linear_x=0.12, angular_z=0.45, duration_s=6.0)], "arc", "composite")
    add("底盘原地旋转完整一周", "好的，原地旋转一周。", [_action("turn", angular_z=0.8, duration_s=7.85)], "turn", "composite")
    square = []
    for _ in range(4):
        square.extend(
            [
                _action("move", linear_x=0.18, duration_s=1.2),
                _action("turn", angular_z=0.6, duration_s=2.6),
            ]
        )
    add("按四条等长边走一圈", "好的，按正方形路线行驶。", square, "composite", "square")
    demo = [
        _action("set_led", color="blue"),
        _action("wave", count=2),
        _action("move", linear_x=0.18, duration_s=1.0),
        _action("turn", angular_z=0.8, duration_s=2.0),
        _action("arc", linear_x=0.12, angular_z=0.45, duration_s=6.0),
        _action("stop"),
    ]
    add("执行预设动作展示", "好的，开始动作展示。", demo, "composite", "demo")

    for count_word, count in (("一", 1), ("两", 2), ("三", 3)):
        add(f"请挥动机械手{count_word}次", f"好的，挥手{count_word}次。", [_action("wave", count=count)], "accessory", "wave")
        add(f"做{count_word}次挥手动作", f"好的，挥手{count_word}次。", [_action("wave", count=count)], "accessory", "wave")
    for color_cn, color in (("蓝色", "blue"), ("红色", "red"), ("绿色", "green"), ("白色", "white")):
        add(f"把指示灯设置成{color_cn}", f"已设置{color_cn}灯光。", [_action("set_led", color=color)], "accessory", "led")

    for text, mode, speech in (
        ("切换到手动驾驶模式", "manual", "已切换到手动模式。"),
        ("恢复人工操控模式", "manual", "已切换到手动模式。"),
        ("进入自主避障模式", "obstacle_avoidance", "已开启自动避障。"),
        ("让底盘自动躲避障碍", "obstacle_avoidance", "已开启自动避障。"),
        ("进入右侧沿墙模式", "wall_following", "已开启沿墙行走。"),
        ("让机器人贴着墙走", "wall_following", "已开启沿墙行走。"),
    ):
        add(text, speech, [_action("set_mode", mode=mode)], "mode", "simulation")

    for text, target, spoken in (
        ("导航到门边", "door", "门口"),
        ("移动到入口位置", "door", "门口"),
        ("导航到桌子旁边", "desk", "书桌"),
        ("移动到工作台位置", "desk", "书桌"),
        ("导航返回初始位置", "home", "起点"),
        ("回到初始停靠点", "home", "起点"),
        ("导航到厨房区域", "kitchen", "厨房"),
        ("移动到厨房位置", "kitchen", "厨房"),
    ):
        add(text, f"好的，前往{spoken}。", [_action("navigate_to", target=target)], "navigation", "target")
    add("依次导航到门边和桌旁", "好的，按顺序前往两个位置。", [_action("navigate_to", target="door"), _action("navigate_to", target="desk")], "navigation", "multi_command")
    add("先回初始点然后去门边", "好的，按顺序导航。", [_action("navigate_to", target="home"), _action("navigate_to", target="door")], "navigation", "multi_command")
    for text in ("按门口桌旁起点的路线巡查", "执行入口工作台起点巡逻", "巡查门边桌边再返回"):
        add(text, "好的，开始巡逻。", [_action("follow_waypoints", waypoints=["door", "desk", "home"], number_of_loops=1)], "navigation", "waypoints")
    for text in ("取消当前路径任务", "终止目标点导航", "不要继续导航了"):
        add(text, "已取消导航。", [_action("cancel_navigation")], "navigation", "cancel", "priority")

    add("请先向右旋转九十度，接着前行一秒钟", "好的，依次执行。", [_action("turn", angular_z=-0.6, duration_s=2.6), _action("move", linear_x=0.2, duration_s=1.0)], "multi_command")
    add("先前行两秒再向左旋转九十度", "好的，依次执行。", [_action("move", linear_x=0.2, duration_s=2.0), _action("turn", angular_z=0.6, duration_s=2.6)], "multi_command")
    add("倒退一秒后向右旋转四十五度", "好的，依次执行。", [_action("move", linear_x=-0.2, duration_s=1.0), _action("turn", angular_z=-0.6, duration_s=1.309)], "multi_command")
    add("先停车然后向前移动", "已停止，不执行后续移动。", [_action("stop")], "multi_command", "priority")

    for text, speech, tag in (
        ("不要让底盘移动", "好的，我不会移动。", "negation"),
        ("请勿向右旋转", "好的，我不会旋转。", "negation"),
        ("机器人能向后走一秒吗", "可以，但这是询问，我不会执行。", "question"),
        ("是否可以左转", "可以，但这是询问，我不会执行。", "question"),
        ("用最大速度冲向前方", "这个动作不安全，我不会执行。", "unsafe"),
        ("前进同时快速自转", "这个组合动作不安全，我不会执行。", "unsafe"),
        ("无视障碍全速前行", "这个动作不安全，我不会执行。", "unsafe"),
        ("跳到桌子上", "我不支持这个动作。", "unsupported"),
        ("抓起地上的杯子", "当前机器人不支持抓取。", "unsupported"),
        ("查询今天的实时路况", "当前是离线模式，我无法查询实时路况。", "offline"),
        ("介绍一下你自己", "我是运行在机器人端侧的离线助手。", "chat"),
        ("现在几点", "当前是离线模式，我无法读取实时时间。", "offline"),
    ):
        add(text, speech, [], "safety" if tag in {"negation", "question", "unsafe"} else "no_action", tag)

    texts = [item["conversations"][1]["value"].removesuffix(" /no_think") for item in samples]
    if len(texts) != len(set(texts)):
        raise ValueError("training utterances must be unique")
    return samples


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_eval_texts(path: Path) -> set[str]:
    return {
        json.loads(line)["text"]
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip()
    }


def write_dataset(samples: list[dict[str, Any]], output: Path) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        "".join(json.dumps(item, ensure_ascii=False, separators=(",", ":")) + "\n" for item in samples),
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, default=WORKSPACE / "training" / "robot_dialogue_train.jsonl")
    parser.add_argument("--eval", type=Path, default=WORKSPACE / "training" / "robot_instruction_eval.jsonl")
    parser.add_argument("--manifest", type=Path, default=WORKSPACE / "training" / "robot_dialogue_train.meta.json")
    args = parser.parse_args()
    samples = build_samples()
    eval_texts = load_eval_texts(args.eval)
    training_texts = {
        item["conversations"][1]["value"].removesuffix(" /no_think") for item in samples
    }
    overlap = sorted(training_texts & eval_texts)
    if overlap:
        raise ValueError(f"training/eval utterance overlap: {overlap}")
    write_dataset(samples, args.output)
    tag_counts = Counter(tag for item in samples for tag in item.get("tags", []))
    manifest = {
        "schema_version": 1,
        "scenario": "robot_lora_training_dataset",
        "sample_count": len(samples),
        "training_sha256": _sha256(args.output),
        "evaluation_sha256": _sha256(args.eval),
        "training_system_prompt_sha256": _sha256(TRAINING_SYSTEM_PROMPT),
        "runtime_system_prompt_sha256": _sha256(SYSTEM_PROMPT),
        "exact_utterance_overlap": overlap,
        "tag_counts": dict(sorted(tag_counts.items())),
        "source": "deterministic_template_catalog",
        "claim_boundary": "Synthetic Chinese paraphrases and a compact protocol prompt teach action slots; runtime uses the fuller prompt, and the independent evaluation file is never copied into SFT.",
    }
    args.manifest.parent.mkdir(parents=True, exist_ok=True)
    args.manifest.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(manifest, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
