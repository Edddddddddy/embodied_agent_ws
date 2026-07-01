#!/usr/bin/env python3
import argparse
import json
from pathlib import Path

from embodied_offline_agent.providers.llama_cpp import LlamaCppLlm
from embodied_online_agent.protocol import TaggedStreamParser
from embodied_online_agent.command_fallback import parse_fallback_action, should_block_model_actions


def parse_output(text):
    parser = TaggedStreamParser()
    events = parser.feed(text)
    final = parser.finish()
    return {
        "speech": "".join(events.speech + final.speech).strip(),
        "actions": [action.as_dict() for action in events.actions + final.actions],
        "errors": events.errors + final.errors,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--minimum", type=float)
    parser.add_argument("--workspace", default="/home/ubuntu/embodied_agent_ws")
    args = parser.parse_args()
    root = Path(args.workspace)
    system = (root / "src/embodied_online_agent/prompts/system_prompt_zh.txt").read_text()
    llm = LlamaCppLlm(
        "http://127.0.0.1:8080/v1", "Qwen3-0.6B-Q8_0.gguf", 0.7, 192
    )
    results = []
    for line in (root / "training/robot_dialogue_seed.jsonl").read_text().splitlines():
        sample = json.loads(line)["conversations"]
        user_text = sample[0]["value"]
        expected = parse_output(sample[1]["value"])
        output = "".join(llm.stream([
            {"role": "system", "content": system + "\n/no_think"},
            {"role": "user", "content": user_text + " /no_think"},
        ]))
        actual = parse_output(output)
        passed = bool(actual["speech"]) and not actual["errors"] and actual["actions"] == expected["actions"]
        fallback = parse_fallback_action(user_text)
        if fallback is not None:
            effective_actions = [fallback.as_dict()]
        elif should_block_model_actions(user_text):
            effective_actions = []
        else:
            effective_actions = list(actual["actions"])
        effective_passed = bool(actual["speech"]) and effective_actions == expected["actions"]
        results.append({
            "input": user_text,
            "passed": passed,
            "expected_actions": expected["actions"],
            "actual_actions": actual["actions"],
            "effective_actions": effective_actions,
            "effective_passed": effective_passed,
            "raw_output": output,
        })
    passed = sum(item["passed"] for item in results)
    score = passed / len(results)
    effective_passed = sum(item["effective_passed"] for item in results)
    report = {
        "model_passed": passed,
        "effective_passed": effective_passed,
        "total": len(results),
        "model_score": score,
        "effective_score": effective_passed / len(results),
        "cases": results,
    }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.minimum is not None and score < args.minimum:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
