from __future__ import annotations

import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / "scripts" / "build_robot_lora_dataset.py"
SPEC = importlib.util.spec_from_file_location("build_robot_lora_dataset", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_training_catalog_is_large_unique_and_disjoint_from_holdout() -> None:
    samples = MODULE.build_samples()
    texts = [
        item["conversations"][1]["value"].removesuffix(" /no_think") for item in samples
    ]
    eval_texts = MODULE.load_eval_texts(ROOT / "training" / "robot_instruction_eval.jsonl")
    assert len(samples) >= 80
    assert len(texts) == len(set(texts))
    assert not (set(texts) & eval_texts)


def test_training_responses_use_closed_tags_and_object_actions() -> None:
    samples = MODULE.build_samples()
    action_count = 0
    for item in samples:
        assert item["conversations"][0]["from"] == "system"
        assert item["conversations"][0]["value"].endswith("\n/no_think")
        assert item["conversations"][1]["from"] == "human"
        assert item["conversations"][1]["value"].endswith(" /no_think")
        response = item["conversations"][2]["value"]
        assert response.startswith("<think>\n\n</think>\n\n<speech>")
        assert "</speech>" in response
        assert response.count("<action>") == response.count("</action>")
        for payload in response.split("<action>")[1:]:
            action = json.loads(payload.split("</action>", 1)[0])
            assert isinstance(action, dict)
            assert set(action) == {"name", "arguments"}
            action_count += 1
    assert action_count >= 70


def test_dataset_info_registers_generated_training_set() -> None:
    info = json.loads((ROOT / "training" / "dataset_info.json").read_text(encoding="utf-8"))
    assert info["robot_dialogue_train"]["file_name"] == "robot_dialogue_train.jsonl"
