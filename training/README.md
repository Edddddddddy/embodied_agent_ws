# Qwen3-0.6B LoRA、GGUF 与 Q8 流水线

本目录提供一条固定入口的可复现流水线：

```text
ShareGPT 数据 → LLaMA-Factory LoRA SFT → 合并 HF 模型
→ llama.cpp F16 GGUF → Q8_0 → 产物与体积审计
```

## 1. 数据边界

- `robot_dialogue_seed.jsonl` 只有 8 条，用于验证格式和流水线，不支撑准确率声明。
- `robot_dialogue_lora_candidates.jsonl` 来自离线模型失败样例，默认不能直接训练。
- 候选样例必须经 `robot_dialogue_lora_review.json` 人工标注后，才能导出
  `robot_dialogue_lora_approved.jsonl`。
- 训练、验证、测试数据应互斥；`robot_instruction_eval.jsonl` 只用于评估，不回流训练。

候选生成与审核：

```bash
bash scripts/acceptance_test.sh instruction-following-eval
bash scripts/acceptance_test.sh instruction-following-lora-candidates
INSTRUCTION_FOLLOWING_LORA_REVIEW_INIT=true \
  bash scripts/acceptance_test.sh instruction-following-lora-review
# 编辑 training/robot_dialogue_lora_review.json 后：
bash scripts/acceptance_test.sh instruction-following-lora-review
```

## 2. 固定工具链

```bash
bash scripts/setup_lora_toolchain.sh --dry-run
bash scripts/setup_lora_toolchain.sh
```

LLaMA-Factory 固定到 commit
`ea31c43d806162a7fd98065abfef2d974fff5766`；llama.cpp 版本见
[OFFLINE_RUNTIME_VERSIONS.md](../docs/OFFLINE_RUNTIME_VERSIONS.md)。

## 3. Dry-run 与正式执行

```bash
# 查看五个阶段命令并生成非严格审计报告
bash scripts/acceptance_test.sh lora-q8-pipeline

# 真正执行训练、合并、转换、Q8_0 量化和严格审计
bash scripts/build_qwen_lora_q8.sh --execute
```

配置与产物：

- `qwen3_0_6b_lora.yaml`：LoRA SFT。
- `qwen3_0_6b_lora_merge.yaml`：合并 adapter 到 HF 模型。
- `outputs/qwen3-0.6b-robot-lora/`：adapter 与 trainer state。
- `outputs/qwen3-0.6b-robot-merged/`：合并后的 HF 模型。
- `outputs/qwen3-0.6b-robot-f16.gguf`：F16 基准产物。
- `models/Qwen3-0.6B-robot-Q8_0.gguf`：Q8_0 产物。
- `logs/lora_q8_pipeline_report.json`：实际文件大小、比例和证据状态。

## 4. 结果声明规则

审计状态分三类：

- `pipeline_ready_not_executed`：脚本/配置就绪，没有产物。
- `artifacts_verified_training_unproven`：有合法 GGUF，但缺 trainer state，不能证明来自本次训练。
- `training_and_artifacts_verified`：adapter 训练证据与 F16/Q8 GGUF 均存在。

Q8_0 相对 FP16 通常接近一半体积，不能写成“压缩至 25%”，除非报告中的实际文件
大小确实支持该结论。模型指令准确率也必须由独立评估集实测，不能从量化完成反推。
