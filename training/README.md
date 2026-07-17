# Qwen3-0.6B LoRA、GGUF 与 Q8 流水线

本目录提供一条固定入口的可复现流水线：

```text
ShareGPT 数据 → LLaMA-Factory LoRA SFT → 合并 HF 模型
→ llama.cpp F16 GGUF → Q8_0 → 产物与体积审计
```

## 1. 数据边界

- `robot_dialogue_train.jsonl` 由 `build_robot_lora_dataset.py` 确定性生成，共 96 条；
  manifest 绑定训练集、独立评估集和两份 system prompt 的 SHA256。
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
`ea31c43d806162a7fd98065abfef2d974fff5766`；llama.cpp、SummerTTS 与 Sherpa
版本由 `scripts/offline_runtime_versions.py` 和对应安装脚本统一检查，设计边界见
[语音 Agent 学习笔记](../docs/learning/VOICE_AGENT.md)。

## 3. Dry-run 与正式执行

```bash
# 查看五个阶段命令并生成非严格审计报告
bash scripts/acceptance_test.sh lora-q8-pipeline

# 真正执行训练、合并、转换、Q8_0 量化和严格审计
bash scripts/build_qwen_lora_q8.sh --execute

# 只重跑原始 Q8 与 LoRA Q8 的 43 条独立对照，不重新训练
bash scripts/acceptance_test.sh lora-q8-comparison
```

配置与产物：

- `qwen3_0_6b_lora.yaml`：LoRA SFT。
- `qwen3_0_6b_lora_merge.yaml`：合并 adapter 到 HF 模型。
- `outputs/qwen3-0.6b-robot-lora/`：adapter 与 trainer state。
- `outputs/qwen3-0.6b-robot-merged/`：合并后的 HF 模型。
- `outputs/qwen3-0.6b-robot-f16.gguf`：F16 基准产物。
- `models/Qwen3-0.6B-robot-Q8_0.gguf`：Q8_0 产物。
- `logs/lora_q8_pipeline_report.json`：实际文件大小、比例和证据状态。
- `docs/evidence/offline/lora_q8_instruction_comparison.{json,md}`：可提交的小型对照证据。

## 4. 结果声明规则

审计状态分三类：

- `pipeline_ready_not_executed`：脚本/配置就绪，没有产物。
- `artifacts_verified_training_unproven`：有合法 GGUF，但缺 trainer state，不能证明来自本次训练。
- `training_and_artifacts_verified_evaluation_unproven`：训练和产物存在，但缺同口径对照。
- `training_quantization_and_holdout_verified`：训练、产物、数据 manifest 和模型对照哈希全部通过。

本轮 F16 为 1,198,182,176 bytes，Q8_0 为 639,446,816 bytes，压缩 46.63%。独立 43 条
holdout 上，原始动作匹配由 30.23% 提升到 53.49%；严格协议 + 动作均正确仍为 25.58%，
fallback 工程出口为 83.72%。四个指标必须分开表述，不能用 fallback 冒充模型准确率，
也不能把合成 holdout 外推为真实麦克风生产成功率。
