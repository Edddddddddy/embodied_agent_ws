# 训练占位说明

当前只提供格式可检查的种子数据和 LLaMA-Factory LoRA 配置，不执行训练。8 条样本仅用于打通格式，不能支撑“85% 指令遵循率”。正式训练前应扩充为训练/验证/测试互斥的数据集，并单独构建动作 schema、拒绝危险动作、多轮对话和无动作闲聊的评测集。

离线 LLM 评估失败样例可以先导出成候选集：

```bash
bash scripts/acceptance_test.sh instruction-following-eval
bash scripts/acceptance_test.sh instruction-following-lora-candidates
```

这会生成：

- `training/robot_dialogue_lora_candidates.jsonl`
- `training/robot_dialogue_lora_candidates.meta.json`

候选集是 LLaMA-Factory ShareGPT 格式，但它只是“待审核数据”。请先检查
`meta.json` 中的 `skipped_cases`、`failure_type_counts` 和每条样本的 `metadata.raw_output`，
再决定是否合并到正式训练集。不要把候选集生成当作“已经完成 LoRA 训练”的证据。

候选集审核分两步执行：

```bash
INSTRUCTION_FOLLOWING_LORA_REVIEW_INIT=true \
  bash scripts/acceptance_test.sh instruction-following-lora-review

# 人工编辑 training/robot_dialogue_lora_review.json：
# - approved：可信样例，允许导出到 LoRA 训练集
# - rejected：脏样本或重复样本，不训练
# - needs_edit：需要改写答案或动作 schema
# - needs_review：尚未审核，默认不训练

bash scripts/acceptance_test.sh instruction-following-lora-review
```

这会在存在 `approved` 样例时生成：

- `training/robot_dialogue_lora_review.json`
- `training/robot_dialogue_lora_approved.jsonl`
- `training/robot_dialogue_lora_approved.meta.json`

`robot_dialogue_lora_approved.jsonl` 已注册到 `dataset_info.json`，但训练配置默认仍使用
`robot_dialogue_seed`。只有当审核完成且 approved 样例质量确认后，才建议把训练配置改为：

```yaml
dataset: robot_dialogue_seed,robot_dialogue_lora_approved
```

运行训练前，将本目录的 `dataset_info.json` 合并到 LLaMA-Factory 的数据目录，然后执行：

```bash
llamafactory-cli train training/qwen3_0_6b_lora.yaml
```

训练后先合并 LoRA，再用 llama.cpp 转换为 F16 GGUF，最后运行 `scripts/quantize_qwen_q8.sh`。注意：Q8_0 从 FP16 权重通常约压缩一半，并不等于“压缩至 25%”；体积比例必须用实际文件大小计算。
