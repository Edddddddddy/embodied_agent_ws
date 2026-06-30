# 训练占位说明

当前只提供格式可检查的种子数据和 LLaMA-Factory LoRA 配置，不执行训练。8 条样本仅用于打通格式，不能支撑“85% 指令遵循率”。正式训练前应扩充为训练/验证/测试互斥的数据集，并单独构建动作 schema、拒绝危险动作、多轮对话和无动作闲聊的评测集。

运行训练前，将本目录的 `dataset_info.json` 合并到 LLaMA-Factory 的数据目录，然后执行：

```bash
llamafactory-cli train training/qwen3_0_6b_lora.yaml
```

训练后先合并 LoRA，再用 llama.cpp 转换为 F16 GGUF，最后运行 `scripts/quantize_qwen_q8.sh`。注意：Q8_0 从 FP16 权重通常约压缩一半，并不等于“压缩至 25%”；体积比例必须用实际文件大小计算。
