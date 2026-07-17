# Offline 证据

- [lora_q8_instruction_comparison.md](lora_q8_instruction_comparison.md) / `.json`：原始模型、LoRA/Q8 与工程 fallback 的同口径对照。
- `logs/offline_latency_report.json`：llama.cpp 与 TTS 子指标。
- `logs/offline_voice_e2e_report.json`：真实离线语音链路。

没有对应运行报告时，不引用固定准确率、tokens/s、压缩率或整链路时延。模型文件和第三方运行时不提交 Git。
