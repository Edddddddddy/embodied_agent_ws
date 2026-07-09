# 离线模型 Benchmark 报告模板

本文档用于把“端侧部署能力”从项目叙述变成可复查证据。每次更换 GGUF、ASR、TTS
或运行设备后，建议重新填写一版。

## 1. 环境信息

| 项目 | 记录 |
| --- | --- |
| 日期 | TODO |
| 机器/CPU | TODO |
| OS / ROS 2 | Ubuntu 24.04 / ROS 2 Jazzy |
| llama.cpp commit | 见 `docs/OFFLINE_RUNTIME_VERSIONS.md` |
| GGUF 模型 | `models/Qwen3-0.6B-Q8_0.gguf` |
| ASR 模型 | Sherpa-ONNX ZipFormer |
| TTS 模型 | Sherpa-TTS 默认；SummerTTS 作为 C++ runtime 展示 |

## 2. 必跑命令

```bash
bash scripts/acceptance_test.sh offline-runtime-versions
bash scripts/acceptance_test.sh llama-cpp-preflight
bash scripts/acceptance_test.sh llama-cpp-smoke
bash scripts/acceptance_test.sh offline-latency
bash scripts/benchmark_offline.sh
bash scripts/evaluate_instruction_following.sh --minimum 0.70
```

## 3. 指标记录

| 指标 | 目标 | 实测 | 证据命令 |
| --- | --- | --- | --- |
| LLM 首 token | ≤ 1000ms | TODO | `offline-latency` |
| 默认 TTS 首音频 | ≤ 300ms | TODO | `offline-latency` |
| llama.cpp tokens/s | 记录即可 | TODO | `/offline_agent/metrics` 或 `llama-cpp-smoke` |
| ASR realtime factor | < 1.0 更好 | TODO | `benchmark_offline.sh` |
| TTS realtime factor | < 1.0 更好 | TODO | `benchmark_offline.sh` |
| 指令动作准确率 | ≥ 70% 起步 | TODO | `evaluate_instruction_following.sh` |

## 4. 错误样例回归

错误样例应补进 `training/robot_instruction_eval.jsonl`，字段建议：

```json
{"id":"asr_tail_001","text":"左转九十度","expected_actions":[{"name":"turn","arguments":{"angular_z":0.6,"duration_s":2.6}}],"tags":["turn","tail_number"]}
```

新增样例后至少跑：

```bash
python3 scripts/validate_instruction_eval_dataset.py
```

## 5. 当前结论模板

```text
本轮离线链路可以支撑演示：llama.cpp 首 token 达到目标，Sherpa-TTS 首音频达到目标；
SummerTTS 已完成服务化封装，但 CPU 合成仍慢，不作为默认低延迟 TTS。
主要不足是 LoRA 微调和更大规模动作准确率评估尚未完成。
```
