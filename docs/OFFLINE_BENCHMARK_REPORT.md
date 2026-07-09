# 离线模型 Benchmark 与展示报告

本文档用于把“端侧部署能力”从项目叙述变成可复查证据。每次更换 GGUF、ASR、TTS
或运行设备后，建议重新生成一版报告。

快速生成默认报告：

```bash
bash scripts/acceptance_test.sh offline-showcase-report
bash scripts/acceptance_test.sh offline-evidence-audit
```

默认不会启动 llama.cpp 或真实 ASR/TTS benchmark，会输出：

```text
logs/offline_showcase_report.json
logs/offline_showcase_report.md
logs/offline_evidence_audit.json
```

`offline-evidence-audit` 会把报告分成 `blockers`、`warnings` 和 `claim_guidance`：
默认报告可以证明模型资产、运行时版本和 deterministic parser 评估；如果没有运行
`offline-latency` / `llama-decode-benchmark` 或对应 `--run-*` 参数前，它会提示不要宣称
首 token、首音频或 tokens/s 指标已在当前机器复现。

`offline_showcase_report.json` 还包含 `claim_evidence` 指标证据矩阵，用于把每项能力标成
`proven`、`missing`、`not_reproduced` 或 `not_default`。汇报时优先引用这张矩阵：
它能清楚说明 Q8 GGUF 模型资产和 deterministic parser 评估已经有证据，而 LoRA 训练、
llama.cpp tokens/s、离线 LLM 指令遵循准确率等仍需要单独 benchmark 或训练日志支撑。

演示前如果要补充真实延迟和 Sherpa ASR/TTS benchmark：

```bash
python3 scripts/generate_offline_showcase_report.py --run-latency --run-llama-bench --run-asr-tts
```

## 1. 环境信息

| 项目 | 记录 |
| --- | --- |
| 日期 | 见生成报告 `generated_at` |
| 机器/CPU | 现场演示机器；如需精确记录可附 `lscpu` 输出 |
| OS / ROS 2 | Ubuntu 24.04 / ROS 2 Jazzy |
| llama.cpp commit | 由 `scripts/offline_runtime_versions.py` 自动采集 |
| GGUF 模型 | `models/Qwen3-0.6B-Q8_0.gguf` |
| ASR 模型 | Sherpa-ONNX ZipFormer |
| TTS 模型 | Sherpa-TTS 默认；SummerTTS 作为 C++ runtime 展示 |

## 2. 必跑命令

```bash
bash scripts/acceptance_test.sh offline-runtime-versions
bash scripts/acceptance_test.sh offline-showcase-report
bash scripts/acceptance_test.sh offline-evidence-audit
bash scripts/acceptance_test.sh llama-cpp-preflight
bash scripts/acceptance_test.sh llama-cpp-smoke
bash scripts/acceptance_test.sh llama-decode-benchmark
bash scripts/acceptance_test.sh offline-latency
bash scripts/benchmark_offline.sh
bash scripts/acceptance_test.sh instruction-parser-eval
bash scripts/evaluate_instruction_following.sh --minimum 0.70
```

## 3. 指标记录

| 指标 | 目标 | 实测 | 证据命令 |
| --- | --- | --- | --- |
| 模型资产大小 | 记录即可 | 见 `logs/offline_showcase_report.md` | `offline-showcase-report` |
| 运行时版本 | 固定版本匹配 | 见 `logs/offline_showcase_report.md` | `offline-showcase-report` |
| 指标证据矩阵 | 区分可宣称/不可宣称 | 见 `claim_evidence` | `offline-showcase-report` + `offline-evidence-audit` |
| LLM 首 token | ≤ 1000ms | 见真实延迟报告 | `offline-latency` 或 `--run-latency` |
| 默认 TTS 首音频 | ≤ 300ms | 见真实延迟报告 | `offline-latency` 或 `--run-latency` |
| llama.cpp tokens/s | 记录即可 | 见 `logs/llama_decode_benchmark.json` 或 `claim_evidence` | `llama-decode-benchmark` 或 `--run-llama-bench` |
| ASR realtime factor | < 1.0 更好 | 见真实 benchmark | `benchmark_offline.sh` 或 `--run-asr-tts` |
| TTS realtime factor | < 1.0 更好 | 见真实 benchmark | `benchmark_offline.sh` 或 `--run-asr-tts` |
| deterministic parser 动作准确率 | ≥ 95% | 当前代表集 39/39（100%） | `instruction-parser-eval` |
| 离线 LLM 指令动作准确率 | ≥ 70% 起步 | TODO | `evaluate_instruction_following.sh` |

## 4. 错误样例回归

错误样例应补进 `training/robot_instruction_eval.jsonl`，字段建议：

```json
{"id":"asr_tail_001","text":"左转九十度","expected_actions":[{"name":"turn","arguments":{"angular_z":0.6,"duration_s":2.6}}],"tags":["turn","tail_number"]}
```

新增样例后至少跑：

```bash
python3 scripts/validate_instruction_eval_dataset.py
python3 scripts/evaluate_instruction_parser.py --minimum 0.95
```

当前评估集覆盖基础移动/转向、短命令补全、ASR 错词归一化、多命令队列、组合动作、
Nav2 目标点/巡航、附件/模式命令，以及否定、疑问和危险速度请求等安全拒绝样例。
`evaluate_instruction_parser.py` 会输出 `source_counts`、`tag_accuracy` 和 `failed_cases`，
便于把真实 ASR 错误持续沉淀成回归用例。

## 5. 当前结论模板

```text
本轮离线链路可以支撑工程演示：Q8 GGUF、Sherpa-ONNX、Sherpa-TTS/SummerTTS
模型资产和运行时版本可复查，deterministic parser 在当前代表集上通过评估。
如果已额外运行 offline-latency，则可以引用本机首 token/首音频实测值；否则不应宣称
这些低延迟指标已复现。SummerTTS 已完成服务化封装，但当前不作为默认低延迟 TTS。
主要不足是 LoRA 微调、llama.cpp tokens/s 和离线 LLM 指令遵循准确率仍需补 benchmark。
```
