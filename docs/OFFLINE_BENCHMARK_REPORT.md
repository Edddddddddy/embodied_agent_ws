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
同时，`benchmark_gap_plan` 会列出缺失指标对应的下一条可执行命令，便于按报告逐项补齐证据。

`offline_showcase_report.json` 还包含 `claim_evidence` 指标证据矩阵，用于把每项能力标成
`proven`、`missing`、`not_reproduced` 或 `not_default`。汇报时优先引用这张矩阵：
它能清楚说明 Q8 GGUF 模型资产和 deterministic parser 评估已经有证据，而 LoRA 训练、
llama.cpp tokens/s、离线 LLM 指令遵循准确率等仍需要单独 benchmark 或训练日志支撑。

演示前如果要补充真实延迟和 Sherpa ASR/TTS benchmark：

```bash
python3 scripts/generate_offline_showcase_report.py --run-latency --run-llama-bench --run-instruction-following --run-asr-tts --run-voice-e2e
```

## 1. 环境信息

| 项目 | 记录 |
| --- | --- |
| 日期 | 2026-07-10 |
| 机器/CPU | WSL2，Intel Core i5-14400F，8 核 / 16 线程，约 8GB RAM |
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
bash scripts/acceptance_test.sh offline-voice-e2e-report
bash scripts/benchmark_offline.sh
bash scripts/acceptance_test.sh instruction-parser-eval
bash scripts/acceptance_test.sh instruction-following-eval
```

## 3. 指标记录

| 指标 | 目标 | 实测 | 证据命令 |
| --- | --- | --- | --- |
| 模型资产大小 | 记录即可 | 见 `logs/offline_showcase_report.md` | `offline-showcase-report` |
| 运行时版本 | 固定版本匹配 | 见 `logs/offline_showcase_report.md` | `offline-showcase-report` |
| 指标证据矩阵 | 区分可宣称/不可宣称 | 见 `claim_evidence` | `offline-showcase-report` + `offline-evidence-audit` |
| LLM warm turn 首 token | ≤ 1000ms | P50 496.90ms，P95 544.79ms | `offline-latency` / `--run-latency` |
| Sherpa 短句整句合成 | ≤ 600ms | 178.94ms | `offline-latency` / `--run-latency` |
| llama.cpp CPU decode | ≥ 8.6 tokens/s | 16.4276 tokens/s（8 threads） | `llama-decode-benchmark` |
| ASR realtime factor | < 1.0 | 0.0620 | `benchmark_offline.sh` / `--run-asr-tts` |
| TTS realtime factor | < 1.0 | 0.9004 | `benchmark_offline.sh` / `--run-asr-tts` |
| 真实 Agent 端点→首 PCM | < 3500ms | 707.53ms | `offline-voice-e2e-report` / `--run-voice-e2e` |
| 真实 Agent LLM 首 token | ≤ 1000ms | 120.61ms | `offline-voice-e2e-report` |
| 伪流式首文本→首 PCM | 记录即可 | 315.08ms | `offline-voice-e2e-report` |
| 整轮完成 | ≤ 3500ms | 1428.57ms | `offline-voice-e2e-report` |
| deterministic parser 动作准确率 | ≥ 95% | 当前代表集 43/43（100%） | `instruction-parser-eval` |
| 离线 LLM 原始指令动作准确率 | ≥ 70% 起步 | 3/8，37.5%，未达标 | `instruction-following-eval` / `--run-instruction-following` |
| fallback/安全层后动作准确率 | ≥ 85% | 8/8，100% | 同上；不能冒充模型分数 |

`instruction-following-eval` 输出两个分数：

- `model_score`：只看离线 LLM 原始 `<speech>/<action>` 协议输出是否正确，适合判断模型本身是否需要 LoRA/提示词优化。
- `effective_score`：只看确定性 fallback 和安全层兜底后的动作序列是否正确，报告字段
  `effective_score_policy=action_only_after_fallback_and_safety`。它适合判断工程链路在演示动作域内的可用性，
  不代表模型本身已经学会了标签协议。

如果 `model_score` 低但 `effective_score` 高，应如实表述为“离线 LLM 原始指令遵循仍弱，
当前靠轻量 NLU/fallback/ActionGuard 保证演示动作稳定”，不要把 effective score 说成模型训练后准确率。

当 `model_score` 低于目标时，把失败样例导出成 LoRA 候选集：

```bash
bash scripts/acceptance_test.sh instruction-following-lora-candidates
```

导出的 `training/robot_dialogue_lora_candidates.jsonl` 仍需人工审核；它用于准备下一轮
LLaMA-Factory SFT/LoRA 数据，不代表训练已经完成。

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
这些低延迟指标已复现。如果已运行 llama-decode-benchmark 和 instruction-following-eval，
则可以引用本机 decode tokens/s、model_score 和 effective_score；否则仍只能说已有评估入口。
SummerTTS 已完成服务化封装，但当前不作为默认低延迟 TTS。主要不足是 LoRA 微调和训练后
大规模指令遵循精度仍需补完整训练日志与更大评估集。
如需展示 SummerTTS 的低延迟改进，只建议引用 `summer-tts-cache-audit` 对固定短反馈语缓存
命中的 roundtrip 证据，不应把它扩展成整句生成或默认首音频 `<300ms`。
```
