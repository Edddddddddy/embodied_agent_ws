# 运行时证据状态

本文只回答“当前有哪些运行指标已经被证据证明”，不把接口存在、自动 mock 通过或 fallback
有效率当作真实模型能力。JSON 报告默认写入 `logs/`，该目录不提交 Git；结论必须能由下方
命令重新生成。

## 当前事实边界

| 证据 | 当前结果 | 结论 |
| --- | --- | --- |
| 离线真实麦克风长稳 | 旧报告 180.022 s；识别率 60%，动作准确率/成功率 70%；最终零速通过；turn P50/P95 为 3824/6972 ms | **失败且时长不足**，不能宣称 5 分钟稳定 |
| 在线真实麦克风长稳 | 尚无独立 5 分钟报告 | **缺失** |
| 离线模型原始指令遵循 | 代表样例 3/8，37.5% | **已测但未达 70% 目标** |
| fallback + safety 系统有效率 | 同一批样例 8/8，100% | **已测**，只能描述系统兜底效果，不能替代模型原始分数 |
| 离线 fixture E2E | endpoint 到首音频 707.533 ms，turn complete 1428.566 ms，LLM 首 token 120.607 ms | **已测且 <3.5 s**，但它是测试音频/fixture，不是 5 分钟真人长稳证据 |
| 离线组件性能 | warm LLM P95 544.79 ms、解码中位数 30.52 tokens/s、Sherpa-TTS 178.94 ms | **已测**，属于组件 benchmark |
| OpenLORIS SLAM | office1-1：322 对齐位姿、99.65% 覆盖，Ceres/GTSAM ATE 2.879/2.890 cm | **已测**，片段约 27 s 且无真值回访事件，不能宣称回环 precision/recall |

> 旧离线真人报告是一次失败样本，保留它是为了暴露问题，而不是选择性删除不理想结果。

## 证据口径

- `proven`：报告通过全部门槛、时长至少 300 s、模式与 online/offline 匹配，并明确声明
  `real_microphone`。
- `failed`：报告存在，但任一门槛、时长、模式或采集来源不满足。
- `missing`：对应模式没有报告。
- 自动 mock、测试 WAV 和 fixture 用于回归与延迟定位，不替代真实麦克风长稳验收。
- 模型原始准确率与规则/LLM fallback 后的系统有效率永久分栏。

## 重新留证

先在安静环境完成离线 5 分钟，随后再做在线模式：

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh
bash scripts/acceptance_test.sh wsl-microphone-preflight
bash scripts/acceptance_test.sh continuous-voice-evidence offline
bash scripts/acceptance_test.sh continuous-voice-evidence online
bash scripts/acceptance_test.sh runtime-evidence-summary
```

关键产物：

```text
logs/continuous_voice_offline_live_report.json
logs/voice_benchmark_offline_report.json
logs/continuous_voice_online_live_report.json
logs/voice_benchmark_online_report.json
logs/runtime_evidence_summary.json
```

正式通过标准：在线、离线的 `status` 都为 `proven`；每份报告至少 300 s；动作成功率和
识别率达到 benchmark 场景门槛；queue reject、误触发和延迟 P50/P95 都有数值；退出后
session sleeping，最终 `/cmd_vel` 为零。

若报告失败，不要手工修改 JSON。先检查 `ignored_transcript_count`、
`recognition_retry_count`、`queue_rejected_count` 和 latency 分布，再调 VAD/profile、说话节奏
或本地模型性能后重跑。
