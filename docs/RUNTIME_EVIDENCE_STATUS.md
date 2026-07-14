# 运行时证据状态

本文只回答“当前有哪些运行指标已经被证据证明”，不把接口存在、自动 mock 通过或 fallback
有效率当作真实模型能力。JSON 报告默认写入 `logs/`，该目录不提交 Git；结论必须能由下方
命令重新生成。

## 当前事实边界

| 证据 | 当前结果 | 结论 |
| --- | --- | --- |
| 离线真实麦克风长稳 | 300.021 s；识别率 80%，动作准确率/成功率 90%；最终零速通过；turn P50/P95 为 2413/6946 ms | **已留证**，但用户现场仍观察到语音截断和长尾，本轮不把成功率作为算法门禁 |
| 在线真实麦克风长稳 | 尚无独立 5 分钟报告 | **缺失** |
| 原始 Q8 → LoRA Q8 动作语义 | 独立合成 holdout 43 条：13/43（30.23%）→ 23/43（53.49%） | **已测且有提升**，但不是严格协议分数或真实语音准确率 |
| 严格标签协议 + 动作 | 两侧均 11/43（25.58%）；LoRA 协议完整率 60.47% | **未提升**，Qwen3 thinking/标签输出仍是明确不足 |
| fallback + safety 系统有效率 | 两侧均 36/43（83.72%） | **已测**，只能描述系统兜底效果，不能替代模型原始分数 |
| 离线 fixture E2E | endpoint 到首音频 707.533 ms，turn complete 1428.566 ms，LLM 首 token 120.607 ms | **已测且 <3.5 s**，但它是测试音频/fixture，不是 5 分钟真人长稳证据 |
| 离线组件性能 | warm LLM P95 544.79 ms、解码中位数 30.52 tokens/s、Sherpa-TTS 178.94 ms | **已测**，属于组件 benchmark |
| OpenLORIS SLAM | office1-7：449 对齐位姿、99.753% 覆盖，Ceres/GTSAM ATE 约 10 cm；6 组前端阈值均为 46 条相邻边、accepted 非局部边 0；baseline 47 个图节点中 39 次被 near-linked 排除、8 次历史不足、0 次 coarse check | **已测**，已补来源绑定的 5 MB SLAM-only bag、阈值消融和 Karto 前端诊断；当前不能宣称回环检测成功 |

> 5 分钟报告通过不代表截断问题消失；现场观察到的尾部丢失和 P95 长尾继续作为已知问题保留。

## 证据口径

- `proven`：报告通过全部门槛、时长至少 300 s、模式与 online/offline 匹配，并明确声明
  `real_microphone`。
- `failed`：报告存在，但任一门槛、时长、模式或采集来源不满足。
- `missing`：对应模式没有报告。
- 自动 mock、测试 WAV 和 fixture 用于回归与延迟定位，不替代真实麦克风长稳验收。
- 模型原始动作语义、标签协议完整率、严格总分与 fallback 后系统有效率永久分栏。

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
