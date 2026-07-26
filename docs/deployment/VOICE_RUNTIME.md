# 语音运行时部署

本文只说明 VAD、ASR、LLM/RAG、TTS 的部署契约和排障顺序。语音算法调用链见
[语音学习笔记](../learning/VOICE_AGENT.md)，机器人控制与 SLAM/Nav2 验收见
[测试手册](../TESTING.md)。

## 1. 两套 profile

| Profile | VAD | ASR | LLM | RAG | TTS |
| --- | --- | --- | --- | --- | --- |
| `offline-edge` | Silero → WebRTC → energy | Sherpa Zipformer | llama.cpp/Qwen3-0.6B | 本地稀疏 BM25 | Sherpa |
| `online-cloud` | Silero → WebRTC → energy | Qwen Realtime | Qwen OpenAI-compatible | 本地稀疏 BM25 | Qwen Realtime |

部署资产清单是 [`config/voice_runtime_profiles.yaml`](../../config/voice_runtime_profiles.yaml)。
模型不复制进 Git 或基础镜像；linked worktree 默认通过 Git common directory 复用主工作区
`models/` 和 `third_party/`，也可显式传 `--runtime-root`。

该清单负责检查“本机能否启动”，不会在后台改写 ROS 参数。真实连续语音入口仍由
`continuous_voice_control.sh` 把已验证的 VAD 选择传给 launch；当前只有
Silero → WebRTC → energy 是自动 fallback。ASR、LLM、RAG 和默认 Sherpa-TTS 缺失时
直接 fail-closed。SummerTTS 需显式设置离线 `tts_provider`，不伪装成自动降级。

## 2. 运行 preflight

```bash
source scripts/activate.sh

# profile 契约：适合 CI，不要求模型和密钥
bash scripts/acceptance_test.sh voice-runtime-preflight offline-edge --contract-only
bash scripts/acceptance_test.sh voice-runtime-preflight online-cloud --contract-only

# 目标机器资产检查：逐文件校验 ZipFormer、llama-server/GGUF 和 Sherpa-TTS
bash scripts/acceptance_test.sh voice-runtime-preflight offline-edge --json
bash scripts/acceptance_test.sh voice-runtime-preflight online-cloud --json

# llama-server 已启动时，额外检查只读 /health
bash scripts/acceptance_test.sh voice-runtime-preflight \
  offline-edge --probe-endpoints --json
```

该入口不会下载权重、不会运行生成，也不会调用云端付费推理。在线 profile 只检查
SDK 与 `DASHSCOPE_API_KEY` 是否存在，不输出密钥值；静态检查无法证明网络、账号权限
或密钥有效，因此云 ASR/LLM/TTS 会保守标为 `unverified`，汇总状态为 `degraded`。

## 3. 状态含义

- `ready`：所有必需组件有可用 candidate，且不存在尚未验证的 endpoint。
- `unverified`（组件级）：endpoint 未探测，或云端只完成 SDK/Key 静态检查，不能宣称
  服务健康；汇总状态会保守显示为 `degraded`。
- `degraded`：存在未验证 endpoint，或 VAD 首选 candidate 不可用但已选择明确 fallback，
  例如 Silero 缺失后使用 WebRTC。
- `blocked`：ASR、LLM、RAG 或 TTS 等必需组件没有可用实现。
- `contract_valid`：只验证声明结构，不代表本机模型、密钥或设备可用。

组件证据只记录模块名、路径、provider 和 endpoint，不复制用户文本、知识正文或密钥。

## 4. RAG 部署和安全

代码入口：

- [`prompt_context.py`](../../src/embodied_agent_core/embodied_agent_core/prompt_context.py)
  - `RagQueryRouter.route()`：控制命令零检索。
  - `SparseKnowledgeRetriever.retrieve()`：中文字符 unigram/bigram + ASCII token 的 BM25。
  - `PromptContextAssembler.build()`：统一 system、用户画像、历史和当前证据。
- [`robot_runtime_zh.md`](../../src/embodied_agent_core/knowledge/robot_runtime_zh.md)：内置小型知识库。

知识文件是只读不可信数据：它只能帮助回答，不能授权动作、覆盖系统提示词或绕过 C++
ActionGuard。上下文有 `top_k` 和字符预算；日志只输出 source_id、耗时、命中数和索引版本。
响应结束后会校验模型引用的 source_id 是否属于本轮真实命中；缺失引用会产生告警和离线指标，
但不会伪造来源。
Prompt 还使用 `llm_context_window_tokens - llm_max_tokens - prompt_safety_reserve_tokens`
计算输入预算：先整轮淘汰最旧历史，再裁剪 RAG，系统约束和当前问题不截断；若固定内容仍超窗，
请求会在调用 provider 前 fail-closed。

`online-cloud` 会把命中的证据随 Prompt 发给 DashScope。默认
`rag_cloud_context_policy=builtin_only`，只允许包内公开运行手册；配置自定义知识文件会在
Lifecycle configure 阶段失败，必须审查数据出端边界后显式设为 `allow_custom`。离线 profile
默认允许自定义本地文档，也可设为 `off` 完全关闭检索。

默认稀疏检索没有 torch、embedding service 或向量数据库依赖，适合 8 GiB WSL 与 Gazebo 共存。
后续只有在固定 QA 集证明召回不足时，才增加 BGE-small ONNX + RRF；不默认常驻 BGE-M3、
Qdrant/Milvus 或 Self-RAG。

## 5. 启动顺序

离线：

```text
模型/知识资产 preflight
→ llama-server health
→ Lifecycle configure：加载 Zipformer、Prompt/知识索引和 TTS
→ warmup：稳定 system/history 前缀
→ activate：开始音频输入
```

在线：

```text
依赖/API key preflight
→ Lifecycle configure：创建 provider、加载本地知识索引
→ WebSocket/TLS warmup
→ activate：开始音频输入
```

ROS 启动后的节点 READY、麦克风波形和 VAD 现场采样仍分别由 `system_readiness_check.py`、
`voice_control_readiness_check.py` 负责；部署前资产检查不能替代运行时观测。

## 6. 当前边界

- Sherpa-TTS 是稳定离线默认；SummerTTS 是显式选择的可选 C++/ROS Service 实现，
  当前不参与自动 fallback。
- 在线 TTS 使用持久 WebSocket，但当前 server commit 仍是 response 级，不能夸大为模型级逐 token TTS。
- 当前没有真正的 LLM/TTS/扬声器 barge-in；急停已抢占机器人动作，但语音输出取消是下一阶段。
- Docker 已有基础镜像和 Compose 门禁，尚未把模型、麦克风和全部 sidecar 封装成生产拓扑。
- 论文中的延迟、准确率和内存结果不是本项目指标；项目数据必须来自当前机器的报告。
