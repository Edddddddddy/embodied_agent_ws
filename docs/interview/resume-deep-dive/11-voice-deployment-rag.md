# 语音运行时部署与 RAG 深挖

本章回答两个面试问题：为什么没有把所有热门模型堆进机器人，以及怎样证明当前组合在受限 WSL
中可以部署。论文结论只用于技术选型，论文指标不等于本项目实测。

## 1. 一分钟架构讲法

口述：

项目把语音交互分为控制快通道和知识问答通道。明确运动、导航和急停由本地 NLU 解析，零检索、
零自由文本授权，最终仍经过 C++ ActionGuard。知识问答才按需检索本地手册，把有 source_id 和字符
预算的证据送给在线或离线 LLM。在线/离线只替换 provider，共用 VAD 端点、会话、Prompt 构造、
记忆、安全动作和 ROS 2 执行链。

```text
PCM → VAD/Endpoint → ASR final → Session gate
                              ├─ 控制命令 → CommandNLU → typed Action → ActionGuard
                              └─ 知识问题 → RAG → LLM → TTS
```

关键代码：

- [`AsrEndpointRuntime`](../../../src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py)
- [`RagQueryRouter`、`SparseKnowledgeRetriever`、`PromptContextAssembler`](../../../src/embodied_agent_core/embodied_agent_core/prompt_context.py)
- [`OnlineStreamingTurnRuntime`](../../../src/embodied_online_agent/embodied_online_agent/online_turn_runtime.py)
- [`OfflineStreamingTurnRuntime`](../../../src/embodied_offline_agent/embodied_offline_agent/offline_turn_runtime.py)
- [`VoiceRuntimeDeploymentChecker`](../../../src/embodied_agent_core/embodied_agent_core/voice_runtime_deployment.py)

## 2. 为什么选这套部署组合

| 模块 | 当前默认 | 采用原因 | 没有直接替换成什么 |
| --- | --- | --- | --- |
| VAD | Silero ONNX 优先，WebRTC/energy fallback | CPU 轻、模型小，sidecar 与端点语义解耦 | 不训练自有 Semantic VAD |
| 离线 ASR | sherpa-onnx Zipformer | 流式、INT8/ONNX、CPU 部署成熟 | 不把较重的 FunASR 2-pass 与 Gazebo 常驻 |
| 在线 ASR | Qwen Realtime | 已有 WebSocket provider，云端效果用于在线 profile | 不用在线服务替代离线可用性 |
| 离线 LLM | llama.cpp + Qwen3-0.6B Q8 | 常驻 server、SSE、低内存、GGUF 工具链成熟 | 不默认 32K context、thinking 或第二个 draft model |
| RAG | 本地稀疏 BM25 基线 | 零模型依赖、小语料足够、可解释、可测 | 不默认 BGE-M3、向量数据库或 Self-RAG |
| 离线 TTS | Sherpa-TTS | 当前稳定默认、CPU 可用 | Summer 是显式可选 C++ 服务；CosyVoice2 留给 GPU sidecar |
| 在线 TTS | Qwen Realtime | 持久 WebSocket，避免每轮重新握手 | 当前不宣称模型级逐 token true streaming |

部署资产清单位于
[`voice_runtime_profiles.yaml`](../../../config/voice_runtime_profiles.yaml)。`offline-edge` 和
`online-cloud` 都声明 VAD、ASR、LLM、RAG、TTS 的必需资产；preflight 不会暗中改写 launch。
连续语音入口已有 VAD 自动选择并把结果传给 launch，因此只有 VAD 声明自动 fallback；
ASR、LLM、RAG、默认 Sherpa-TTS 缺失时 fail-closed，SummerTTS 必须显式选择。
云端仅发现 SDK 和非空 Key 时仍标为 `unverified`；只有受控在线验收才能证明网络、权限和
密钥确实可用，避免把“配置存在”误写成“服务健康”。

## 3. 论文怎样映射到项目

### 3.1 ASR、VAD 与 TTS

| 文献/实现 | 关键结论 | 本项目采用方式 |
| --- | --- | --- |
| [Zipformer, ICLR 2024](https://proceedings.iclr.cc/paper_files/paper/2024/hash/c1bb0e3b062f0a443f2cc8a4ec4bb30d-Abstract-Conference.html) | 多帧率 encoder、attention 权重复用，面向更快更省内存 ASR | 使用 sherpa-onnx 流式 Zipformer；记录 tail padding、RTF 和模型版本 |
| [Semantic VAD, Interspeech 2023](https://www.isca-archive.org/interspeech_2023/shi23c_interspeech.pdf) | 语义信息可降低只看声学静音造成的尾部延迟 | 不训练论文模型；采用“声学端点 + partial 稳定 + 可取消 commit delay”的工程近似 |
| [FunASR, Interspeech 2023](https://arxiv.org/abs/2305.11013) | 流式/离线两遍、VAD、标点和热词适合工业 ASR 工具链 | 作为高精度 sidecar 候选；8 GiB WSL 默认不与 Gazebo 同驻 |
| [CosyVoice 2 technical report, 2024](https://arxiv.org/abs/2412.10117) | chunk-aware causal flow matching 统一流式与非流式 TTS | 作为 GPU/服务化 TTS 候选；当前 CPU 默认仍是 Sherpa |
| [Silero VAD 官方实现](https://github.com/snakers4/silero-vad) | ONNX、CPU 小模型、成熟工程集成 | 真实麦克风首选 provider；模型缺失后按 profile 明确降级 |

论文说“降低多少延迟”不能直接写进简历。本项目必须在当前麦克风、CPU、线程数和模型 SHA 下重新测
endpoint latency、CER、尾部截断率、RTF、TTS 首音频和 RSS。

### 3.2 LLM 推理

| 文献 | 关键结论 | 本项目取舍 |
| --- | --- | --- |
| [Qwen3 technical report, 2025](https://arxiv.org/abs/2505.09388) | 小尺寸模型、thinking/non-thinking 等统一能力 | 使用 0.6B GGUF 且关闭 thinking；上下文限制在实际需要范围 |
| [SmoothQuant, ICML 2023](https://proceedings.mlr.press/v202/xiao23c.html) | training-free W8A8 可平滑 activation outlier | 用于理解量化取舍；当前 CPU 工具链保持 GGUF Q8 |
| [AWQ, MLSys 2024 Best Paper](https://proceedings.mlsys.org/paper_files/paper/2024/hash/42a452cbafa9dd64e9ba4aa95cc1ef21-Abstract-Conference.html) | activation-aware PTQ 能保护重要权重 | 后续用机器人指令校准集做 Q8/Q5/Q4 imatrix 消融，不直接迁移 GPU TinyChat runtime |
| [Speculative Decoding, ICML 2023](https://proceedings.mlr.press/v202/leviathan23a.html) | 在不改变目标分布下并行验证 draft token | 先评 llama.cpp n-gram draft；0.6B 已很小，不默认再常驻一个模型 |

现有 [`LlamaCppLlm`](../../../src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py)
记录首 token、总耗时、token 数、重试和 decode 速度。控制 turn 可以保存模型原始协议输出以复用
KV 公共前缀；RAG/普通聊天只保存干净用户原文和纯 `<speech>`。这会牺牲部分 RAG 跨轮 KV 命中，
但避免知识正文、提示注入和已拦截 action 污染后续上下文。

### 3.3 RAG 与记忆

| 文献 | 关键结论 | 本项目取舍 |
| --- | --- | --- |
| [Adaptive-RAG, NAACL 2024](https://aclanthology.org/2024.naacl-long.389/) | 按问题复杂度选择 zero/single/multi-hop retrieval | 不训练分类器；用 CommandNLU 做 zero-retrieval 安全路由，知识问题做一次有界检索 |
| [BGE-M3, Findings ACL 2024](https://aclanthology.org/2024.findings-acl.137/) | dense、sparse、multi-vector 的多功能检索 | 568M 默认过重；固定 QA 集证明稀疏召回不足后再评 BGE-small ONNX + RRF |
| [RAGAS, EACL 2024](https://aclanthology.org/2024.eacl-demo.16/) | 无参考或弱参考地拆分 RAG 评测维度 | CI 先测 Recall@k、source_id、无证据 abstention 和控制 zero retrieval；LLM judge 只作可选离线分析 |
| [LongMemEval, ICLR 2025](https://proceedings.iclr.cc/paper_files/paper/2025/file/d813d324dbf0598bbdc9c8e79740ed01-Paper-Conference.pdf) | 长期记忆应拆分 indexing、retrieval、reading，并关注时间信息 | 保留显式画像、带时间 recent evidence 和有限摘要；低置信声纹拒绝写入 |

当前 [`UserMemoryStore`](../../../src/embodied_agent_core/embodied_agent_core/user_memory.py) 是用户画像，
不是 RAG。当前 RAG 知识库是只读公共文档，两者生命周期、权限和清除语义保持分离。

声纹身份也要单独说明事实边界：

- 默认演示 provider 是 `mock`，用于稳定验证 typed identity、录入消息、低置信拒写、偏好应用和 turn
  快照，不构成声纹准确率证据。
- [`SpeakerIdentityNode`](../../../src/embodied_voice_frontend/embodied_voice_frontend/speaker_identity_node.py)
  的 Sherpa mode 已实现 speaker embedding extractor/manager、注册样本聚合、top-1 threshold 与 top-2
  margin；它是目标机接入 seam，不是默认 profile 的已验收事实。
- 真实部署仍需固定模型 SHA、注册语料、麦克风距离和噪声条件，报告误认率、拒识率及 unknown 比例；
  在这些证据完成前，简历只写“声纹与用户记忆接口及 Sherpa 接入 seam”。

## 4. Prompt 构造为什么是一个深模块

以前在线节点直接调用 `ConversationMemory.prompt_messages()`，离线节点又维护 `_llm_messages()`。
加入检索后如果继续在两个节点拼字符串，会产生路由、预算、引用和 `/no_think` 漂移。

现在 `PromptContextAssembler.build()` 一次完成：

1. `RagQueryRouter.route()` 先识别控制快通道。
2. 知识问题调用 `SparseKnowledgeRetriever.retrieve()`。
3. 对 `top_k` 和总字符数做硬限制。
4. 给证据加 source_id 和“不可信数据”边界。
5. 组合 system、用户画像、历史和当前 user。
6. 用 context window、生成上限和安全余量计算输入预算，先删旧历史、再裁 RAG。
7. 校验回答是否引用本轮真实 source_id；缺失时告警，不伪造来源。
8. 返回 `route/retrieval_ms/source_ids/corpus_version/budget`，但不把正文复制进日志。

在线、离线 turn runtime 只消费 `PromptBuildResult`，不再知道具体检索算法。

## 5. 怎样部署和验收

```bash
# CI：只验证 profile 契约，不依赖模型/API
bash scripts/acceptance_test.sh voice-runtime-preflight offline-edge --contract-only --json
bash scripts/acceptance_test.sh voice-runtime-preflight online-cloud --contract-only --json

# 目标机器：检查模块、模型、知识文件、API key 和 fallback
bash scripts/acceptance_test.sh voice-runtime-preflight offline-edge --json
bash scripts/acceptance_test.sh voice-runtime-preflight online-cloud --json

# llama-server 已启动后才显式做只读 health GET
bash scripts/acceptance_test.sh voice-runtime-preflight \
  offline-edge --probe-endpoints --json
```

退出码：

- `0`：所有必需组件可用，或契约有效。
- `1`：必需组件缺失。
- `2`：profile 或 CLI 参数错误。

不把实际模型/API preflight 加进无模型 CI；CI 运行确定性假依赖，目标机运行真实资产检查。

## 6. 面试追问

### Q1. 为什么不用端到端 speech-to-speech？

它会弱化可观察文本、typed action、ActionGuard 和逐层错误归因，而且模型和显存需求与当前
WSL/Gazebo 共存目标冲突。模块化链路可以独立替换 VAD/ASR/TTS，并保留机器人控制的确定性边界。

### Q2. 为什么 RAG 不直接决定机器人动作？

检索文档可能过时或包含提示注入，LLM 也可能误解证据。RAG 只服务问答；运动命令由本地 NLU
快通道处理，候选动作仍被 C++ 校验。即使知识库失效，急停和基础控制也不受影响。

### Q3. 为什么第一版不用向量数据库？

当前知识库小，稀疏 BM25 可解释、零模型依赖，启动内存和故障面更小。应先用固定 QA 集证明
Recall@k 不足，再以相同数据比较 BGE-small ONNX + RRF；不能因为向量数据库流行就常驻新服务。

### Q4. 如何防止 Prompt 无限增长？

短期历史有轮数上限，用户画像有字符上限，RAG 有 top_k/字符预算。请求前再以
`context_window - max_output - safety_reserve` 得到输入 token 预算，先整轮删除最旧历史，再二分
裁剪 RAG；系统约束和当前问题不截断，仍无法容纳则 fail-closed。日志记录 estimated token、
retrieved_chars 和 source_id，可以定位 TTFT 变慢是否来自检索上下文。

### Q5. 在线 RAG 的数据边界是什么？

在线命中的证据会随 Prompt 发送给 DashScope，因此默认 `builtin_only`，只允许包内公开运行手册。
自定义路径在 Lifecycle configure 阶段会被拒绝；只有审查文档权限和供应商边界后，才能显式设置
`allow_custom`。离线 profile 不出端，也可用 `off` 完全关闭检索。

### Q6. 下一阶段最重要的交互优化是什么？

真正的 barge-in：新的有效用户语音要产生 generation，取消 LLM/TTS、清播放双缓冲和 AEC reference，
同时保证机器人急停不等待语音线程。其次才是低置信 ASR 二次终稿和 dense retrieval A/B。

## 7. 可说与不可说

可以说：

- 已实现在线/离线统一 Prompt seam、控制零检索、本地知识检索、引用预算和部署 profile。
- 当前默认组合针对 8 GiB WSL 与 Gazebo 共存，优先低依赖、可降级和可观测。
- 论文方法用于选型，项目性能只引用当前机器的实际报告。

不要说：

- 已部署 Self-RAG、BGE-M3、向量数据库或生产级长期记忆平台。
- CosyVoice2、FunASR 2-pass 或论文延迟已经在当前项目复现。
- 稀疏 RAG 已达到尚未建立金集验证的 Recall@k 或回答准确率。
