# 语音 Agent 学习笔记

覆盖音频前端、在线/离线推理、连续会话、轻量 NLU、身份与记忆。每节均按功能、关键代码、上下游、设计原因、替代方案和安全边界组织。运行方法以 [测试手册](../TESTING.md) 为准。

## 1. 音频 AEC、VAD、endpoint 与 ASR commit

### 【功能】

从 WSL/PulseAudio 获取 PCM，执行回声抑制和音频指标统计，检测说话开始/结束，并在 endpoint 后
延迟提交 ASR，减少“左转九十度”被截成“左转”的尾部漏识别。

### 【关键文件/类/函数】

- `src/embodied_agent_cpp/src/audio_frontend_node.cpp`：`AudioFrontendNode`。
- `src/embodied_agent_cpp/src/audio_processing.cpp`：`EnergyVad::is_speech()`、
  `SpeechEndpointDetector::update()`、`SilenceDetector::update()`、`NlmsEchoCanceller::process()`。
- `src/embodied_voice_frontend/embodied_voice_frontend/silero_vad_sidecar.py`：
  `StreamingVadEndpoint`。
- `src/embodied_voice_frontend/embodied_voice_frontend/webrtc_vad_node.py`：`WebRtcVadNode`、`_on_audio()`。
- `src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py`：`AsrEndpointRuntime`。

### 【上游 → 处理 → 下游】

```text
PulseAudio source + TTS reference
→ NlmsEchoCanceller / AudioFrontendNode
→ /audio/clean_pcm + frontend metrics
→ energy、Silero 或 WebRTC VAD
→ /audio/speech_started + /audio/speech_ended
→ AsrEndpointRuntime 延迟 commit
→ online WebSocket commit 或 offline ASR queue event
```

### 【为什么这样设计】

音频采集、声学 endpoint 和语义理解属于不同变化方向。C++ 前端保证低开销 PCM 处理与统一指标，
成熟 VAD 作为 sidecar 可替换；Agent 只响应稳定 endpoint。commit delay 用少量延迟换取数字、量词
等尾部完整性。

### 【与替代方案区别】

- 固定静音 0.4 秒：响应快，短停顿和尾音容易截断。
- 只用能量 VAD：部署简单，对噪声和远场说话适应较弱。
- Silero/WebRTC VAD：泛化更好，增加模型/依赖和采样格式要求。
- 完整 WebRTC AEC：更成熟，WSL 音频路由与参考时钟接入更复杂。

### 【失败/安全边界】

NLMS 是轻量回声抑制，不等于生产级双讲 AEC。VAD provider 缺失时必须明确降级，不能静默声称
使用成熟模型。RMS/peak 很低时先检查 source 和输入增益，不要把阈值降到环境底噪以下。

### 【对应测试】

```bash
pytest -q src/embodied_voice_frontend/test
colcon test --packages-select embodied_agent_cpp --event-handlers console_direct+
```

## 2. 在线 ASR、LLM 与流式 TTS Adapter

### 【功能】

在线节点把音频发送给 Qwen/DashScope 兼容 ASR，把上下文交给 OpenAI-compatible LLM，增量解析
speech/action 标签，并把可说句子交给 Qwen TTS；provider 差异不侵入共享控制面。

### 【关键文件/类/函数】

- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`：`OnlineAgentNode`、
  `_on_clean_audio()`、`_commit_asr_endpoint()`、`_on_asr_final()`、`_accept_transcript()`、`_run_turn()`。
- `src/embodied_online_agent/embodied_online_agent/providers/qwen_asr.py`：
  `QwenRealtimeAsr.start()`、`push_audio()`、`commit()`。
- `src/embodied_online_agent/embodied_online_agent/providers/openai_compatible_llm.py`：
  `OpenAiCompatibleLlm.stream()`。
- `src/embodied_online_agent/embodied_online_agent/providers/qwen_tts.py`：
  `QwenRealtimeTts.synthesize()`。
- `src/embodied_agent_core/embodied_agent_core/streaming_turn.py`：`StreamingTurnRuntime.feed()`、`finish()`。

### 【上游 → 处理 → 下游】

```text
/audio/clean_pcm
→ QwenRealtimeAsr.push_audio()/commit()
→ OnlineAgentNode._on_asr_final()
→ AgentApplicationRuntime.accept_transcript()
→ OpenAiCompatibleLlm.stream()
→ StreamingTurnRuntime.feed()/finish()
→ QwenRealtimeTts.synthesize() + typed action candidate
```

### 【为什么这样设计】

节点负责 ROS/Lifecycle 接线，provider 负责云协议，`StreamingTurnRuntime` 负责标签协议、分句和动作
选择。API SDK 更新不会迫使会话、记忆、队列和 ActionGuard 一起变化。token 和可说句子分开，便于
分别观察首 token、首音频和完整动作。

### 【与替代方案区别】

- 单次 HTTP：实现简单，无法提供 ASR partial、LLM token 和低等待 TTS。
- 节点直接解析 SDK event：接入快，测试必须连接真实云端。
- provider Adapter：多一层接口，换来 mock、重试和 API 替换能力。
- LLM 自由输出动作：泛化强、协议不稳定，仍需 parser 和 Guard。

### 【失败/安全边界】

网络、配额、key、限流和模型升级都可能失败。在线延迟必须由本次报告证明，不能永久引用历史
`<1s`。LLM 输出只是候选，任何动作仍要通过 typed transport 和 C++ ActionGuard。

### 【对应测试】

```bash
bash scripts/acceptance_test.sh continuous-online
pytest -q src/embodied_agent_core/test/test_streaming_turn.py
```

## 3. 离线 Sherpa ASR、llama.cpp 与 TTS 双缓冲

### 【功能】

在无云环境使用 Sherpa-ONNX ZipFormer、llama.cpp GGUF 和 Sherpa-TTS/SummerTTS；通过文本/音频
双缓冲让 LLM 与 TTS 并行，并记录首 token、decode 和首音频指标。

### 【关键文件/类/函数】

- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`：`OfflineAgentNode`、
  `_enqueue_asr()`、`_run_asr()`、`_on_asr_final()`、`_run_turn()`。
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_asr.py`：
  `SherpaZipformerAsr.push_audio()`、`commit()`、`_decode_ready()`。
- `src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py`：
  `LlamaCppLlm.warmup()`、`stream()`。
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_tts.py`：`SherpaVitsTts.synthesize()`。
- `src/embodied_offline_agent/embodied_offline_agent/double_buffer.py`：`DoubleBuffer.put()`、`get()`、`abort()`。
- `src/embodied_offline_agent/embodied_offline_agent/pseudo_streaming_tts.py`：
  `PseudoStreamingTtsPipeline._tts_worker()`、`_audio_worker()`。
- `src/embodied_offline_agent/embodied_offline_agent/offline_turn_runtime.py`：
  `OfflineStreamingTurnRuntime.run()`。

### 【上游 → 处理 → 下游】

```text
clean PCM + endpoint
→ SherpaZipformerAsr
→ AgentApplicationRuntime / OfflineStreamingTurnRuntime
→ LlamaCppLlm.stream()
→ 文本 DoubleBuffer → TTS worker
→ 音频 DoubleBuffer → audio worker
→ /audio/tts_pcm
```

### 【为什么这样设计】

ASR、LLM 和 TTS 的计算特征不同。有界双缓冲隔离生成速度与播放速度，避免 TTS 阻塞 LLM token；
abort/close 让取消和停机有明确语义。provider 测试可以使用 fake runtime，不必下载大模型。

### 【与替代方案区别】

- LLM 完成后再 TTS：简单，首音频等待长。
- 原生流式 TTS：延迟更低，模型必须支持增量状态。
- 分句伪流式：适配现有模型，第一句仍需整句合成。
- SummerTTS 常驻 C++ service：减少加载开销，未命中缓存仍可能较慢。

### 【失败/安全边界】

模型、tokenizer、GGUF 架构与运行时版本必须匹配。队列满时不能无限占用内存；取消必须 abort 两个
buffer。Q8/LoRA 合成 holdout 不等于真实麦克风准确率，伪流式不能表述为原生流式 TTS。

### 【对应测试】

```bash
pytest -q src/embodied_offline_agent/test
bash scripts/acceptance_test.sh continuous-offline
```

## 4. 连续会话、多命令 NLU 与执行队列

### 【功能】

支持一次唤醒后连续输入，把一句话解析成多个动作，在前一动作执行时继续接收命令，并用优先 stop、
TTL、duplicate/filler 过滤和 command ID 保证长期控制不乱序。

### 【关键文件/类/函数】

- `src/embodied_agent_core/embodied_agent_core/continuous_voice.py`：
  `ContinuousVoiceSession`、`ContinuousCommandQueue`。
- `src/embodied_agent_core/embodied_agent_core/agent_control_plane.py`：
  `AgentControlPlane.accept_transcript()`、`enqueue_command()`。
- `src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py`：
  `AgentApplicationRuntime.accept_transcript()`、`run_queued_turn()`、私有 `_run_preparsed_turn()`。
- `src/embodied_agent_core/embodied_agent_core/command_nlu.py`：`CommandNLU.parse()`。
- `src/embodied_agent_core/embodied_agent_core/command_completion.py`、`command_fallback.py`。
- `src/embodied_agent_core/embodied_agent_core/agent_execution_runtime.py`：`AgentExecutionRuntime`。

### 【上游 → 处理 → 下游】

```text
ASR final
→ AgentApplicationRuntime.accept_transcript()
→ AgentControlPlane.accept_transcript()
→ wake/session、normalization、completion、NLU batch
→ enqueue_command() → run_queued_turn()
→ _run_preparsed_turn() 或 provider turn
→ publish_actions()
→ 等待同 command_id 的 Action result
```

### 【为什么这样设计】

输入与执行速度不同，busy 时丢命令会像卡住，并行执行又会产生运动冲突。单 worker + FIFO 保证
顺序，batch ID 表示同一句多动作，command/request ID 关联结果。高置信度 NLU 直接执行，低置信度
才交给 LLM，兼顾延迟和泛化。

### 【与替代方案区别】

- 按“然后/再”切字符串：难处理否定、自然表达和组合动作。
- 小型字符模型 + 槽位规则：轻量可解释，领域外泛化有限。
- 全部 function calling：能力强，在线成本和延迟高，离线小模型协议不稳。
- 并行动作：吞吐高，移动、转向和导航无法安全并发。

### 【失败/安全边界】

`停下/急停` 不排队，必须取消 active goal 并清 pending；计划 STOP 不能误当用户急停。过期命令
不得很久后执行，旧 result 不得唤醒下一 command ID。否定句和疑问句不应猜测执行。

### 【对应测试】

```bash
pytest -q src/embodied_agent_core/test/test_continuous_voice.py \
  src/embodied_agent_core/test/test_command_nlu.py
bash scripts/acceptance_test.sh continuous-multi-command
```

## 5. 声纹身份、用户记忆与行为偏好

### 【功能】

把声纹身份、录入、用户偏好和交互历史组合为 turn 级不可变上下文，使“默认慢一点”等习惯确定性
影响动作参数，同时防止低置信度身份污染个人画像。

### 【关键文件/类/函数】

- `src/embodied_voice_frontend/embodied_voice_frontend/speaker_identity_node.py`：
  `SpeakerIdentityNode`、`classify_speaker_scores()`、`_on_speech_ended()`。
- `src/embodied_agent_core/embodied_agent_core/user_context_runtime.py`：
  `UserContextRuntime.snapshot()`、`handle_command()`、`record_interaction()`。
- `src/embodied_agent_core/embodied_agent_core/memory_command_service.py`：`MemoryCommandService.handle()`。
- `src/embodied_agent_core/embodied_agent_core/user_memory.py`：
  `UserMemoryStore.profile()`、`set_preference()`、`record_interaction()`、`prompt_summary()`。
- `src/embodied_agent_core/embodied_agent_core/user_preferences.py`：动作偏好应用逻辑。

### 【上游 → 处理 → 下游】

```text
clean PCM + speech_ended
→ SpeakerIdentityNode
→ /agent/speaker_identity
→ UserContextRuntime.update_identity()
→ 命令入队时 snapshot()
→ system prompt + deterministic preferences
→ 已选择动作 → record_interaction()
```

### 【为什么这样设计】

声纹会在 turn 期间异步更新。入队时冻结 `UserContextSnapshot`，可保证 prompt、偏好和 interaction
属于同一用户。记忆只记录最终被策略接受的动作，不让被安全层拦截的模型输出污染画像。

### 【与替代方案区别】

- 全局共享记忆：简单，多人串写。
- 每次读取当前身份：实时，同一 turn 可能前后换用户。
- 向量数据库：适合开放知识召回；固定控制偏好用结构化 profile 更可审计。
- 云声纹：可能更准，引入隐私、网络和费用。

### 【失败/安全边界】

low-confidence/unknown 不可写个人 profile；多人、多房间 FAR/FRR 尚未充分评测。记忆不是安全授权，
用户应能清理数据，偏好修改后的动作仍必须经过 ActionGuard。

### 【对应测试】

```bash
pytest -q src/embodied_agent_core/test/test_user_context_runtime.py \
  src/embodied_agent_core/test/test_user_memory.py \
  src/embodied_voice_frontend/test/test_speaker_identity_node.py
```
