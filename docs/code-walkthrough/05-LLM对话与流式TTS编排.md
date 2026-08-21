# 05 LLM 对话与流式 TTS 编排

## 源码导航

| 文件与关键行 | 符号 | 观察点 |
|---|---|---|
| `src/embodied_online_agent/embodied_online_agent/online_agent_node.py:209` | `_accept_transcript` | busy 锁与 turn 创建 |
| `src/embodied_online_agent/embodied_online_agent/online_agent_node.py:230` | `_run_turn` | 在线 LLM/TTS 流水线 |
| `src/embodied_online_agent/embodied_online_agent/protocol.py:31` | `TaggedStreamParser.feed` | 增量标签状态机 |
| `src/embodied_online_agent/embodied_online_agent/protocol.py:96` | `SentenceChunker` | TTS 切句 |
| `src/embodied_online_agent/embodied_online_agent/providers/qwen_tts.py:97` | `QwenRealtimeTts.synthesize` | 持久连接与 commit |
| `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py:260` | `_run_turn` | 离线三级流水线 |
| `src/embodied_offline_agent/embodied_offline_agent/double_buffer.py:16` | `DoubleBuffer` | 背压、close、abort |

## 1. 一轮对话不是串行三步

朴素实现是等待 LLM 全文，再调用 TTS，再播放。首音频延迟会累加：

```text
LLM 全文时间 + TTS 全文时间 + 播放准备时间
```

项目改为流式流水线：

```text
LLM token -> 协议解析 -> speech delta -> 句子切块 -> TTS -> PCM chunk -> 播放
                     \-> complete action -> Guard
```

speech 可以尽早说，action 必须完整解析后才发布。

## 2. 一轮 turn 的状态与锁

`_accept_transcript()`：

1. `WakeWordGate.process()` 去掉唤醒词或接受 10 秒 follow-up；
2. 重试计数归零；
3. 在 `_state_lock` 下检查并设置 `_busy`；
4. 启动 daemon thread 执行 `_run_turn()`。

`_busy` 防止两个 utterance 同时修改 memory、复用 provider 或发布交错 TTS。检查和设置必须在同一临界区，否则两个 callback 都可能先看到 false。

缺点是重叠语音被丢弃，不排队，也不打断当前 turn。这是明确的产品取舍。

## 3. 对话记忆

`ConversationMemory` 只保存 user/assistant 消息，最大轮数乘 2 得到 message 上限。

写入流程：

```text
更新内存列表 -> 写 memory.json.tmp -> os.replace(tmp, memory.json)
```

`os.replace` 避免程序在写一半崩溃留下半截 JSON。`RLock` 保护 load/messages/append/clear。文件损坏时恢复为空，而不是阻止节点启动。

这属于短期对话历史，不是向量数据库、长期语义记忆或 RAG。

## 4. 标签协议为什么需要增量状态机

模型被要求输出：

```text
<speech>好的，向前走。</speech>
<action>{"name":"move","arguments":{...}}</action>
```

网络 token 可能在任意位置切分：

```text
"<spe" + "ech>好" + "的</speech><act" + "ion>{...}"
```

`TaggedStreamParser` 保存 `buffer` 和 `state`：outside、speech、action。

- outside 只保留可能组成下一标签的尾部；
- speech 在没有闭标签时释放“确定不可能属于 `</speech>`”的安全前缀；
- action 一直等待完整 `</action>`，再一次性 `json.loads()`；
- `finish()` 检测未闭合标签。

这保证半个 JSON 不会变成机器人命令，同时 speech 仍可边到边播。

## 5. SentenceChunker 的延迟权衡

TTS 不适合每个字符调用，也不应等整段。`SentenceChunker` 遇到中文标点或达到最大字符数就 flush。

| chunk 短 | chunk 长 |
|---|---|
| 首音频更快 | 韵律和上下文更完整 |
| TTS 调用/commit 更频繁 | 调用次数更少 |
| 可能断句生硬 | 用户等待更久 |

在线默认 32 字符，离线默认 24，因为离线 TTS 是句级整块生成，需要更早开始。

## 6. 在线 TTS 流水线

`OnlineAgentNode._run_turn()` 创建 `text_queue` 和 TTS thread。LLM thread 将 speakable chunk 放入队列，TTS thread 通过 generator 阻塞消费。

`QwenRealtimeTts`：

- `connect()` 预建持久 WebSocket；
- `_operation_lock` 保证同一 session 一次只合成一个 response；
- 逐 chunk `append_text()`；
- generator 结束后 `commit()`；
- callback 收到 `response.audio.delta` 立即发布 PCM；
- 30 秒无 done 则 timeout。

WebSocket 在多轮间复用，减少 TLS/连接首轮成本。`online_warmup_enabled` 还会预连 TTS、发一个最小 LLM 请求。

## 7. 离线三级流水线

离线没有真正的 token-to-audio 模型流，采用伪流式：

```text
LLM thread
  -> message_buffer(capacity=2)
TTS thread
  -> 对一句完整 synthesize
  -> 切为 80 ms PCM
  -> audio_buffer(capacity=2)
Audio thread
  -> publish /audio/tts_pcm
```

`DoubleBuffer` 的 capacity 固定为 2，限制内存和流水线领先程度。当前 `_run_turn()` 为两个 buffer 都传入 `drop_oldest=False`，所以满时生产者最多等待 5 秒，超时则整轮失败。虽然 `DoubleBuffer` 支持音频丢最旧策略，但当前离线 turn 没有启用；不能把类注释中的可选能力说成当前运行行为。

`close()` 表示正常排空后停止，`abort()` 表示丢弃排队工作并立刻唤醒消费者。异常路径必须 abort 两个 buffer，否则线程可能永远等 sentinel。

## 8. 动作与朗读的不同发布时机

speech delta 到达后马上进入 TTS。action 则在完整 JSON 被 parser 解析后先保存在 `model_actions`，LLM 流结束时再做确定性 fallback 和语义安全仲裁。

因此机器人可能先说“好的”，随后动作才进入 Guard。这降低首音频延迟，但也意味着 speech 不是执行成功确认。真正执行结果要看 Action result/ACK。

## 9. 延迟指标

在线 `LatencyTracker` 记录：

- LLM request -> first token；
- ASR final -> first token；
- first TTS text -> first audio。

离线 `OfflineLatency` 记录：

- silence -> ASR final；
- LLM start -> first token；
- turn start -> first audio；
- turn complete。

缺失时间戳返回 `None`，不会伪造 0 ms。目标是否达成只对已测量值判断。少量本机测试是当前环境证据，不是生产 SLA。

## 10. 失败与资源释放

在线：

- LLM/TTS 异常 -> state `error`；
- 向 queue 放 sentinel；
- TTS thread 35 秒仍未停止 -> timeout；
- finally 释放 `_busy` 并回 listening。

离线：

- 任一 worker 异常 -> `errors`；
- abort 两个 buffer；
- 正常路径等待两个 worker 排空，60 秒未结束则失败；
- finally 释放 busy。

当前 daemon turn thread 在节点 shutdown 时没有统一 join；进程结束会终止它。生产化可增加 cancellation token 和受控线程生命周期。

## 11. 面试回答模板

**问题：怎样降低语音回答延迟？**

我没有等待 LLM 完整响应，而是用增量标签解析器把 speech 和 action 分离。speech 的安全前缀按标点或最大长度切句，立即送到独立 TTS worker；action 必须等闭标签和完整 JSON 后才暴露，避免半个 token 形成命令。在线 TTS 复用持久 WebSocket，LLM token、TTS 合成和 PCM 播放可以重叠；离线 VITS 不是原生流式，所以按句合成后切成 80 ms PCM，用容量为 2 的双缓冲形成三级流水线和背压。busy 锁阻止并发 turn 交错。项目分别记录首 token、首音频和整轮时间，但这些是当前机器测量，不是生产 SLA。

## 12. 自测

1. 为什么 speech 可以增量释放，action 必须等闭标签？
2. `_busy` 的 check/set 为什么要放进同一个锁？
3. 离线 TTS 为什么叫伪流式？
4. `close` 和 `abort` 对消费者有什么不同？
5. 机器人已经说“好的”为什么不等于动作执行成功？
