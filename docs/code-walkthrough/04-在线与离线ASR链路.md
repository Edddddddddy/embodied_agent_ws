# 04 在线与离线 ASR 链路

## 源码导航

| 文件与关键行 | 符号 | 观察点 |
|---|---|---|
| `src/embodied_online_agent/embodied_online_agent/providers/base.py:5` | `AsrProvider` | provider 最小契约 |
| `src/embodied_online_agent/embodied_online_agent/providers/qwen_asr.py:24` | `QwenRealtimeAsr.start` | WebSocket session 与 callback |
| `src/embodied_online_agent/embodied_online_agent/providers/qwen_asr.py:75` | `push_audio/commit` | 在线 push/commit |
| `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py:190` | `_run_asr` | 单 worker 事件消费 |
| `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_asr.py:42` | `push_audio/commit` | ZipFormer stream 生命周期 |
| `src/embodied_online_agent/embodied_online_agent/recognition_retry.py:18` | `RecognitionRetryTracker` | 失败反馈计数 |

## 1. ASR 在系统中的位置

ASR 只负责把 PCM 转成文本，不负责唤醒、动作解析或控制机器人。输入来自 `/audio/clean_pcm`，断句信号来自 `/audio/silence_timeout`，输出分为 partial 和 final：

```text
clean PCM -> push_audio -> partial text
silence timeout -> commit -> final text
final text -> WakeWordGate -> Agent turn
```

把“音频分帧”和“提交一句话”分开，是在线和离线 provider 能共用同一上层编排的关键。

## 2. Provider 接口

`providers/base.py` 定义：

```python
class AsrProvider(ABC):
    def start(on_partial, on_final): ...
    def push_audio(pcm16): ...
    def commit(): ...
    def stop(): ...
```

Node 只依赖接口，不知道底层是 DashScope WebSocket、Sherpa ZipFormer 还是 mock。这个抽象的价值不是“用了面向对象”，而是把变化频繁的模型 SDK 隔离在 adapter 中，使 ROS Topic 和对话状态机保持稳定。

## 3. 在线 ASR

源码：`providers/qwen_asr.py`。

### 3.1 启动

`start()` 做四件事：

1. 检查 `DASHSCOPE_API_KEY`；
2. 构造 SDK callback，将 partial/final 事件转成普通 Python callback；
3. 建立 WebSocket conversation；
4. 关闭服务端 turn detection，声明 PCM、语言和采样率。

```python
self.conversation.update_session(
    output_modalities=[MultiModality.TEXT],
    enable_turn_detection=False,
    enable_input_audio_transcription=True,
    transcription_params=TranscriptionParams(
        language=self.language,
        sample_rate=self.sample_rate,
        input_audio_format="pcm",
    ),
)
```

关闭服务端 turn detection 意味着句尾由本地 VAD 决定。优点是在线/离线使用同一个 0.4 秒断句规则；代价是本地能量 VAD 的误判会直接影响云端 final 时机。

### 3.2 push 与 commit

每个 clean PCM frame 被 base64 编码后 append：

```python
self.conversation.append_audio(base64.b64encode(pcm16).decode("ascii"))
```

静音事件调用 `conversation.commit()`。SDK 之后通过 callback 返回 completed transcript，最终进入 `_on_asr_final()`。

### 3.3 忙碌策略

`OnlineAgentNode._on_clean_audio()` 在 `_busy` 时直接 return，`_on_silence_timeout()` 也不 commit。即 Agent 正在思考/说话时不继续识别，避免自己的 TTS 再触发新一轮命令。这是半双工策略；即使有 AEC，也没有实现用户随时打断播报的 barge-in。

## 4. 离线 ASR

源码：`OfflineAgentNode` 和 `providers/sherpa_asr.py`。

Sherpa 的 stream 要求所有操作在一个 worker thread 中执行。ROS subscription 回调不直接调用模型，而是向 `_asr_events` 放事件：

```text
ROS callback -> Queue[("audio", bytes) | ("commit", None)]
            -> _run_asr single worker
            -> SherpaZipformerAsr
```

这样避免 ROS executor 和模型内部状态并发访问，也让回调保持短小。

### 4.1 队列满时

队列容量为 64。满时 `_enqueue_asr()` 丢最旧事件，再放新事件。按 20 ms PCM 估算，64 帧约 1.28 秒积压。选择丢旧音频是为了避免延迟无限增长。

代码的 `preserve=True` 表达 commit 更重要，但当前满队列分支仍是“先丢一个旧事件再放当前事件”；只有极端并发下第二次操作异常才记录 commit 无法入队。面试应描述实际行为，不把 `preserve` 说成严格优先队列。

### 4.2 ZipFormer stream

`push_audio()`：

1. 用 NumPy 将 little-endian int16 转成 float32 `[-1, 1]`；
2. `accept_waveform()`；
3. 当 recognizer ready 时循环 `decode_stream()`；
4. partial 变化时回调。

`commit()`：

1. `input_finished()`；
2. 解码剩余帧；
3. 读取 final；
4. 创建全新 stream；
5. 清空 `_last_partial`。

必须创建新 stream，否则下一句话会接在旧上下文后面。

## 5. 热词、唤醒词与 VAD 的区别

| 机制 | 工作层 | 解决的问题 |
|---|---|---|
| VAD | 音频 | 什么时候开始/结束一句话 |
| ASR hotword biasing | 解码 | 让“小智”等词在声学歧义中更容易被选中 |
| WakeWordGate | 文本 | final transcript 是否授权进入对话 |
| 独立 KWS | 音频/模型 | 低功耗持续检测唤醒词，当前未实现 |

ZipFormer 配置将 `hotwords_file`、score、modeling unit 和 max active paths 传给 recognizer。这是上下文偏置，不保证唤醒词一定正确，更不等同于专用 KWS。

## 6. 识别失败与重试

ASR final 仍可能没有唤醒词。`RecognitionRetryTracker.failed()` 发布：

```json
{
  "status": "retry",
  "reason": "wake_word_not_detected",
  "attempt": 1,
  "max_attempts": 3
}
```

attempt 达到 max 后会循环到 1，不会锁死识别。成功后 reset 为 0。这个“重试”是用户反馈计数，不是自动重新请求同一段音频。

## 7. 在线与离线对比

| 维度 | 在线 Qwen | 离线 ZipFormer |
|---|---|---|
| 网络 | 必需 | 不需要 |
| 隐私 | 音频发往云端 | 音频留在设备 |
| 算力 | 云端承担模型 | 本机 CPU/内存承担 |
| API 状态 | WebSocket callback | 单 worker 持有 stream |
| 断句 | 本地 silence 后 commit | 本地 silence 后 input_finished |
| 热词 | 当前 adapter 未配置 | 支持 contextual biasing |
| 故障 | key、网络、SDK、限流 | 模型文件、CPU backlog、native wheel |

共同上层接口使两种模式能复用唤醒、LLM 协议、Guard 和执行链。

## 8. 故障定位

```text
/audio/clean_pcm 无数据
  -> 查 AudioFrontend/声卡
有 clean PCM，无 partial
  -> 查 ASR start、key/model、QoS、worker backlog
有 partial，无 final
  -> 查 VAD 和 /audio/silence_timeout、commit
有 final，不进入 thinking
  -> 查 wake word gate 和 busy 状态
```

观察命令：

```bash
ros2 topic hz /audio/clean_pcm
ros2 topic echo /audio/silence_timeout
ros2 topic echo /agent/asr_partial
ros2 topic echo /agent/asr_final
ros2 topic echo /agent/recognition_feedback
```

## 9. 面试回答模板

**问题：在线和离线 ASR 怎么统一？**

我把 ASR 抽象为 start、push_audio、commit 和 stop。C++ 音频前端统一发布 16 kHz PCM，并在检测到连续 400 ms 静音时发送 commit 事件。在线 adapter 把 PCM base64 后追加到 Qwen WebSocket，commit 由云端产生 final；离线 adapter 用单独 worker 串行驱动 Sherpa ZipFormer stream，commit 时 input_finished、解码尾帧并重建 stream。离线必须经过事件队列，因为原生 recognizer 状态不应被多个 ROS callback 并发访问。两种实现上层共享唤醒、重试、LLM 和动作安全链路。当前是半双工，Agent 忙时丢弃新音频，不支持播报中用户打断。

## 10. 自测

1. 为什么 online adapter 关闭服务端 turn detection？
2. 离线 ASR 为什么不能在 subscription 回调里直接 decode？
3. 64 个 20 ms 帧代表多少积压时间？
4. hotword、wake gate 和 KWS 有什么区别？
5. “重试三次”为什么不等于把同一音频识别三次？
