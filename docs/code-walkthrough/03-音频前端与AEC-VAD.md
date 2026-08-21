# 03 音频前端与 AEC/VAD

## 源码导航

| 文件与关键行 | 符号 | 观察点 |
|---|---|---|
| `src/embodied_agent_cpp/src/audio_frontend_node.cpp:24` | `AudioFrontendNode` | 参数、ROS 接口、声卡资源 |
| `src/embodied_agent_cpp/src/audio_frontend_node.cpp:140` | `input_callback` | 实时 callback 只搬运 PCM |
| `src/embodied_agent_cpp/src/audio_frontend_node.cpp:166` | `processing_loop` | AEC、publish、VAD、断句 |
| `src/embodied_agent_cpp/src/audio_frontend_node.cpp:214` | `playback_loop` | reference 与声卡播放 |
| `src/embodied_agent_cpp/src/audio_processing.cpp:109` | `NlmsEchoCanceller::process` | NLMS 逐样本更新 |
| `src/embodied_agent_cpp/src/audio_processing.cpp:19` | `EnergyVad::is_speech` | RMS 能量判断 |
| `src/embodied_agent_cpp/src/audio_processing.cpp:42` | `SilenceDetector::update` | 400 ms 状态机 |

## 1. 先纠正“降噪”这个词

当前音频前端实现三件事：

1. NLMS Acoustic Echo Cancellation：利用“机器人自己正在播放什么”估计并抵消回声。
2. Energy VAD：用一帧音频的 RMS 能量判断是否像语音。
3. Silence Detection：听到语音后连续静音 0.4 秒，通知 ASR 提交当前 utterance。

它没有实现：

- 对风扇、车轮、键盘等非参考噪声的频谱抑制；
- 多麦克风波束形成；
- 深度学习降噪；
- WebRTC NS/AGC 的完整处理链。

因此准确说法是“回声消除和端点检测前端”，不是“解决了所有环境降噪”。

## 2. 数据与线程路径

源码入口：`src/embodied_agent_cpp/src/audio_frontend_node.cpp`。

```text
PortAudio input callback
  -> input_queue_ (最多 50 帧，满时丢最旧)
  -> processing_thread_
     -> NlmsEchoCanceller::process
     -> publish /audio/clean_pcm
     -> EnergyVad::is_speech
     -> SilenceDetector::update
     -> publish /audio/silence_timeout

/audio/tts_pcm subscription
  -> playback_queue_ (最多 100 块，满时丢最旧)
  -> playback_thread_
     -> echo_canceller_.add_reference
     -> Pa_WriteStream
```

关键设计是 PortAudio callback 只复制 PCM 和维护有界队列。它不做日志、ROS publish、网络请求或 DSP 长计算，因为音频 callback 超时会造成丢帧和爆音。

## 3. PCM 表示

麦克风默认参数：

```text
16000 samples/s
mono
signed int16
20 ms/frame
```

每帧样本数：

```text
16000 * 20 / 1000 = 320 samples
```

字节数为 `320 * sizeof(int16_t) = 640 bytes`。代码用 `memcpy` 在 `vector<int16_t>` 与 `UInt8MultiArray.data` 间转换。消息没有显式携带 sample rate、channel、endianness，所以生产者和消费者必须通过配置约定格式；这是当前接口的隐式契约。

## 4. 为什么使用有界队列

callback 生产速度固定为每 20 ms 一帧。如果处理线程偶尔变慢，无界队列会让内存和端到端延迟持续增长。

代码策略：

```cpp
if (input_queue_.size() >= kMaximumInputFrames) {
  input_queue_.pop_front();
  ++dropped_input_frames_;
}
input_queue_.push_back(std::move(frame));
```

50 帧约等于 1 秒音频。满时丢最旧帧意味着“保实时性，不保完整历史”。这适合实时对话，但可能截断语句。`dropped_input_frames_` 当前只计数，没有发布 diagnostics，是可以继续增强的可观测性缺口。

## 5. NLMS 回声消除

源码：`audio_processing.cpp` 的 `NlmsEchoCanceller::process()`。

设：

- `x[n]`：扬声器参考信号；
- `w[n]`：自适应滤波器权重；
- `d[n]`：麦克风采样，包含人声、环境声和扬声器回声；
- `y[n] = w^T x`：预测回声；
- `e[n] = d[n] - y[n]`：消除后的输出。

代码逐样本计算：

```cpp
predicted += weights_[i] * history_[i];
energy += history_[i] * history_[i];
error = desired - predicted;
scale = step_ * error / energy;
weights_[i] += scale * history_[i];
```

对应 NLMS 更新：

```text
w(n+1) = w(n) + μ * e(n) * x(n) / (ε + ||x(n)||²)
```

与普通 LMS 相比，除以参考信号能量使步长对音量变化不那么敏感。`1e-6` 防止静音时除零。

### 5.1 参数意义

| 参数 | 默认值 | 作用 | 过小/过大风险 |
|---|---:|---|---|
| `aec_taps` | 64 | 建模回声路径的 FIR 长度 | 太短无法覆盖回声，太长计算量和收敛时间增加 |
| `aec_step` | 0.35 | 权重更新速度 μ | 太小收敛慢，太大可能发散或失真 |
| `aec_delay_ms` | 80 | 播放参考到麦克风回声的粗延迟 | 不匹配会使参考与回声错位 |
| microphone rate | 16 kHz | ASR 输入速率 | 必须与采集和 ASR 一致 |
| reference rate | 24 kHz | 在线 TTS PCM 速率 | 不同速率需重采样 |

64 taps 在 16 kHz 下只覆盖约 4 ms FIR 历史；另外代码用 `delay_samples_` 插入固定 80 ms 延迟。真实房间有更长混响，多路径和声卡时钟漂移，因此目标硬件仍需实测。

### 5.2 reference 的来源

playback thread 在写扬声器前调用：

```cpp
echo_canceller_.add_reference(samples);
Pa_WriteStream(output_stream_, samples.data(), samples.size());
```

这提供“即将播放”的数字参考。reference buffer 首次插入固定延迟，再按麦克风采样率重采样。若 reference 数据不足一个 microphone frame，`process()` 原样返回麦克风帧，避免用不完整参考做错误抵消。

### 5.3 重采样

`resample_reference()` 使用线性插值：

```text
ratio = microphone_rate / reference_rate = 16000 / 24000 = 2/3
output_size = floor(input_size * ratio)
```

线性插值实现简单、延迟低，但没有高质量带限滤波。它足以作为工程原型，不应描述成专业音频 resampler。

### 5.4 双讲问题

当用户说话与扬声器同时播放时，NLMS 可能把近端语音视作误差。理想 AEC 需要 double-talk detection，避免在双讲期间错误更新权重；当前实现没有该机制。面试时应主动说这是硬件验收和后续增强点。

## 6. Energy VAD

`EnergyVad::is_speech()` 先将 int16 归一化到 `[-1, 1]`，计算：

```text
RMS = sqrt(sum(sample²) / N)
speech = RMS >= threshold
```

默认 threshold 是 `0.018`。它计算便宜、可确定性测试，但不能区分“人声”和“同等能量的电机噪声”。在固定安静环境中可用；噪声底变化时应考虑自适应阈值、频谱特征或专用 VAD。

## 7. 静音断句状态机

`SilenceDetector` 有三个状态变量：

- `heard_speech_`：本轮是否先听到过语音；
- `silence_seconds_`：之后累计静音；
- `emitted_`：是否已经发出一次 timeout。

状态变化：

```text
初始静音 -> 不触发
speech=true -> heard=true, silence=0, emitted=false
随后静音 -> silence += frame duration
silence >= 0.4s -> 触发一次，回到等待语音
继续静音 -> 不重复触发
```

20 ms 帧需要连续 20 帧静音达到 400 ms。测试 `SilenceDetectorTest.EmitsOnceAfterFourHundredMilliseconds` 明确验证了这一点。

为什么不在纯静音每 400 ms commit：那会不停产生空 ASR 结果和网络请求。

## 8. 线程同步与关闭

input/playback 队列各有独立 mutex 和 condition variable。AEC 内部还有 mutex，因为 reference 由 playback thread 写，microphone 由 processing thread 读写滤波状态。

析构关闭顺序：

1. 停 input stream，阻止 callback 继续生产；
2. `running=false`；
3. notify 两个 condition variable；
4. join processing/playback threads；
5. close streams；
6. `Pa_Terminate()`。

先 join 再销毁成员，避免后台线程访问已释放对象。`running_` 使用 atomic，队列使用 mutex；两者保护的对象不同。

## 9. 测试证据

`test_audio_processing.cpp` 覆盖：

- 零样本为静音，固定幅值超过 VAD threshold；
- 先有语音再静音 400 ms，只触发一次；
- 没有 reference 时麦克风原样输出；
- 重复纯回声训练 20 轮后，输出能量低于输入的 10%。

最后一个测试证明算法能在理想合成信号上学习，不证明真实房间、真实扬声器、双讲和时钟漂移下达到同样指标。

## 10. 故障推演

| 现象 | 可能原因 | 观察方法 |
|---|---|---|
| ASR 不断听到机器人自己的声音 | reference 未进入、delay 不准、AEC 未收敛 | echo topic、扬声器开关、录制 clean/raw 对比 |
| 一直不 commit | VAD threshold 太低导致噪声总被判 speech | 记录每帧 RMS、提高 threshold |
| 人说一句被切成多段 | threshold 太高或 silence timeout 太短 | 查看 partial/final 时间线 |
| 音频越来越延迟 | 处理/播放消费者跟不上 | 暴露 dropped/high-watermark 指标 |
| clean PCM 有周期断裂 | callback 队列满丢旧帧 | 发布 `dropped_input_frames_` diagnostics |

## 11. 面试回答模板

**问题：项目怎么做降噪？**

更准确地说，项目当前做的是声学回声消除和端点检测。扬声器播放的 TTS PCM 作为 reference，C++ 前端用 NLMS 自适应 FIR 估计回声路径，从麦克风信号中减去预测回声；由于 TTS 是 24 kHz、麦克风是 16 kHz，reference 先做线性重采样并补偿固定延迟。处理后的帧再经过 RMS 能量 VAD，只有先检测到语音、随后连续静音 400 ms 才通知 ASR commit。PortAudio callback 只把 PCM 放进有界队列，DSP 和 ROS publish 在工作线程执行，避免阻塞实时音频线程。边界是它没有通用频谱降噪、double-talk detection 或麦克风阵列能力，真实机器人仍需重新标定 delay、step 和 threshold。

## 12. 自测

1. 16 kHz、20 ms 一帧有多少样本和字节？
2. 为什么 NLMS 要除以 reference energy？
3. 没有 reference 时 `process()` 返回什么，为什么？
4. 为什么 VAD 不能被称为 ASR？
5. 为什么 callback 队列满时选择丢最旧而非阻塞？
6. 真实房间里合成测试通过仍可能失败的三个原因是什么？
