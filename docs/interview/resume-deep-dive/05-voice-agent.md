# 语音与具身智能交互追问

## Q1. 从麦克风到机器人执行的完整链路是什么？

口述：

麦克风 PCM 先进入 C++ 音频前端，处理线程完成回声抵消、VAD 和端点检测。ASR 产生最终文本后，Python 会话层处理唤醒、重复文本、急停和连续命令队列。NLU 或 LLM 把文本变成结构化候选动作，C++ 再检查字段、参数和目标。合法命令通过 ROS 2 Action 交给行为树及 Gazebo 或 Nav2，最终结果按 `command_id` 回到会话。

源码主线：

- 音频采集：[audio_frontend_node.cpp](../../../src/embodied_agent_cpp/src/audio_frontend_node.cpp)
- ASR 端点：[asr_endpoint_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py)
- 会话应用：[agent_application_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py)
- 动作映射：[ros_action_transport.py](../../../src/embodied_agent_core/embodied_agent_core/ros_action_transport.py)
- C++ 校验：[action_validator.cpp](../../../src/embodied_agent_cpp/src/action_validator.cpp)

一句话总结：模型负责理解，规则层负责授权，机器人层负责执行和证明。

## Q2. PortAudio 回调为什么只允许复制和入队？

口述：

PortAudio 回调由音频系统按固定周期调用，若在其中执行 DSP、ROS 发布或网络请求，超时会直接造成采集断裂。项目回调只复制当前帧、压入有界队列并唤醒处理线程。回声抵消、VAD 和发布都在 worker 中完成。处理速度不足时丢最旧帧，以稳定当前交互延迟。

源码：[audio_frontend_node.cpp](../../../src/embodied_agent_cpp/src/audio_frontend_node.cpp)

```cpp
int input_callback(const void * input, unsigned long frames) {
  AudioFrame frame = copy_pcm(input, frames); // 固定量内存复制
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    if (input_queue_.size() >= 50) {
      input_queue_.pop_front();               // 丢旧帧，不累积陈旧延迟
    }
    input_queue_.push_back(std::move(frame));
  }
  input_cv_.notify_one();
  return paContinue;
}
```

边界：示例是等价简化，真实代码还维护统计信息和 PortAudio 参数。

## Q3. 项目中的“降噪”到底实现了什么？

口述：

当前 C++ 前端真正实现的是 NLMS 自适应回声抵消，主要去除扬声器播放的 TTS 回灌；随后使用 VAD 和端点检测稳定切句。配置中虽然保留 noise suppression 和 automatic gain 开关，但当前增强后端没有启用通用环境降噪和自动增益，状态会报告未激活。因此面试时应说“回声抵消和语音端点稳定”，不能泛化成完整降噪方案。

源码：

- 算法：[audio_processing.cpp](../../../src/embodied_agent_cpp/src/audio_processing.cpp)
- 参数状态：[voice_frontend_launch_contract.py](../../../src/embodied_agent_bringup/embodied_agent_bringup/voice_frontend_launch_contract.py)

概念区分：

| 环节 | 解决的问题 |
| --- | --- |
| AEC | 去除已知扬声器参考信号造成的回声 |
| Noise Suppression | 抑制风扇、环境底噪等未知噪声 |
| AGC | 调整整体响度 |
| VAD | 判断当前帧是否有人声 |
| Endpoint | 根据连续语音和静音判断一句话边界 |

## Q4. NLMS 回声抵消怎样运行？

口述：

播放线程把 TTS PCM 写入参考缓冲，处理线程取得麦克风帧和延迟对齐后的参考帧。自适应滤波器用当前权重估计回声，再用“麦克风减估计回声”得到误差信号。权重根据误差和参考信号能量更新，归一化可以降低音量变化导致的步长不稳定。无播放参考时不应盲目更新权重。

源码：[audio_processing.cpp](../../../src/embodied_agent_cpp/src/audio_processing.cpp)

```text
estimated_echo = w 与 reference 的点积
clean_sample   = microphone - estimated_echo
w_new          = w + step * clean_sample * reference
                       / (epsilon + reference_energy)
```

项目还把 24 kHz TTS 参考重采样到 16 kHz 麦克风采样率，并补偿约定的参考延迟。`mutex_` 保护播放线程写参考与处理线程更新滤波状态。

边界：NLMS 对非线性扬声器失真、时变延迟和强环境噪声能力有限，生产方案通常会评估 WebRTC AEC/NS/AGC 或硬件回声路径。

## Q5. VAD 和端点检测为什么要分开？

口述：

VAD 对单帧或短窗口给出是否为语音，端点检测根据连续帧形成“开始说话”和“结束说话”事件。项目设置语音开始防抖、尾部静音、最短句长和最长句长，避免一个瞬时噪声开句，也避免用户停顿一下就把一句话切成两段。Energy、Silero 和 WebRTC 可以作为不同 VAD provider，但端点语义保持一致。

源码：

- C++ 端点：[audio_processing.cpp](../../../src/embodied_agent_cpp/src/audio_processing.cpp)
- sidecar 部署：[voice_frontend_launch_contract.py](../../../src/embodied_agent_bringup/embodied_agent_bringup/voice_frontend_launch_contract.py)

运行：VAD 连续为真达到开始阈值后发布 `speech_started`；随后连续静音达到尾部阈值才发布 `speech_ended`；ASR 收到端点后提交 final。

## Q6. 为什么 `speech_ended` 后还要延迟几百毫秒提交 ASR？

口述：

真实语音结尾可能有短停顿，立即提交容易丢掉“秒”“度”等尾词。项目允许端点事件后延迟提交；如果用户在延迟期间继续说话，就取消旧 timer，把两段仍当作同一句。generation 保证停用或新一代语句开始后，旧 timer 即使触发也不会访问当前 provider。

源码：[asr_endpoint_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py)

```python
generation = self._generation
timer = Timer(delay, self._perform_commit, args=(source, generation))

def _perform_commit(source, generation):
    if generation != self._generation:
        return False             # 旧端点已经失效
    commit_to_provider()
```

同一端点可能由两个 Topic 镜像发布，运行时还在短窗口内去除跨来源重复事件。

## Q7. 在线和离线 ASR 的实现差异是什么？

口述：

在线 ASR 维持 WebSocket 会话，音频帧经过 Base64 后持续发送，本地 VAD 决定何时调用 commit；服务端分别回调 partial 和 final。离线 ASR 使用 sherpa-onnx 流式 ZipFormer，同一个 worker 线程中送入浮点波形并循环解码；提交时补少量尾部静音，把编码器剩余上下文推出，再重建 stream。两种传输不同，但共用端点去重、延迟和生命周期控制。

源码：

- 在线：[qwen_asr.py](../../../src/embodied_online_agent/embodied_online_agent/providers/qwen_asr.py)
- 离线：[sherpa_asr.py](../../../src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_asr.py)

边界：在线模式依赖网络和云端配额；离线模式依赖本机算力、模型和热词配置。接入成功不等于所有环境下识别准确率达标。

## Q8. partial 和 final 分别怎样使用？

口述：

partial 用于展示当前识别进度和观察稳定性，不直接触发机器人动作；final 才进入会话和命令链路。这样可避免模型在一句话尚未说完时执行“向前走”，随后才识别出“不要”。final 还会经过文本稳定、空文本和已知截断模式处理，只有满足规则的文本才能进入任务解析。

源码：

- 在线回调：[qwen_asr.py](../../../src/embodied_online_agent/embodied_online_agent/providers/qwen_asr.py)
- 文本稳定：[transcript_stabilizer.py](../../../src/embodied_agent_core/embodied_agent_core/transcript_stabilizer.py)
- 唯一应用入口：[agent_application_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py)

设计原则：展示信号和控制信号分开；低延迟不能以提前执行不完整指令为代价。

## Q9. 连续语音输入怎样避免重叠执行？

口述：

会话层先过滤语气词、重复文本、未唤醒输入和过期命令，普通指令进入有界队列。执行运行时只有一个 worker 取得 busy 所有权，逐条调用模型和动作链；单条异常会产生失败事件，但 `finally` 复位 busy 后继续下一条。急停不进普通队列，而是清队列并取消当前动作批次。

源码：

- 会话与去重：[continuous_voice.py](../../../src/embodied_agent_core/embodied_agent_core/continuous_voice.py)
- 应用入口：[agent_application_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py)
- worker：[agent_execution_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_execution_runtime.py)

取舍：串行执行牺牲同时处理多个运动命令的吞吐，但机器人控制本身需要顺序明确。聊天类请求若以后需要并行，应与运动任务使用不同调度域。

## Q10. 用户上下文为什么在入队时冻结？

口述：

连续命令可能等待几秒才执行，期间可能识别到新的说话人或修改偏好。若 worker 消费时再读取“当前用户”，旧命令可能错误使用新用户的速度偏好和记忆。项目在入队时保存不可变的 `UserContextSnapshot` 和 provider turn context，执行时恢复这份快照，并把结果记录回同一次交互。

源码：

- 上下文冻结：[agent_application_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py)
- 用户上下文：[user_context_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/user_context_runtime.py)

这是异步系统中的快照一致性：命令应携带被接受时的上下文，而不是未来某一时刻的全局状态。

## Q11. LLM 输出如何变成机器人动作？

口述：

模型输出先由 NLU 或流式 turn 适配成 `ActionCommand`，只包含允许的动作名和参数；再映射成自定义 `RobotCommand` 候选。C++ 检查动作类型、字段互斥、有限数、速度、时长、地点和 waypoint 数量，必要时限幅或拒绝。只有校验后的消息才进入动作调度与 Action server。

源码：

- 动作值对象：[types.py](../../../src/embodied_agent_core/embodied_agent_core/types.py)
- 文本解析：[command_nlu.py](../../../src/embodied_agent_core/embodied_agent_core/command_nlu.py)
- ROS 映射：[ros_action_transport.py](../../../src/embodied_agent_core/embodied_agent_core/ros_action_transport.py)
- C++ 校验：[action_validator.cpp](../../../src/embodied_agent_cpp/src/action_validator.cpp)

重要回答：结构化输出只是减少格式不确定性，不等于安全。模型仍可能选择错误动作或给出越界值，所以规则层和执行层必须独立校验。

## Q12. 为什么模型不能直接发布 `/cmd_vel`？

口述：

模型不知道传感器是否过期、前方是否近障、底盘速度上限和当前任务所有权，也无法可靠处理取消和迟到结果。项目让模型只生成候选动作，C++ 规则层限制动作和参数，调度器保证单活动任务，执行端再结合 LaserScan、Action 和 Nav2 做物理控制。这样即使模型输出异常，也不会直接变成无边界速度。

代码调用：

```text
LLM 文本
ActionCommand
RobotCommand 候选
C++ 字段与范围校验
ActionScheduler
ExecuteRobotCommand
Gazebo 或 Nav2
```

边界：规则层能限制已知动作空间，但不能证明模型理解了用户真实意图；高风险动作还应增加确认、权限和环境条件。

## Q13. 在线与离线 TTS 怎样处理？

口述：

在线 TTS 使用持久 WebSocket，会话启动时提前连接，文本分块发送后 commit，音频 delta 到达就交给
播放回调；操作锁保证同一连接不会交叉合成两段回复。离线稳定默认是 Sherpa-TTS，SummerTTS
通过 ROS Service 调用常驻 C++ 模型，作为可选实现。播放线程同时把 PCM 送入扬声器和 AEC reference。

源码：

- 在线 TTS：[qwen_tts.py](../../../src/embodied_online_agent/embodied_online_agent/providers/qwen_tts.py)
- 离线默认：[sherpa_tts.py](../../../src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_tts.py)
- 离线 client：[summer_tts_ros.py](../../../src/embodied_offline_agent/embodied_offline_agent/providers/summer_tts_ros.py)
- C++ 服务：[summer_tts_service_node.cpp](../../../src/embodied_agent_cpp/src/summer_tts_service_node.cpp)

取舍：在线流式首包更快但依赖网络；离线更可控但受端侧算力限制。TTS 播放必须进入 AEC reference，否则机器人容易把自己的回复再次识别成用户指令。

## Q14. LLM 流式输出出错时为什么不能总是自动重试？

口述：

如果连接在首个 token 前失败，可以安全重试；一旦部分 token 已经交给上游或 TTS，再静默重试会把半截旧回复和新回复拼接，甚至重复生成动作。离线 llama.cpp provider 记录是否已经输出 token，只有尚未输出且未超过重试次数时才重试。超时、连接错误和 HTTP 错误会转换成可读异常及指标。

源码：[llama_cpp.py](../../../src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py)

```python
emitted_any_token = False
try:
    for delta in response:
        emitted_any_token = True
        yield delta
except Exception:
    if emitted_any_token or final_attempt:
        raise                    # 已有可见副作用，不能透明重放
```

这是通用的幂等性问题：有外部可见副作用后，重试必须由更高层明确决定。

## Q15. 语音链路怎样验证？

口述：

单元测试分别验证 endpoint 去重、generation 取消、连续队列、动作 ID 和 provider 错误路径；mock smoke 验证 ROS Topic 与 Action 接线；真人麦克风测试再观察 VAD、ASR final、响应延迟和回声回灌。性能指标包括首 token 时间、总耗时、丢帧和解码速度，但当前机器上的少量样本不能写成生产 SLA。

测试入口：

- [test_asr_endpoint_runtime.py](../../../src/embodied_agent_core/test/test_asr_endpoint_runtime.py)
- [test_action_sequence.py](../../../src/embodied_agent_core/test/test_action_sequence.py)
- [smoke_test_audio_endpoint.sh](../../../scripts/smoke_test_audio_endpoint.sh)
- [VOICE_AGENT.md](../../learning/VOICE_AGENT.md)

## Q16. RAG 为什么不能进入机器人控制关键路径？

口述：

知识文档可能过时、缺失或含提示注入，检索和生成也增加不确定延迟。项目先用本地 NLU 判断是否为
明确控制命令；控制命令零检索并继续走 typed Action/ActionGuard。只有部署、原理和排障等知识问题
才进入 RAG，证据不足时允许澄清，但不能把检索文本直接转换成机器人授权。

源码：[prompt_context.py](../../../src/embodied_agent_core/embodied_agent_core/prompt_context.py)

## Q17. 在线和离线 Prompt 怎样避免两份实现漂移？

口述：

`PromptContextAssembler` 统一组合系统约束、用户画像、短期历史和当前检索证据。它返回模型实际看到的
消息、检索来源和耗时；在线/离线 turn runtime 只消费这个结果。离线的 `/no_think` 作为构造参数，
不再由节点临时拼接。RAG 正文只用于当前请求；ConversationMemory 保存干净用户原文，并把非控制
turn 的模型输出收敛为纯 `<speech>`，避免被拦截 action 或知识注入跨轮传播。

进一步的论文和部署取舍见 [11-voice-deployment-rag.md](11-voice-deployment-rag.md)。
