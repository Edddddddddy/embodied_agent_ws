# 源码深度讲解：阅读索引

这套文档面向两件事：第一，真正理解当前源码如何运行；第二，能够在面试中从结论讲到实现、取舍、证据和边界。内容以 2026-07-13 的 `/home/ubuntu/embodied_agent_ws` 当前源码为准，不把规划中的能力写成已实现能力。

## 建议学习顺序

| 阶段 | 文档 | 学完应能回答 |
| --- | --- | --- |
| 1 | [00-system-code-map.md](00-system-code-map.md) | 一条语音命令经过哪些进程、对象和 ROS 接口？ |
| 2 | [01-ros-interaction-models.md](01-ros-interaction-models.md) | Topic、Service、Action、Lifecycle 各解决什么问题？QoS 为什么不同？ |
| 3 | [02-audio-vad-kws.md](02-audio-vad-kws.md) | PCM 如何采集、回声如何消除、VAD 如何判端点、项目是否真的做了降噪？ |
| 4 | [03-asr-session-and-queue.md](03-asr-session-and-queue.md) | ASR final 如何通过唤醒、去重、补全、FIFO 和急停抢占？ |
| 5 | [04-nlu-llm-and-tts.md](04-nlu-llm-and-tts.md) | 为什么固定命令优先本地 NLU？LLM 流如何安全解析？伪流式 TTS 如何并行？ |
| 6 | [05-action-safety-and-scheduling.md](05-action-safety-and-scheduling.md) | 为什么模型不能发 `/cmd_vel`？Guard、outbox、scheduler 如何形成安全闭环？ |
| 7 | [06-simulation-bt-and-controller.md](06-simulation-bt-and-controller.md) | Action server、BT、pluginlib、控制器和雷达停车如何协作？ |
| 8 | [07-nav2-navigation.md](07-nav2-navigation.md) | “去门口”如何变成 `NavigateToPose`？取消和迟到回调如何处理？ |
| 9 | [08-lifecycle-launch-and-readiness.md](08-lifecycle-launch-and-readiness.md) | 节点为何要 Lifecycle？启动顺序、健康心跳和 readiness 如何保证？ |
| 10 | [09-memory-speaker-and-hardware.md](09-memory-speaker-and-hardware.md) | 声纹、用户快照、长期记忆、UART/SPI 协议与 watchdog 如何实现？ |
| 11 | [10-debugging-and-test-evidence.md](10-debugging-and-test-evidence.md) | 一次故障如何逐层定位？哪些测试能证明哪些事实？ |
| 12 | [11-question-bank-with-answers.md](11-question-bank-with-answers.md) | 面试追问如何从代码证据回答？ |
| 13 | [12-key-technical-points.md](12-key-technical-points.md) | 如何把整个项目凝练成关键技术点？ |

## 三种使用方式

### 第一次学习

按 00 到 10 顺序阅读。每章先看“先给结论”，再沿“源码调用链”打开文件，最后不看答案完成“自测问答”。遇到不理解的类，先确认它拥有哪一份状态，而不是先钻进每一行语法。

### 面试前速查

先读 [12-key-technical-points.md](12-key-technical-points.md)，再读 [11-question-bank-with-answers.md](11-question-bank-with-answers.md)。重点记住每个结论后面的源码对象、ROS 接口和事实边界。

### 对着源码走读

按下面这条主线打开文件：

```text
audio_frontend_node.cpp
  -> AgentRosIo subscriptions
  -> AgentControlPlane.accept_transcript()
  -> AgentApplicationRuntime.accept_transcript()/run_queued_turn()
  -> AgentControlPlane.enqueue_command()
  -> StreamingTurnRuntime / preparsed_actions
  -> SequentialActionPublisher.publish()
  -> ActionGuardNode::on_candidate()
  -> ActionScheduler::enqueue()
  -> TypedActionBridgeNode::dispatch_goal()
  -> SimulationControlNode::handle_accepted()
  -> ActiveActionRuntime::update()
  -> GazeboRobotExecutor / Nav2RobotExecutor
```

## 阅读约定

- “领域层”指不依赖 ROS graph 也能测试的状态和决策逻辑。
- “Adapter”指把 provider、ROS 消息或硬件接口转换成领域接口的薄层。
- “拥有状态”表示该对象是这份状态的唯一权威修改者。
- `RobotCommand` 是跨节点动作契约，`ActionCommand` 是 Python 领域对象，两者不要混为一谈。
- “伪流式 TTS”表示短句级合成后分块发布，不等于模型原生逐帧生成。
- “AEC”是回声消除，不等于通用环境噪声抑制。

## 当前事实边界

- 当前真实实现了 NLMS AEC；`noise_suppression_enabled` 和 `auto_gain_enabled` 只是为未来 WebRTC enhancer 预留，运行时会告警且状态消息标记 `active=false`。
- Energy、WebRTC、Silero 都用于 VAD/端点检测。VAD 可以减少非语音误触发，但不能被描述成完整降噪算法。
- Gazebo executor 对语义导航提供可观测替代运动；只有 Nav2 executor 才真正发送 `NavigateToPose` 或 `FollowWaypoints` goal。
- UART/SPI transport、帧协议和伪终端测试已实现；当前 README 明确不宣称真实机器人硬件闭环。
- LoRA 流水线已准备但未完成项目训练，fallback 或工程出口分数不能写成模型准确率。

## 源码版本

本文档创建时仓库提交为 `7073e08 docs: record architecture audit and boundaries`。后续改动若涉及接口、状态所有权或默认参数，应同步更新本目录。
