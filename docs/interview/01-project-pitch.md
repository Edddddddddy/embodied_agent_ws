# 项目讲述稿

## 30 秒版本

我做的是一个基于 ROS 2 的端侧语音机器人控制系统，目标不是只跑通一个语音 demo，而是把“人说话”到“机器人安全执行动作”做成可测试闭环。链路从麦克风和 ASR 开始，经过 LLM 解析出结构化动作，再由 C++ ActionGuard 做 schema 校验、限幅、拒绝和急停，最后通过 ROS 2 Action、BehaviorTree.CPP 和 pluginlib executor 驱动 Gazebo 里的 TurtleBot3，后续也可以接 UART/SPI 硬件后端。项目同时支持在线 Qwen 链路和离线 ZipFormer + llama.cpp + Sherpa-TTS 链路，并用 ACK、`/cmd_vel`、`/odom`、BT 状态和分层 release gate 验证动作确实执行。

## 1 分钟版本

这个项目叫 `Embodied Voice Agent for ROS 2`，我把它定位成一个可复现的语音机器人控制工程原型。它解决的问题是：LLM 可以理解自然语言，但它的输出不能直接控制机器人，所以我设计了一条从语音输入到安全动作执行的 ROS 2 闭环。

具体链路是：麦克风音频先经过 C++ 声学前端做 AEC、VAD 和 0.4 秒静音断句；然后走在线 Qwen ASR/LLM/TTS，或者离线 ZipFormer、Qwen3-0.6B Q8 llama.cpp 和 Sherpa-TTS；LLM 输出 `<speech>` 和 `<action>`，动作先进入 C++ ActionGuard，经过白名单、schema、速度和时长限幅、安全拒绝，再转换成可信的 ROS 2 动作命令。执行层用 ROS 2 Action 表达可反馈、可取消、可超时的运动任务，用 LifecycleNode 管启动顺序，用 BehaviorTree.CPP 编排 Validate、Safety、Execute、Confirm，最后通过 pluginlib 切换 Gazebo executor 或 mock executor。

我比较看重验证闭环，不只看 topic 有没有发出去。验收里会同时检查 Action result、BT confirm、ACK、非零 `/cmd_vel` 和 Gazebo `/odom` 位移。当前记录里 mock、online、offline、Gazebo、语音到 Gazebo release gates 都通过，但我也明确标注了边界：LoRA 还没训练，性能数字是当前机器少量样本，实体硬件还需要目标机器人验收。

## 3 分钟版本

我这个项目的背景是，我想做一个“语音控制机器人”的工程原型，但不希望它停留在“LLM 生成一段 JSON，然后发个 topic”这种脆弱 demo。所以项目的核心问题是：如何把自然语言和大模型输出，变成机器人端可信、可取消、可观测、可验收的动作执行链路。

整个系统分成五层。第一层是声学输入，C++ PortAudio 采集 PCM，做 NLMS AEC、能量 VAD 和 0.4 秒静音断句，尽量把实时音频处理从 Python 和云 SDK 里隔离出来。第二层是模型交互，在线链路接 Qwen 实时 ASR、流式 LLM 和 TTS，离线链路接 sherpa-onnx ZipFormer、Qwen3-0.6B Q8/llama.cpp 和 Sherpa-TTS。第三层是对话和动作解析，Python 负责流式解析 `<speech>` 和 `<action>`，但这里只产生候选动作，不直接控制机器人。

第四层是我认为最关键的安全动作层。LLM 输出永远被视为不可信输入，必须先进 C++ ActionGuard。Guard 只接受白名单动作，比如 `move / turn / stop`，并检查字段、枚举、速度、角速度、持续时间和拒绝原因。这样做的原因是，机器人控制和模型生成之间必须有一个强边界；模型可以建议动作，但不能直接拥有 `/cmd_vel` 控制权。

第五层是 ROS 2 执行层。这里我没有把动作简单做成普通 topic，而是迁移成强类型 ROS 2 Action，因为运动任务天然需要 feedback、cancel、timeout 和 terminal result。Guard 和仿真执行器都是 C++ LifecycleNode，由 Nav2 lifecycle manager 管 configure 和 activate，避免节点还没准备好就开始执行。单次任务内部用 BehaviorTree.CPP 编排 `Validate -> Safety -> Execute -> Confirm`，执行后端用 pluginlib 抽象成 `RobotExecutor`，所以同一套 Action/BT 逻辑可以切 Gazebo、mock，后续也能接真实硬件。

验证方面，我专门避免只验证“消息发布成功”。一个动作通过必须同时看到 Action 成功、BT 到 `confirm/succeeded`、executor ACK、`/cmd_vel` 非零，以及 Gazebo `/odom` 产生物理位移。当前文档记录里，2026-07-02 的 release gate 覆盖 mock、online、offline、Gazebo、离线语音到 Gazebo、在线语音到 Gazebo；130 项 colcon 测试和 2 项仓库约束测试为零失败。性能上，当前 WSL 环境少量样本里在线热启动首 token 350-384 ms，TTS 首音频 222-242 ms，离线 Q8 CPU decode 34.10 token/s，离线语音全链 2.313 s。但我不会把这些写成生产 SLA，因为还缺固定硬件下的 100 轮 P50/P95。

项目的亮点不是说我训练了一个很强的模型，而是把语音、LLM、安全控制、ROS 2 Action、BT、仿真和测试验收打通成了一条工程链路，并且把模型能力和工程兜底区分清楚。比如原始 0.6B 模型在 8 条种子命令里只有 2 条通过，fallback 后系统链路是 7/8，所以我明确写成“fallback 提升系统可用性”，不把它包装成模型准确率。下一步如果继续做，我会优先补独立 KWS、真实机器人硬件验收、P95 延迟报告，以及把语音扰动做成可复现 benchmark。

## 10 秒极简版

我做了一个 ROS 2 语音机器人控制工程原型，把在线/离线语音识别、LLM 动作解析、C++ 安全 Guard、ROS 2 Action/BT 执行和 Gazebo 位移验收串成闭环，重点解决“大模型输出不能直接控制机器人”的工程安全问题。

## 面试官可能接着问的点

- 为什么模型不能直接发布 `/cmd_vel`？
- 为什么动作执行要用 ROS 2 Action，而不是 topic 或 service？
- Lifecycle 和 BehaviorTree.CPP 分别解决什么问题？
- 怎么证明机器人真的执行了动作，而不是只发了消息？
- 在线链路和离线链路有什么取舍？
- 项目里最值得改进的地方是什么？

## 讲述时的取舍

- 面试时间短：讲 1 分钟版本，重点放在“LLM 不可信 -> C++ Guard -> Action/BT -> Gazebo 验真”。
- 面试官偏 ROS 2：强调 Action、Lifecycle、QoS、namespace、pluginlib、diagnostics。
- 面试官偏 AI：强调 ASR/LLM/TTS 流式链路、fallback 边界、模型成绩和工程成绩分离。
- 面试官偏工程：强调 release gate、测试矩阵、可观测性、实体硬件待验收边界。
