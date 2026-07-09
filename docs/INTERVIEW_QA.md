# 面试问答：ROS 2 / C++ 具身语音 Agent 项目

这份文档用于把项目讲得更聚焦。建议先看
[PROJECT_PRESENTATION_15MIN.md](PROJECT_PRESENTATION_15MIN.md)，再用本文准备追问。

## 1. 项目主线是什么？

项目主线是打通“语音输入 → ASR → Agent 动作解析 → C++ 安全校验 → ROS 2 Action
→ Gazebo/Nav2 仿真执行”的端到端链路。它不是单纯聊天机器人，也不是完整导航栈项目，
核心价值在于把大模型/语音能力放进 ROS 2 工程链路，并保留可测试、安全、可取消的执行边界。

## 2. 为什么不让 LLM 直接发布 `/cmd_vel`？

`/cmd_vel` 是底盘速度控制接口，直接暴露给 LLM 风险太高。项目让 Agent 只发布动作候选，
再由 C++ `ActionGuard` 做白名单、参数限幅和强类型转换，最后通过 ROS 2 Action 执行。
这样可以把模型的不确定性隔离在安全边界之外。

## 3. 为什么要用 ROS 2 Action，而不是 topic 或 service？

移动一秒、转九十度、目标点导航都不是瞬时请求，它们需要 feedback、取消和 result。
topic 适合广播事件，service 适合短请求，ROS 2 Action 更适合持续动作。因此本项目用
`RobotCommand.msg` 表达强类型命令，用 `ExecuteRobotCommand.action` 表达可取消执行。

## 4. C++ 部分体现在哪里？

核心 C++ 模块包括：

- `embodied_agent_cpp`：音频前端、ActionGuard、typed action bridge、硬件 mock、SummerTTS service。
- `embodied_simulation`：BehaviorTree.CPP、pluginlib executor、Gazebo/Nav2 执行层。
- C++ 单测覆盖动作校验、adapter、硬件协议、仿真控制、BT、pluginlib、Nav2 places。

## 5. Python 部分为什么还保留？

Python 更适合快速接 ASR/LLM/TTS provider、prompt、连续语音状态机、监控脚本和轻量 NLU。
项目不是盲目 C++ 化，而是把强实时/安全边界/ROS 执行层放在 C++，把模型编排和工具层放在
Python。这个分工更接近工程上的成本收益平衡。

## 6. 连续语音控制怎么保证不丢命令？

`ContinuousVoiceSession` 负责唤醒、休眠、filler 过滤和重复过滤；
`ContinuousCommandQueue` 负责 FIFO 排队；急停/停下是高优先级命令，会清空普通队列并抢占。
每条命令带 request/batch 信息，Action result 回来后再执行下一条，避免旧 result 误唤醒新任务。

## 7. “向右转，向前走一秒”怎么处理？

轻量 `CommandNLU` 会把一条 ASR final 识别成多个动作片段，例如 `turn -> move`。
这些动作按 `batch_id / batch_index / batch_size` 入队。低置信度时回退到现有 fallback/LLM
路径，所有动作最终仍经过 ActionGuard。

## 8. 离线链路的可信证据是什么？

当前有四类证据：

- `llama-cpp-preflight/smoke`：验证 llama-server、GGUF 模型和流式 chat。
- `offline-latency`：验证 LLM 首 token 和默认 TTS 首音频目标。
- `offline-sherpa-typed`：验证 Sherpa ASR/TTS 进入 typed Action 仿真控制闭环。
- `OFFLINE_RUNTIME_VERSIONS.md`：固定 llama.cpp、SummerTTS、sherpa-onnx 版本，避免第三方漂移。

## 9. SummerTTS 的定位是什么？

SummerTTS 是 C++ 离线 TTS runtime 接入展示点。项目已经提供命令行 provider 和常驻 C++
ROS service，其中 service 避免每句重新启动进程和加载模型。但当前 CPU 合成仍是秒级，
不能把它宣称为 `<300ms` 默认低延迟 TTS。低延迟 gate 仍以 Sherpa-TTS 路径为准。

## 10. Nav2 做到了什么，没做到什么？

已经做到：

- 语义地点 `door/desk/home`。
- `navigate_to / follow_waypoints / cancel_navigation`。
- fake Nav2 action bridge 验收。
- TurtleBot3/Nav2 bringup 入口。

还没做到：

- 完整 SLAM 建图流程。
- 动态地图/大规模目标点管理。
- planner/controller/localization failure 的细粒度可视化分析。

## 11. 真实语音为什么仍是最大风险？

真实麦克风受 WSLg/PulseAudio、环境噪声、回声、多人说话和 ASR 抖动影响。项目当前用
energy VAD、commit delay、短命令补全、profile 推荐和 monitor 兜底，但还没有把 WebRTC VAD、
Silero VAD 或声学 KWS 作为默认强依赖。因此演示前必须跑 `wsl-microphone-preflight` 和
`continuous-offline`。

## 12. 你会如何继续优化？

优先级从高到低：

1. 固定 demo world、录制演示脚本和一键 evidence 报告。
2. 接入成熟 VAD/KWS 默认方案，降低真实语音环境敏感性。
3. 建立离线 benchmark 报告：模型大小、tokens/s、首 token、动作准确率。
4. 增加 Nav2 地图/waypoint assets 和 RViz 展示。
5. 把更多诊断和执行编排 C++ 化，补 launch test。
