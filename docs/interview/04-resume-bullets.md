# 简历表述

## 项目标题

推荐标题：

```text
Embodied Voice Agent for ROS 2：端侧语音机器人控制系统
```

备选标题：

```text
基于 ROS 2 Jazzy 的在线/离线语音机器人安全控制原型
```

更简洁版本：

```text
ROS 2 语音机器人控制与 Gazebo 仿真闭环
```

## 技术栈

```text
ROS 2 Jazzy, C++17, Python 3.12, Gazebo, TurtleBot3, BehaviorTree.CPP, pluginlib,
rclcpp_lifecycle, ROS 2 Action, PortAudio, sherpa-onnx, llama.cpp, Qwen ASR/LLM/TTS,
UART/SPI, pytest, colcon, GitHub Actions
```

## 简历项目描述

偏工程版本：

```text
设计并实现基于 ROS 2 的端侧语音机器人控制原型，打通麦克风输入、在线/离线 ASR、LLM 结构化动作解析、C++ 安全仲裁、ROS 2 Action/BehaviorTree 执行、Gazebo 仿真与硬件传输接口，形成从语音指令到机器人动作执行的可测试闭环。
```

偏机器人版本：

```text
基于 ROS 2 Jazzy 与 Gazebo 构建 TurtleBot3 语音控制链路，将自然语言命令转换为受限机器人动作，通过 C++ ActionGuard、LifecycleNode、BehaviorTree.CPP 和 pluginlib executor 实现动作校验、可取消执行、安全停车与仿真位移验收。
```

偏 AI + 系统版本：

```text
构建在线/离线双链路语音 Agent，将 Qwen 云端模型与 ZipFormer + llama.cpp + Sherpa-TTS 离线模型统一接入 ROS 2 安全执行层，解决 LLM 不可信输出到机器人可信动作之间的 schema 校验、限幅、拒绝、反馈和结果确认问题。
```

## 职责 bullet

推荐使用 4-6 条，不要全部堆上去。

- 设计语音到动作的数据流：麦克风 PCM 经 C++ AEC/VAD 断句后进入在线 Qwen 或离线 ZipFormer/llama.cpp/Sherpa-TTS 链路，LLM 输出统一解析为候选 `<speech>/<action>`。
- 实现 C++ ActionGuard，将不可信 LLM 输出转换为可信机器人动作，覆盖动作白名单、schema 校验、速度/角速度/时长限幅、拒绝原因、急停和 watchdog。
- 将执行链从 JSON topic 迁移到强类型 ROS 2 Action，支持 goal、feedback、result、cancel、timeout 和 preempt，并保留旧 JSON gateway 兼容路径。
- 使用 `rclcpp_lifecycle` 和 Nav2 lifecycle manager 管理 Guard 与仿真执行器启动顺序，保证未激活状态不执行动作，deactivate/cleanup 时终止目标并发布零速。
- 引入 BehaviorTree.CPP 编排 `Validate -> Safety -> Execute -> Confirm`，在动作运行期间持续检查安全条件，并将执行阶段发布到 `/robot/bt_status` 便于验收和排障。
- 抽象 `RobotExecutor` pluginlib interface，使同一套 Action/BT/Guard 链路可切换 Gazebo 与 mock executor，后续可扩展 UART/SPI 或其他机器人后端。
- 建立分层验收体系，覆盖 C++/Python 单元测试、ROS mock、Lifecycle、typed Action、BT、pluginlib、namespace、component container、在线/离线模型和 Gazebo 物理位移。
- 通过 Action result、BT confirm、executor ACK、非零 `/cmd_vel` 和 Gazebo `/odom` 位移联合证明动作真实执行，避免只以 topic 发布成功作为闭环依据。

## 可写的量化结果

这些数字来自当前文档记录，写简历时建议加“当前环境”或“少量样本”限定。

- 当前 WSL Ubuntu 24.04 环境下，mock release gate 记录为 130 项 colcon 测试与 2 项仓库约束测试零失败。
- 当前样本中，在线热启动 LLM 首 token 为 350-384 ms，在线 TTS 首音频为 222-242 ms。
- 当前样本中，离线 Q8 llama.cpp CPU decode 为 34.10 token/s，离线语音全链为 2.313 s。
- Gazebo 验收中，typed Action/BT 链路产生 0.330 m `/odom` 位移；在线语音 typed 闭环产生 0.163 m 位移；离线语音到 typed Action 到 Gazebo 产生 0.162 m 位移。
- 原始 0.6B 模型在 8 条种子命令上动作为 2/8，有限命令 fallback 后系统链路为 7/8；需明确这是工程兜底，不是模型准确率。

## 一版完整简历写法

```text
Embodied Voice Agent for ROS 2：端侧语音机器人控制系统
技术栈：ROS 2 Jazzy、C++17、Python、Gazebo、BehaviorTree.CPP、pluginlib、rclcpp_lifecycle、llama.cpp、sherpa-onnx

- 设计并实现语音到机器人动作的 ROS 2 工程闭环，支持在线 Qwen ASR/LLM/TTS 与离线 ZipFormer + Qwen3-0.6B Q8/llama.cpp + Sherpa-TTS，两条链路统一进入 C++ 安全执行层。
- 实现 C++ ActionGuard，对 LLM 候选动作进行白名单、schema、速度/时长限幅、拒绝和 watchdog 处理，避免模型直接发布 `/cmd_vel`。
- 将动作执行抽象为强类型 ROS 2 Action 与 BehaviorTree.CPP 流程，支持反馈、取消、超时、抢占和 `Validate -> Safety -> Execute -> Confirm` 阶段观测。
- 使用 LifecycleNode、pluginlib executor 和 diagnostics 支持 Gazebo/mock 后端切换、组件化部署和 namespace 隔离，为后续接入 UART/SPI 或真实机器人保留扩展点。
- 建立分层 release gate，通过 Action result、BT confirm、ACK、非零 `/cmd_vel` 和 Gazebo `/odom` 位移联合验收；当前记录中 mock、online、offline、Gazebo 和语音到 Gazebo 自动 gate 均通过。
```

## 更短版本

```text
基于 ROS 2 Jazzy 构建端侧语音机器人控制原型，打通在线/离线 ASR、LLM 动作解析、C++ ActionGuard、ROS 2 Action、BehaviorTree.CPP 与 Gazebo TurtleBot3 仿真闭环；实现动作 schema 校验、限幅、急停、取消、超时、pluginlib 后端切换和分层 release gate，通过 Action result、BT status、ACK、`/cmd_vel` 与 `/odom` 验证动作真实执行。
```

## 面试口径中的事实边界

可以说：

- “当前定位是可复现工程原型。”
- “主链路聚焦 `move / turn / stop`。”
- “在线/离线/Gazebo/语音到 Gazebo release gates 已有记录。”
- “UART/SPI interface 和 mock 测试已做，实体硬件仍需目标机器人验收。”
- “LoRA 配置和种子数据已准备，但尚未训练。”

不要说：

- “实现完整工业级语音机器人系统。”
- “实现完整具身智能 Agent。”
- “离线模型准确率达到 7/8。”
- “LoRA 后准确率达到 85%。”
- “延迟稳定低于某个生产 SLA。”
- “真实机器人硬件全链路已完成验收。”

## 针对不同岗位的强调方式

机器人 / ROS 岗：

```text
重点讲 ROS 2 Action、Lifecycle、BT、pluginlib、Gazebo、`/cmd_vel`、`/odom`、diagnostics、namespace。
```

C++ 工程岗：

```text
重点讲 ActionGuard、状态机、线程安全快照、Lifecycle 资源管理、CRC/watchdog、测试分层。
```

AI 应用岗：

```text
重点讲在线/离线模型接入、流式协议、fallback 边界、模型能力与工程兜底分离。
```

系统工程岗：

```text
重点讲端到端链路、可观测性、release gate、故障定位顺序和事实边界。
```
