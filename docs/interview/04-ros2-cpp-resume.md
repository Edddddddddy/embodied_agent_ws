# ROS2 / C++ 机器人软件开发求职简历

> 使用说明：本文件中的“自我评价、个人技能、项目经历”可以直接替换原 C++ 后端简历对应部分。姓名、联系方式、教育经历、工作时间等未知信息保留为方括号占位符，投递前替换即可。

## 一、求职定位

**求职方向：ROS2 开发工程师 / C++ 机器人软件开发工程师 / 机器人中间件开发工程师**

关键词：`ROS 2`、`C++17`、`Linux`、`rclcpp`、`rclpy`、`DDS/QoS`、`Action`、`Lifecycle`、`Nav2`、`Gazebo`、`BehaviorTree.CPP`、`pluginlib`、`多线程`、`实时数据流`、`机器人安全控制`

## 二、可直接投递版

### 基本信息

**[姓名]**  |  ROS2 / C++ 机器人软件开发工程师  
[手机号] | [邮箱] | [所在城市] | [GitHub/Gitee 地址]

### 自我评价

具备扎实的现代 C++、Linux 系统编程与 ROS2 机器人软件开发基础，熟悉 Topic、Service、Action、Lifecycle、DDS/QoS、launch、component、pluginlib 等核心机制，能够独立完成节点划分、接口设计、异步状态管理、机器人动作执行与测试验收。具有端侧语音机器人控制系统开发经验，完成从音频采集、VAD/ASR、NLU/LLM 到 C++ 动作安全校验、BehaviorTree、Gazebo/Nav2 执行的完整链路，重点解决连续指令、急停抢占、超时取消、迟到回调和安全停机等工程问题。曾参与中科院脑磁数据分析系统研发，积累了现代 C++、实时数据流处理、Qt3D/OpenGL 可视化与工程交付经验，同时具备 C++ 后端和网络编程基础，能够快速理解复杂系统并推进设计、开发、联调与问题定位。

### 个人技能

1. **现代 C++：**熟悉 C++11/14/17 常用特性，掌握 RAII、智能指针、移动语义、lambda、模板及 STL 容器与算法；具备多线程、异步回调、资源生命周期管理和模块化接口设计经验。
2. **ROS2 与中间件：**熟悉 `rclcpp/rclpy`，掌握 Topic、Service、Action、Parameter、Lifecycle Node、launch、component composition 等开发机制；理解 DDS discovery 及 Reliable/Best Effort、Volatile/Transient Local 等 QoS 策略的适用场景，能够设计强类型 `msg/srv/action` 接口。
3. **机器人控制与导航：**熟悉 `/cmd_vel`、`LaserScan`、里程计和速度控制链路，具备速度/加速度限幅、近障停车、传感器超时保护、避障和沿墙 PID 等实现经验；了解 Nav2 的 `NavigateToPose`、`FollowWaypoints`、goal feedback/result/cancel 机制及语义目标点转换流程。
4. **机器人软件架构：**熟悉 BehaviorTree.CPP 行为编排与 pluginlib 插件化机制，能够将 Action Server、行为树、安全检查和 Mock/Gazebo/Nav2 执行后端解耦；掌握 Lifecycle 安全启停、系统 readiness、diagnostics、watchdog 和多层故障收敛设计。
5. **语音与端侧智能：**具备 PortAudio 音频采集、NLMS 回声消除、Energy/WebRTC/Silero VAD、KWS、语音端点检测和连续会话状态机开发经验；了解在线 ASR/LLM/TTS 与 Sherpa-ONNX、llama.cpp 等离线推理链路，能够实现确定性 NLU、流式协议解析和伪流式 TTS 缓冲。
6. **Linux 工程能力：**熟悉 Linux 进程/线程、IPC、虚拟内存、文件 I/O 和同步控制；熟练使用 Git、CMake、colcon、GDB，掌握 Valgrind、perf 等内存与性能分析工具，能够在 WSL/Ubuntu 和 ROS2 环境完成构建、部署与排障。
7. **测试与质量保障：**熟悉 gtest、pytest 和 ROS2 集成测试，具备单元测试、Action/Topic 链路测试、mock/fake server、仿真 smoke test 和自动化验收脚本开发经验；能够基于 command ID、generation、timeout 和状态不变量验证异步系统正确性。
8. **通用软件开发：**熟悉 TCP/IP、Socket、Reactor、epoll、线程池和连接池；熟悉 MySQL、Redis、gRPC、Kafka 等常用中间件。具备 Qt 信号槽、Qt3D/OpenGL 实时可视化开发经验；LeetCode 200+，熟悉 DFS、BFS、贪心、动态规划等常见算法思想。

### 项目经历

#### 具身智能语音机器人控制系统（ROS2）

**项目角色：**核心开发 / 个人项目  
**项目时间：**[20XX.XX - 至今]  
**技术栈：**ROS2 Jazzy、C++17、Python、rclcpp/rclpy、DDS/QoS、Lifecycle、ROS2 Action、BehaviorTree.CPP、pluginlib、Nav2、Gazebo、PortAudio、Sherpa-ONNX、llama.cpp、gtest、pytest

**项目描述：**  
面向端侧机器人的在线/离线语音交互与运动控制系统，打通“麦克风/文本输入 -> VAD/ASR -> NLU/LLM -> 强类型动作候选 -> C++ 安全校验 -> ROS2 Action/BehaviorTree -> Gazebo 或 Nav2 执行”的完整链路。系统支持一次唤醒后的连续多指令、FIFO 排队、急停抢占、语义目标点导航、执行反馈和安全停机，并通过自动化测试与仿真链路形成可复现的验收闭环。

**主要工作与成果：**

1. 设计由 9 个 ROS2 package 组成的分层架构，将接口契约、中间件语义、共享 Agent 控制面、语音前端、部署编排、在线/离线 Provider、C++ 安全层和仿真执行层解耦；抽取在线/离线共用的会话、队列、Lifecycle、用户上下文和 ROS I/O，降低重复实现与后端切换成本。
2. 设计自定义 `RobotCommand`、`ExecuteRobotCommand.action` 及状态/诊断消息，按 command、event、state、sensor、audio、diagnostics 六类语义统一 Python/C++ QoS；针对 DDS 启动发现窗口实现带容量和 TTL 的 guarded outbox，避免订阅端尚未匹配时丢失首条控制命令，同时阻止陈旧命令重放。
3. 使用 C++ 实现 ActionGuard 与 ActionScheduler：完成动作白名单、字段互斥、NaN/Inf 检查、速度/时长限幅和地点校验；采用单 active goal + pending FIFO 保证串行执行，通过 command ID、取消 watchdog 和 stale result 过滤解决急停、超时及异步迟到回调导致的串单问题。
4. 实现连续语音控制状态机，支持唤醒/休眠、重复 ASR final 与语气词过滤、短命令补全、多命令拆分、有界队列和 stale TTL；急停与取消导航绕过普通 FIFO，联动清理自然语言队列、取消 Python 批次并抢占底层 ROS2 Action。
5. 搭建 C++ 音频前端，基于 PortAudio 回调和有界缓冲处理 PCM 数据，实现 NLMS 自适应回声消除、VAD 滞回和 speech endpoint 事件；通过 sidecar 适配 WebRTC/Silero VAD、openWakeWord/KWS 与 Sherpa-ONNX 声纹输入，使模型依赖与 Agent 控制面保持隔离。
6. 基于 BehaviorTree.CPP 编排 `validate -> safety -> execute -> confirm` 动作流程，通过 pluginlib 提供 Mock/Gazebo/Nav2 执行器；实现 20 Hz 运动控制、速度斜坡、LaserScan 前/左/右扇区检测、scan stale fail-safe、近障停车、避障及沿墙 PID，并在动作结束、取消或异常时回零 `/cmd_vel`。
7. 实现语义导航适配，将“门口/书桌/起点”等地点映射为 YAML 中的位姿并构造 Nav2 `NavigateToPose/FollowWaypoints` goal；以 Nav2 原生 result 驱动外层 Action 终态，并使用 goal generation 处理取消发生在 goal handle 返回前的竞态，防止迟到回调产生后台“幽灵导航”。
8. 建立分层自动化验收体系，覆盖 repository contract、Python 单测、C++ gtest、ROS2 Action、Lifecycle、pluginlib、语音会话、导航 bridge、硬件 mock watchdog 和雷达安全链路；当前 mock 主门禁完成 9 个 package 构建，56 项仓库测试、226 项 Python 测试及 419 项 ROS/C++ 测试均零失败。

### 其他项目或工作经历衔接

原简历中的中科院脑磁数据分析系统、高并发通信系统、分布式文件存储和云原生任务调度平台可以继续保留，但建议按以下顺序排列：

1. 具身智能语音机器人控制系统。
2. 中科院脑磁数据分析系统，突出 C++、实时数据流、Qt3D/OpenGL 与工程交付。
3. 选择一个最有技术深度的后端项目，突出并发、网络和性能优化。
4. 其余项目压缩为 2 至 3 条，避免削弱 ROS2 求职定位。

## 三、招聘平台精简版

### 精简自我介绍

具备现代 C++、Linux 与 ROS2 机器人软件开发经验，熟悉 Topic/Service/Action、Lifecycle、DDS/QoS、BehaviorTree.CPP、pluginlib、Gazebo 和 Nav2。完成端侧语音机器人从音频采集、ASR/NLU/LLM 到 C++ 动作安全校验、Action 调度和仿真执行的完整链路，具备多线程异步状态管理、急停抢占、传感器安全控制及自动化测试经验，同时拥有 C++ 后端和 Qt 实时可视化开发基础。

### 项目精简描述

负责 ROS2 端侧语音机器人控制系统核心链路开发，设计在线/离线共享 Agent、强类型动作接口、C++ ActionGuard 和单执行槽调度器，实现连续语音、多命令 FIFO、急停抢占、BehaviorTree 执行、Gazebo 控制及 Nav2 语义目标点适配。通过 Lifecycle、QoS、TTL outbox、watchdog、LaserScan fail-safe 和分层测试保证系统可取消、可超时、可观测和安全停车。

## 四、一页简历压缩版项目 Bullet

当版面只能保留 4 条项目内容时，使用以下版本：

- 设计 9 package ROS2 分层架构，统一在线/离线 Agent 的会话、队列、Lifecycle 与 ROS I/O，基于 typed msg/action 和六类 QoS 语义规范跨节点通信。
- 使用 C++ 实现 ActionGuard 与单 active goal 调度器，完成动作白名单/限幅、FIFO、急停抢占、取消 watchdog、TTL outbox 和迟到结果隔离，避免 LLM 直接控制 `/cmd_vel`。
- 基于 BehaviorTree.CPP + pluginlib 构建 Mock/Gazebo/Nav2 可替换执行端，实现 20 Hz 速度控制、LaserScan stale/近障保护、速度斜坡和安全停车；完成语义地点到 Nav2 Action goal 的转换与结果闭环。
- 搭建 PortAudio + NLMS AEC + VAD/ASR 语音前端和连续会话链路，并建立自动化验收体系；mock 主门禁完成 9 包构建，56 项仓库、226 项 Python、419 项 ROS/C++ 测试零失败。

## 五、事实边界与投递检查

以下内容用于投递前自检，不建议原样放进简历。

| 能力 | 简历可用表述 | 不应写成 |
| --- | --- | --- |
| 音频增强 | 实现 NLMS 自适应回声消除 | 完成 WebRTC 通用降噪和 AGC |
| VAD | 适配 Energy/WebRTC/Silero VAD | 使用 WebRTC 完成降噪 |
| Nav2 | 完成语义地点到 Nav2 Action goal 的适配和 bridge 验证 | 完成真实机器人自主导航量产落地 |
| Gazebo | 完成仿真执行器及动作链路验收 | 完成真实底盘闭环控制 |
| 硬件通信 | 完成 UART/SPI transport 与 mock watchdog | 完成下位机 ACK 和实机运动闭环 |
| 离线模型 | 接入 Sherpa-ONNX、llama.cpp 和 TTS 运行链路 | 完成 LoRA 训练并达到生产准确率 |
| 性能数据 | 当前机器阶段性 benchmark | 生产 SLA 或大规模统计结论 |
| 测试数据 | 自动化测试与 mock/fake/仿真证据 | 真实场景长期稳定性和量产可靠性 |

## 六、投递前最终替换项

- 将 `[姓名]`、联系方式、项目时间、教育经历和工作经历补齐。
- GitHub 仓库保持可访问，并确保 README 第一屏能看到系统链路、构建命令和演示证据。
- 根据 JD 调整技能顺序：导航岗提前 Nav2，机器人平台岗提前 ROS2/QoS/Lifecycle，端侧智能岗提前语音与离线推理。
- 简历正文控制在 1 至 2 页；若保留多个后端项目，机器人项目至少占项目经历篇幅的 40%。
- 简历中的每个技术名词都应能回答“为什么使用、核心实现、异常场景、测试证据”四个问题。
