# 版本记录与路线图

本文档记录项目阶段性演进、当前完成度和后续路线。详细架构见 [ARCHITECTURE_AND_KNOWLEDGE.md](ARCHITECTURE_AND_KNOWLEDGE.md)，关键技术学习笔记见 [LEARNING_NOTES.md](LEARNING_NOTES.md)。

## 1. 阶段性版本记录

| 阶段 | 主要目标 | 结果 |
| --- | --- | --- |
| 初始骨架 | 创建 ROS 2 workspace，搭建在线 Agent、动作 topic、stub 节点 | 完成无密钥 mock 链路 |
| 在线 Agent | 接入在线 ASR/LLM/TTS，设计 prompt、记忆、动作格式 | 完成在线接口 smoke 与动作解析 |
| C++ 化与安全网关 | 将适合 C++ 的 ROS 节点迁移/新增为 C++，加入 ActionGuard | 完成 JSON 校验、限幅、强类型转换 |
| 离线 Agent | 接入 Sherpa/llama.cpp/Sherpa-TTS 适配与 mock 链路 | 完成离线结构、双缓冲、延迟统计、smoke 入口 |
| 仿真控制 | 接入 Gazebo/TurtleBot3，打通 `/cmd_vel` 与 odom 验收 | 完成语音到 Gazebo 运动闭环 |
| Nav2 风格规范化 | 引入自定义 msg/action、Lifecycle、BehaviorTree、pluginlib | 完成 typed Action、BT 编排、mock/Gazebo executor |
| 丰富演示动作 | 增加前进、后退、转向、绕圈、正方形、演示序列 | 完成动作秀和安全演示 |
| 连续语音控制 | 一次唤醒后连续说多条命令，支持队列和急停 | 完成 continuous offline/online 验收入口 |
| 真实语音稳定性 | 修复尾部漏识别、重复识别、filler、queue_full 可观测性 | 完成 VAD profile、commit delay、短命令补全、monitor |
| 阶段性文档收尾 | 整理 README、验收文档、学习笔记、关键中文注释 | 当前阶段 |
| 轻量 NLU 多命令 | 识别一句 ASR final 内的多个动作，并保证队列顺序 | 新增 CommandNLU、batch 可观测性、request_id/result 关联 |
| 语音导航与巡航 | 支持语音目标点导航、多目标点巡航，并接入 typed Action 与 Nav2 bridge | 新增 navigate_to/follow_waypoints/cancel_navigation 协议、NLU、ActionGuard 校验、navigation-demo 和 nav2-bridge |
| Nav2 bringup 入口 | 复用官方 Nav2 TurtleBot3 仿真 launch，接入本项目语音控制链路 | 新增 voice_nav2_turtlebot3.launch.py、nav2-preflight 和 nav2-turtlebot3 重型验收 |
| Nav2 result 闭环 | 用 Nav2 action result 驱动本项目 ExecuteRobotCommand result | 新增 RobotExecutor external_action_update seam，避免导航 goal 按本地 duration 假完成 |
| 真实 Nav2 验收修复 | 跑通 TurtleBot3/Nav2 目标点导航与多目标点巡航 | 修复官方 launch 布尔参数、AMCL initialpose 和导航长动作超时；`nav2-turtlebot3` PASS |
| 真实麦克风 Nav2 连续导航 | 支持一次唤醒后连续说多个目标点/巡航命令并进入 Nav2 队列执行 | 新增 `continuous-nav2-offline/online`、AMCL initialpose 辅助脚本和 live-check 入口 |
| 连续导航队列回归 | 自动验证连续会话中目标点导航与多目标点巡航不会丢队列 | 新增 `continuous-navigation`，覆盖多目标点 NLU、队列元数据和 request_id/result 关联 |
| Nav2 现场验收增强 | 让真实麦克风辅助计分更贴近导航目标 | `continuous-nav2-live-check` 额外要求出现 `navigate_to` 与 `follow_waypoints` |
| 真实麦克风验收留证 | 让现场验收结果可保存、可复查 | `CONTINUOUS_LIVE_CHECK_REPORT=...` 可导出 live-check JSON 报告 |
| 真实麦克风报告复核 | 让现场报告可以脱离仿真环境二次判定 | 新增 `continuous-live-report` / `continuous-nav2-live-report` |
| 一键式 Nav2 现场留证 | 降低真实麦克风 Nav2 验收操作复杂度 | 新增 `continuous-nav2-evidence`，单终端启动控制、计分、保存报告并清理进程 |
| 一键留证 dry-run | 让现场验收脚本可自动测试、可提前检查参数 | `CONTINUOUS_NAV2_EVIDENCE_DRY_RUN=true` 打印控制/计分命令但不启动仿真 |
| 自然多目标导航话术 | 提升真实语音目标点/巡航表达容错 | 支持“先去门口再去书桌最后回起点”“巡逻门口书桌起点”，同时保留两目标语句拆成多个 `navigate_to` 入队 |

## 2. 当前完成度结论

当前项目已经达到“语音输入 → 大模型/规则动作解析 → ROS 2 安全校验 → Gazebo 仿真控制”的主链路目标。

已具备的展示点：

- ROS 2 C++ 节点：音频前端、ActionGuard、typed action bridge、仿真执行层。
- Python Agent：在线/离线 provider、连续语音会话、命令队列、LLM/TTS 编排。
- 工程化接口：自定义 msg/action、Lifecycle、BehaviorTree.CPP、pluginlib。
- 演示能力：真实麦克风连续语音、多动作序列、急停抢占、Gazebo 运动验证。
- 多命令能力：一条 ASR final 可被轻量 NLU 解析为多个队列项，并按 ROS 2 Action result 顺序执行。
- 导航演示能力：支持“去门口”“前往书桌”“依次去门口、书桌、起点”等语音目标点/多点巡航命令，并通过 typed Action 驱动仿真 executor、Nav2 action bridge 或完整 TurtleBot3/Nav2 bringup。
- 测试体系：单元测试、集成 smoke、Gazebo 验收、真实麦克风辅助统计。

需要谨慎表述的边界：

- 当前硬件控制是预留/mock，不是实体机器人完整验收。
- 当前已提供完整 TurtleBot3/Nav2 重型验收入口，但地图构建、复杂目标点规划和更复杂场景仍是后续增强。
- 离线 LoRA 训练、量化指标可以作为规划和接口说明，不应夸大为已复现完整训练结果。
- openWakeWord、LiveKit WakeWord、Silero VAD 是可选 seam/preflight，不是默认强依赖链路。

## 3. 当前最有价值的验收证据

基础自动验收：

```bash
bash scripts/acceptance_test.sh mock
```

连续语音自动验收：

```bash
bash scripts/acceptance_test.sh continuous-endpoint
bash scripts/acceptance_test.sh continuous-mock
bash scripts/acceptance_test.sh continuous-multi-command
bash scripts/acceptance_test.sh continuous-queue-full
bash scripts/acceptance_test.sh voice-readiness
```

语音导航/巡航验收：

```bash
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh navigation-demo
bash scripts/acceptance_test.sh continuous-navigation
bash scripts/acceptance_test.sh nav2-bridge
bash scripts/acceptance_test.sh nav2-preflight
bash scripts/acceptance_test.sh nav2-turtlebot3
```

真实麦克风 Nav2 连续导航验收：

```bash
bash scripts/acceptance_test.sh continuous-nav2-offline
CONTINUOUS_LIVE_CHECK_DURATION=240 bash scripts/acceptance_test.sh continuous-nav2-live-check offline
```

Gazebo 验收：

```bash
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh gazebo-voice
bash scripts/acceptance_test.sh gazebo-voice-online
```

真实麦克风验收：

```bash
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh continuous-live-check offline
```

## 4. 近期路线图

### P0：保持演示稳定

- 优先保证 `continuous-offline` 在 3～5 分钟内稳定连续控制。
- 优先保证 `continuous-nav2-offline` 能支撑 3～5 分钟真实麦克风目标点导航/巡航演示。
- 继续完善 monitor 输出，让失败原因能直接定位到 ASR、session、queue、Action、Gazebo。
- 为常见麦克风和噪声环境补充 profile 建议。

### P1：增强 ROS 2/C++ 求职展示价值

- 继续提高 C++ 节点比例：安全校验、仿真执行、协议层、诊断层优先 C++。
- 补充更多 C++ 单元测试和 launch test。
- 将关键设计整理成可讲的架构图和面试问答。

### P2：补齐端侧部署故事

- 固化离线模型下载、量化、启动 llama.cpp server 的流程。
- 增加离线 benchmark 报告模板。

### P3：增强真实 Nav2 导航栈

- 在已有 `voice_nav2_turtlebot3.launch.py` 基础上增加保存地图、AMCL/SLAM、目标点巡航场景资产。
- 增加更稳定的真实 Nav2 odom/goal-result 统计报告，区分 planner/controller/behavior tree 失败原因。
- 明确 LoRA 训练数据集格式和复现实验入口。

### P3：可选增强

- 接入真实声学 KWS，例如 sherpa-onnx keyword spotting 或 openWakeWord。
- 接入 WebRTC AEC/NS，改善扬声器回声环境。
- 引入更接近 Nav2 的行为树 XML 和复杂安全策略。
- 接真实硬件底盘或串口设备，完成 UART/SPI 实体验收。

## 5. 不建议近期优先做的事情

- 过早引入完整导航栈、地图和路径规划：会稀释当前“语音到动作控制闭环”的主线。
- 大规模重写仓库结构：当前更需要稳定验收和文档清晰。
- 依赖复杂声学模型作为默认链路：会提高部署门槛，影响演示可复现性。

## 6. 项目介绍版本

具身智能机器人智能语音交互与仿真控制系统

项目描述：设计并实现机器人智能语音交互系统的在线与离线 Agent 链路，打通从语音输入、ASR 识别、大模型/规则动作解析、动作安全校验到 ROS 2/Gazebo 仿真控制的端到端流程。项目引入自定义 ROS 2 msg/action、C++ ActionGuard、BehaviorTree.CPP 和 pluginlib 执行器，支持真实麦克风连续语音控制、多命令队列、急停抢占和仿真运动验收。

主要工作：

- 在线流式 Agent：接入在线 ASR/LLM/TTS provider，设计系统 prompt、记忆管理、动作输出格式、fallback parser 与动作回调，实现 ASR final 到动作候选和 TTS 反馈的流式交互。
- 离线 Agent：预留 Sherpa-onnx ZipFormer ASR、llama.cpp、Sherpa-TTS 的端侧部署结构，设计双缓冲和延迟统计，支持 mock 与真实模型 smoke 验收。
- 连续语音控制：实现 wake/session gate、重复 ASR final 过滤、filler 过滤、命令队列、TTL、急停抢占、短命令补全和现场 monitor，提高真实麦克风长时间控制稳定性。
- ROS 2/C++ 控制链路：基于 C++ 编写 ActionGuard、typed action bridge、仿真执行节点，将 LLM 动作 JSON 转换为强类型 RobotCommand 和 ROS 2 Action，并在 Gazebo/TurtleBot3 中验证 `/cmd_vel` 和 odom 变化。
- 工程化与验收：引入 BehaviorTree.CPP 编排动作校验、安全检查、执行和结果确认，用 pluginlib 支持 mock/Gazebo executor，配套单元测试、集成 smoke、Gazebo 验收和真实麦克风人工验收脚本。
