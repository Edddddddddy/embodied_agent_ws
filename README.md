# Embodied Voice Agent for ROS 2

[![ROS 2 CI](https://github.com/Edddddddddy/embodied_agent_ws/actions/workflows/ros2-ci.yml/badge.svg)](https://github.com/Edddddddddy/embodied_agent_ws/actions/workflows/ros2-ci.yml)

面向 TurtleBot3 与端侧机器人的在线/离线语音控制系统：从麦克风、流式 ASR、LLM
动作解析，一直到 C++ 安全仲裁、Gazebo 仿真或 UART/SPI 硬件输出。

> 当前定位是“可复现的工程原型”：优先保证 `move / turn / stop` 的完整链路，
> 不追求复杂导航行为。目标环境为 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12、C++17。

## 能做什么

- 在线链路：Qwen 实时 ASR、流式 LLM、实时 TTS，支持预热、记忆和延迟指标。
- 离线链路：sherpa-onnx ZipFormer、Qwen3-0.6B Q8/llama.cpp、Sherpa-TTS。
- 声学前端：C++ PortAudio、NLMS AEC、VAD、0.4 秒静音断句。
- 识别恢复：热词偏置、唤醒别名、失败反馈和持续重试。
- 动作安全：结构化动作、C++ schema 校验、限幅、急停和 watchdog。
- 生命周期：Guard 与仿真执行器采用 C++ LifecycleNode，由 Nav2 manager 有序激活。
- 行为编排：BehaviorTree.CPP XML 执行验证、安全检查、异步动作与结果确认。
- 执行插件：pluginlib 按参数切换 Gazebo 与无仿真的 mock executor。
- 组件化部署：同一 C++ 控制实现支持独立进程和 ROS 2 component container。
- 可观测性：标准 diagnostics 报告生命周期、执行后端、动作与安全停车状态。
- 多机器人隔离：执行链使用相对 ROS 名称，可整体放入 `namespace`。
- 控制后端：TurtleBot3 Gazebo、UART、SPI 和无硬件 mock。
- 自动验收：单元测试、ROS 冒烟、云模型、离线模型和 Gazebo 物理位移验证。

## 数据流

```mermaid
flowchart LR
  Mic["麦克风 PCM16"] --> Audio["C++ AEC / VAD / 0.4s 断句"]
  Audio --> ASR["在线 Qwen ASR<br/>或离线 ZipFormer"]
  ASR --> Wake["唤醒、热词与重试"]
  Wake --> LLM["流式 LLM"]
  LLM --> Parser["speech/action 增量解析"]
  Parser --> TTS["在线或离线 TTS"]
  Parser --> Guard["C++ ActionGuard"]
  Guard --> BT["BehaviorTree.CPP<br/>Validate / Safety / Execute / Confirm"]
  BT --> Sim["Gazebo /cmd_vel"]
  Guard --> HW["UART / SPI"]
  TTS --> Speaker["C++ 播放队列"]
```

Python 负责模型 SDK、文本协议和对话编排；C++ 负责实时音频、动作安全、仿真控制
和硬件传输。两侧只通过 ROS 2 话题连接。

## 五分钟运行

```bash
cd /home/ubuntu/embodied_agent_ws
bash scripts/bootstrap.sh
source scripts/activate.sh

# 无密钥、无麦克风验证主链路
bash scripts/smoke_test.sh

# mock Agent 驱动 Gazebo（无需麦克风和模型）
ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  provider_mode:=mock microphone_enabled:=false speaker_enabled:=false
```

真实麦克风控制仿真：

```bash
# 离线；首次需执行 scripts/setup_offline_runtime.sh
bash scripts/accept_voice_simulation_microphone.sh offline

# 在线；先按 .env.example 配置 DashScope
bash scripts/accept_voice_simulation_microphone.sh online
```

识别失败时终端会显示重试次数并继续监听，不需要重新启动节点。启用唤醒词验收：

```bash
WAKE_WORD_ENABLED=true \
  bash scripts/accept_voice_simulation_microphone.sh offline
```

## 验收入口

```bash
bash scripts/acceptance_test.sh mock     # 单元/结构测试及无模型全链
bash scripts/acceptance_test.sh online   # 少量云 API 调用
bash scripts/acceptance_test.sh offline  # 本地模型、语音和性能
bash scripts/acceptance_test.sh gazebo   # Gazebo 可信动作与里程计
bash scripts/acceptance_test.sh gazebo-voice  # 离线语音模型直达 Gazebo
bash scripts/acceptance_test.sh all      # 全部自动 release gates（不含真人麦克风）
```

查看所有自动与交互式模式：`bash scripts/acceptance_test.sh --help`。真人麦克风验收也可
统一使用 `microphone-offline` 或 `microphone-online` 模式。

性能数字是验收目标而不是硬编码承诺。当前实测、限制和复现方法见
[测试与验收](docs/TESTING_AND_ACCEPTANCE.md)。

仿真 launch 默认使用新的强类型 ROS 2 Action 链：

```bash
ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  provider_mode:=mock use_typed_actions:=true
```

排查兼容问题时可临时传入 `use_typed_actions:=false` 回到旧 JSON topic 执行路径。
launch 默认 `lifecycle_autostart:=true`；调试启动顺序时可设为 `false`，再使用
`ros2 lifecycle set /<node> configure|activate` 手工转换状态。

不启动 Gazebo 验证同一 Action/BT 链的 executor 插件切换：

```bash
bash scripts/smoke_test_mock_executor.sh
bash scripts/smoke_test_composed_executor.sh
bash scripts/smoke_test_namespaced_executor.sh
```

launch 参数 `executor_plugin` 默认为
`embodied_simulation/GazeboRobotExecutor`，也可选择
`embodied_simulation/MockRobotExecutor`。
`simulation_control.launch.py` 可通过 `use_composition:=true` 改为组件容器，通过
`namespace:=robot1` 隔离整条执行链；两者可同时使用。

## 当前自动验收结论

2026-07-02 在当前 WSL 环境完成了 mock、在线、离线、Gazebo，以及在线/离线语音→Gazebo 验收：
130 项 colcon 测试与 2 项仓库约束测试零失败；在线热启动 LLM 首 token 350–384 ms、
TTS 首音频 222–242 ms；离线
Q8 CPU decode 34.10 token/s、语音全链 2.313 s；typed Action/BT 驱动 Gazebo 位移
0.330 m，在线语音 typed 闭环位移 0.163 m。原始 0.6B 模型动作准确率仅 2/8，fallback 后为 7/8，因此 LoRA 仍明确标记为
未完成，不能用 fallback 成绩冒充模型成绩。完整证据见[测试与验收](docs/TESTING_AND_ACCEPTANCE.md)。

## 项目结构

```text
src/
  embodied_agent_interfaces/ ROS 2 强类型 RobotCommand 与 ExecuteRobotCommand Action
  embodied_agent_cpp/       C++ 音频、ActionGuard、UART/SPI
  embodied_online_agent/    在线 ASR/LLM/TTS 与公共对话模块
  embodied_offline_agent/   ZipFormer、llama.cpp、Sherpa-TTS、双缓冲
  embodied_simulation/      TurtleBot3 控制、雷达安全和 Gazebo launch
scripts/                    安装、启动、基准和分层验收入口
tests/integration/          ROS graph 黑盒探针，由 smoke runner 启动
tests/repository/           仓库结构与交付约束
training/                   LoRA 种子数据与 LLaMA-Factory 配置（尚未训练）
docs/                       三份维护文档
```

## 文档

- [架构与知识笔记](docs/ARCHITECTURE_AND_KNOWLEDGE.md)：模块、关键代码、话题和设计原理。
- [测试与验收](docs/TESTING_AND_ACCEPTANCE.md)：命令、测试矩阵、指标和故障定位。
- [版本记录与路线图](docs/CHANGELOG_AND_ROADMAP.md)：迭代历史、完成度、竞品对比和下一步。
- [测试目录说明](tests/README.md)：单元、集成和仓库约束如何分层。
- [贡献指南](CONTRIBUTING.md)：修改原则、提交前检查和 executor 扩展要求。

## 当前边界

- LoRA 配置和数据已准备，但尚未训练，不能宣称 85% 模型指令遵循率。
- Q8 相对 FP16 通常约压缩一半，不能写成“压缩至 25%”而没有实测基线。
- 合成语音下短唤醒词仍可能误识别；生产环境应接 sherpa-onnx 独立 KWS。
- AEC、P95 延迟和 UART/SPI 仍需在目标机器人硬件上验收。

项目采用 [Apache License 2.0](LICENSE)，GitHub Actions 运行仓库约束及 ROS 2 Jazzy
build/test；本地 release gate 继续负责需要模型、云 API、Gazebo 与真人麦克风的链路。
