# Embodied Voice Agent for ROS 2

面向 ROS 2/C++ 求职展示的语音具身智能项目。系统同时提供在线与离线 Agent，并打通：

```text
麦克风 → VAD/ASR → 会话与 NLU/LLM → typed ROS 2 Action
→ C++ ActionGuard/调度 → BehaviorTree.CPP/pluginlib → Gazebo/Nav2/SLAM
```

项目当前以 TurtleBot3 仿真为主；UART/SPI 只保留 Adapter/mock，不宣称已完成实体硬件验收。

## 能力概览

- 在线/离线 Agent：云端 provider，以及 Sherpa-ONNX ZipFormer、llama.cpp、Sherpa-TTS；SummerTTS
  是可选 ROS 服务组件。
- 连续语音：一次唤醒后持续接收，多命令 NLU、FIFO、TTL、重复/filler 过滤和急停抢占。
- ROS 2/C++ 控制：自定义 msg/action、Lifecycle、ActionGuard、ActionScheduler、反馈/取消/超时。
- 仿真执行：BehaviorTree.CPP 编排、pluginlib executor、Gazebo 运动和最终零速度保护。
- SLAM/Nav2：frontier 探索、SLAM Toolbox、地图保存、AMCL、Nav2 目标导航与动态障碍重规划。
- 算法证据：Ceres/GTSAM 后端、LiDAR 回环 shadow pipeline、动态障碍关联/预测/costmap 消融。

当前 schema v4 session `20260721T072342Z-2344751-5452a492` 已完成 unknown-world 探索、动态起点返航、
本次地图定位、3 个运行时目标、动态重规划与停车：覆盖 `99.81%`、区域最低 `98.76%`、AMCL P95
`0.120m`，全部 checks 为 true。机器人运行时不读取真值；truth 只供验收结束后的 evaluator 复核。

## 核心架构

```mermaid
flowchart LR
    MIC["WSLg 麦克风"] --> FE["Audio Frontend / VAD"]
    FE --> ASR["在线 ASR / Sherpa ZipFormer"]
    ASR --> AGENT["Session + NLU/LLM"]
    AGENT --> CMD["RobotCommand"]
    CMD --> GUARD["C++ ActionGuard + Scheduler"]
    GUARD --> ACTION["ExecuteRobotCommand Action"]
    ACTION --> BT["BehaviorTree.CPP"]
    BT --> EXEC["pluginlib RobotExecutor"]
    EXEC --> GZEXEC["GazeboRobotExecutor"]
    EXEC --> NAVEXEC["Nav2RobotExecutor"]
    GZEXEC --> GAZEBO["Gazebo"]
    NAVEXEC --> NAV2["Nav2 Action servers"]
    GAZEBO --> SENSOR["Gazebo sensors / scan / odom / TF"]
    SENSOR --> SLAM["SLAM Toolbox / frontier policy"]
    SLAM --> MAP["本次会话地图"]
    MAP --> AMCL["map_server + AMCL"]
    AMCL --> NAV2
```

关键原则：Agent 负责意图，ActionGuard 负责 primitive 动作安全，executor 负责副作用。初始/恢复扫描
与安全 STOP 复用 Agent → Guard → BT；Unknown-world 的运行时采样目标则由任务编排器完成 known-free
预筛和 `ComputePathToPose` 准入后，直接调用 typed Nav2 Action，避免把地图坐标重新翻译成自然语言。
探索与采样策略只能读取在线 scan/odom/TF/map；静态真值和场地区域只允许进入评分器。

## 目录

| 路径 | 作用 |
| --- | --- |
| `src/embodied_agent_interfaces` | 自定义 msg/action/service schema |
| `src/embodied_agent_core` | 会话、NLU、队列、记忆与共享运行时 |
| `src/embodied_agent_bringup` | 公共 launch 参数和 Lifecycle 拓扑 |
| `src/embodied_voice_frontend` | VAD、KWS、声纹等输入 Adapter |
| `src/embodied_online_agent` / `src/embodied_offline_agent` | 在线/离线 provider Adapter |
| `src/embodied_agent_cpp` | C++ 音频、安全、调度和硬件 seam |
| `src/embodied_simulation` | BT、pluginlib、Gazebo/Nav2 executor 与场景 |
| `src/embodied_slam_tools` | 自动建图任务、证据状态与阶段进程 |
| `src/embodied_slam` / `src/embodied_navigation` | SLAM 后端/回环与动态障碍算法 |
| `Dockerfile` / `compose.yaml` / `docker` | 多阶段构建、容器测试门禁与运行镜像入口 |
| `tools/acceptance` | 验收注册表、session、probe、evaluator 与报告判定 |
| `tests` | pytest/GTest、仓库契约和确定性 evaluation |

## 部署

推荐环境：WSL Ubuntu 24.04、ROS 2 Jazzy、Python 3.12、Gazebo Harmonic。

```bash
git clone git@github.com:Edddddddddy/embodied_agent_ws.git
cd embodied_agent_ws
bash scripts/bootstrap.sh
source scripts/activate.sh
```

离线模型和固定版本 Explore Lite：

```bash
bash scripts/setup_offline_runtime.sh
bash scripts/setup_frontier_exploration.sh
bash scripts/acceptance_test.sh offline-runtime-versions
```

在线模式在 `.env` 配置 `DASHSCOPE_API_KEY`。密钥、模型、`build/install/log` 不提交 Git。

### Docker 构建与可追溯交付

```bash
docker compose build test && docker compose run --rm test
docker compose build runtime-smoke && docker compose run --rm runtime-smoke
```

PR 和 `dev/main` push 执行相同容器门禁；合入 `main` 后的稳定 SemVer 标签才允许发布 GHCR 运行镜像，
并保存 digest、Git revision、OCI labels 和发布 manifest。详见 [Docker 与交付流程](docs/deployment/CONTAINER_DELIVERY.md)。

本阶段扩展了 `SlamSessionState`、`SlamMappingCompletionEvidence` 等 SLAM 证据接口；ROS 2 接口
type hash 已变化。切换分支后须全量重建依赖包，旧 overlay 或旧 rosbag 不能作为当前证据。

```bash
# 重新配置全部自有包；不在文档里提供可能误删其他 worktree 的递归删除命令。
colcon build --symlink-install --cmake-clean-cache --executor sequential
source install/setup.bash
embodied_workspace_doctor true
```

重型验收会隔离外部 Nav2 overlay，并在 manifest 记录包来源。linked worktree 的真人离线入口自动复用
Git 主工作区的 llama/GGUF/VAD/校准资产；自定义位置用 `EMBODIED_RUNTIME_ROOT` 覆盖。

## 推荐演示

### 1. Unknown-world 正式自主闭环

```bash
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh unknown-world-slam-e2e
```

机器人运行时不读取真值地图、bootstrap 路线或预生成语义坐标；truth 只由验收结束后的 evaluator 使用。
任务需完成可达空间探索、动态起点返航、本次地图定位、至少 3 个运行时采样目标、动态重规划和最终零速。
证据保存在 `logs/acceptance/unknown_world_slam_nav/<session_id>`；报告必须为 schema v4 且全部 checks
为 true。探索收口、typed STOP、地图质量门槛和失败链见 [TESTING.md](docs/TESTING.md) 与
[Navigation 证据索引](docs/evidence/navigation/README.md)。

### 2. 真人语音 + unknown-world 联合验收（待现场）

```bash
bash scripts/acceptance_test.sh wsl-microphone-preflight
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-unknown-world-slam-e2e offline
# 将 offline 换成 online 可补跑在线 Agent
```

看到提示后说“**小智，开始自动巡检建图**”。真人编排器只消费通过唤醒门的 WakeEvent；synthetic 核心
入口才使用 raw ASR。报告写入
`logs/acceptance/voice_unknown_world_slam_nav/<session_id>/voice_unknown_world_slam_e2e_report.json`；
schema v1 的五个语音 checks 与内嵌同 session strict v4 必须全部为 true。当前尚无真人现场 PASS；完整
契约和故障定位见 [TESTING.md §5.5](docs/TESTING.md)。

正常冷启动时，离线模型 warmup 可能持续十几秒；终端会每 5 秒打印一次 `WAIT: system readiness` 及缺失
组件。若阶段子进程失败，入口会立即输出原始原因并生成失败报告，不再留下“界面已开但车不动”的静默等待。

### 3. Known-world 稳定回归与语音交互

```bash
bash scripts/acceptance_test.sh slam-nav-e2e
bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
# 或 online
```

`slam-nav-e2e` 保留已知场景的确定性集成回归，语音 demo 验证真人触发和阶段交互。它们可能使用场景
bootstrap/语义地点，不能作为“机器人面对未知环境自主完成探索”的证据。

### 4. 连续语音控制

```bash
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh continuous-online
```

推荐序列：`小智` → `向前走一秒` → `左转九十度` → `向右转，然后向前走一秒` → `停下` →
`退出控制`。该入口验证语音、队列和基础仿真控制，不代替 SLAM/Nav2 E2E。

## 测试与验收

公开入口共 9 个，实际列表以脚本帮助和注册表为准：

```bash
bash scripts/acceptance_test.sh --help
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh continuous-online
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh slam-nav-e2e
bash scripts/acceptance_test.sh unknown-world-slam-e2e
bash scripts/acceptance_test.sh voice-unknown-world-slam-e2e offline
bash scripts/acceptance_test.sh robotics-gate
```

`--help-all` 仅用于维护内部回归和实验模式。分层测试、严格阈值和故障排查见
[TESTING.md](docs/TESTING.md)。

常用补充入口：`offline-latency`、`offline-voice-e2e-report`、`release-gate`。

`offline-latency` 的组件目标为 LLM 首 token `≤ 1000ms`、短句完整 TTS 合成 `≤ 600ms`，不等于
真人语音整链路。release/demo gate 产物分别为 `logs/acceptance_report.json`、
`logs/demo_acceptance_report.json`。麦克风异常先运行 `wsl-microphone-preflight` 和
`voice-calibration-report`，再用
`APPLY_VOICE_CALIBRATION=true bash scripts/acceptance_test.sh continuous-offline` 应用建议；产物为
`logs/audio_calibration.json`、`logs/voice_calibration_report.json` 与 `logs/voice_calibration.env`。
需要留存连续识别样本时设置 `CONTINUOUS_SAMPLE_LOG=<path>`。

## 文档

从 [文档阅读地图](docs/README.md) 开始：先用本页完成项目定位、最短部署和公开入口，再按目标进入
[系统架构](docs/ARCHITECTURE.md)、[测试手册](docs/TESTING.md)、学习笔记、证据或
[15 分钟汇报](docs/PRESENTATION_15MIN.md)。开发日志和历史记录不作为当前功能事实。

## 事实边界

- 仿真证据不等于实体硬件证据；mock、真人语音、Gazebo 和公开 bag 也不能相互替代。
- LoRA/Q8、延迟和识别准确率只引用对应报告，不从 fixture 或单次 smoke 外推。
- SummerTTS 已组件化接入；默认低延迟离线链路仍以 Sherpa-TTS 为主。
- 历史 PASS 不自动继承到新接口、新地图或新验收 schema。

许可证与第三方依赖遵循各子项目声明。
