# 阶段架构审计

决策审计基线：2026-07-13，本地复核更新：2026-07-15。当前可计算规模以自动生成的
[架构事实报告](evidence/architecture_facts.md) 为准；本文不再把会随提交变化的数量作为长期契约。

## 1. 当前结论

项目已形成清晰的领域、输入 Adapter、部署、控制和仿真边界，适合继续作为 ROS 2/C++
求职展示工程维护。后续优化应由真实变化触发，不再单纯按文件行数拆分。

当前依赖方向：

```text
embodied_agent_interfaces
  ├── embodied_agent_middleware
  ├── embodied_agent_core
  │     └── embodied_voice_frontend
  └── embodied_agent_cpp

embodied_agent_bringup
  ├── embodied_agent_core
  ├── embodied_voice_frontend
  └── embodied_agent_cpp

embodied_online_agent / embodied_offline_agent
  ├── embodied_agent_core
  └── embodied_agent_bringup

embodied_simulation
  ├── embodied_agent_middleware
  ├── embodied_agent_bringup
  └── online/offline Agent（仅组合演示 launch）

embodied_slam / embodied_slam_tools
  ├── slam_toolbox / Karto Adapter
  ├── GTSAM 位姿图与 LiDAR 回环深模块
  └── 离线语料、消融与证据脚本

embodied_navigation
  ├── embodied_agent_interfaces
  ├── 动态目标跟踪与运动模型
  └── Nav2 predicted costmap plugin
```

领域 core 不再依赖 launch、具体 Agent 或仿真包；bringup 是语音控制装配根，simulation 是执行演示
集成根。SLAM 和动态导航保持独立算法包，通过 typed message、pluginlib 和 Nav2/slam_toolbox 接口接入。

## 2. 已收敛的关键边界

- `embodied_agent_core`：连续会话、NLU、执行、Lifecycle、用户上下文、记忆和 ROS I/O。
- `embodied_voice_frontend`：Silero/WebRTC VAD、KWS、声纹输入 Adapter。
- `embodied_agent_bringup`：Agent 参数、语音前端节点和安全部署拓扑的 launch contract。
- `embodied_agent_middleware`：Python/C++ 共用的命名 QoS 语义与系统 readiness。
- `ActionGuard + ActionScheduler`：强类型动作安全、FIFO、取消、超时和结果关联。
- `ActiveActionRuntime`：仿真长动作的唯一生命周期所有者。
- `SimulationRosIo`：仿真 managed publisher、QoS、ACK/BT/diagnostics 状态映射。
- `RobotExecutor`：Gazebo、Mock、Nav2 三个独立 pluginlib 后端。
- `GtsamPoseGraphOptimizer + GtsamScanSolver`：纯优化深模块与 Karto Adapter 分离。
- `LiDAR loop frontend`：候选、几何验证、序列门控和 shadow/commit 分层。
- `DynamicObstacleTracker`：全局关联与四种运动模型共享稳定 `update()` 接口。
- `PredictedObstacleLayer`：将未来占用写入 Nav2 costmap，并负责旧区域清除。
- `SessionOrchestratorNode + StageProcessManager`：自动 frontier 建图、存图、AMCL/Nav2 切换和语义巡检。
- `embodied_resolve_workspace + embodied_workspace_doctor`：保证源码、install、Action 与 Explore Lite 来自同一 worktree。
- `voice_control_profile.sh`：真实语音 profile 的唯一默认值解析器。
- `tests/repository`：按 architecture、delivery、voice runtime 分区的仓库契约。

## 3. 当前保留的大文件

### online/offline Agent 节点

两个节点的当前行数由架构事实报告从源码计算；共享状态机、Lifecycle、ROS I/O、执行、端点和用户
上下文已经下沉。剩余内容主要是 provider 创建、在线/离线 ASR 差异、TTS/LLM 调用和 ROS callback
适配。

只有出现以下变化时再继续拆分：

- 新增第三种 Agent provider，需要复用现有节点 Adapter；
- provider 初始化/关闭再次出现在线离线重复；
- 单元测试必须实例化完整 ROS 节点才能覆盖 provider 逻辑。

### `acceptance_test.sh`

该脚本的公开/高级/router mode 数量由架构事实报告计算，当前定位是稳定 CLI router，具体实现已经
分散在独立脚本和 Python 探针中。默认帮助只保留稳定公共入口；新增模式必须同时满足帮助可发现、
case 可路由和 release-gate 契约，精确数量由自动架构事实生成，repository test 会检查三者是否漂移。

### `command_nlu.py`

当前同时包含意图原型、小型字符 n-gram 模型和槽位解析。下一次扩充大量语料时，应把原型数据迁移到
版本化 JSON/YAML；在现有命令规模下保留单模块更方便审查安全语义。

## 4. 阶段验收证据

```bash
bash scripts/acceptance_test.sh mock
```

长期有效的验收契约：

- GitHub Actions 构建矩阵必须与全部 ROS 2 `package.xml` 一致；
- `robotics-gate` 必须覆盖仓库/Agent/C++、连续多命令、Nav2、SLAM、OpenLORIS fixture 和动态障碍；
- 公开帮助保持精简，高级帮助中的每个入口都必须可路由；精确模式数量由架构事实报告生成；
- 精确包数、模式数、节点行数和 typed interface 数见自动生成报告；单次测试通过数量只保留在 CI/终端
  证据中，不再复制成会过期的架构事实；
- 最终控制链路必须回到零速度。

2026-07-15 本地阶段测试快照（仅作为历史证据，不替代当前 CI）：

- 12 个 ROS 2 包完成构建，包含独立 SLAM、工具和动态导航包；
- 仓库结构与交付契约 180 项通过；
- Python Agent 快速测试 237 项通过；
- ROS/C++/Python 包汇总 558 项，0 error、0 failure、0 skipped；
- 连续语音、端点提交、短命令补全、多命令队列、急停、Nav2 mock、pluginlib、硬件 mock、
  Lifecycle 和雷达安全冒烟全部通过；SLAM、LiDAR 回环和动态导航另有分层测试与重型验收；
- 最终控制链路能回到零速度。

KWS calibration 首轮曾因启动发现窗口没有采到样本，单项复跑采到 22 个样本，随后完整 mock 复跑通过。
真实麦克风、云 API、OpenLORIS、Gazebo GUI 和完整 Nav2 TurtleBot3 仍属于人工或重型验收，不能由
mock 结果替代。

## 5. 下一阶段触发条件

优先处理可量化问题：自动 frontier 覆盖率/耗时、地图质量、定位恢复、真实 ASR 样本准确率、离线
模型延迟和 Gazebo/Nav2 重型演示回归。若这些证据稳定，架构层不再主动扩大改动面。
