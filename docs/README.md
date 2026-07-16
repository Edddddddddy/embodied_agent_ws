# 项目文档中心

本文是 `docs/` 顶层文档的唯一入口，面向第一次部署、现场验收、代码走读和后续维护。根目录
`README.md` 负责项目概览；本文说明当前完成度、推荐阅读顺序、运行入口及各文档的权威边界。

项目当前主线是：

```text
真实语音
→ 在线/离线 ASR
→ 会话、NLU/LLM 与命令队列
→ typed RobotCommand
→ C++ ActionGuard 与 ActionScheduler
→ ROS 2 Action
→ BehaviorTree.CPP / pluginlib Executor
→ Gazebo、SLAM、AMCL 与 Nav2
```

## 1. 当前完成度

| 能力 | 当前状态 | 证据边界 |
| --- | --- | --- |
| 在线与离线 Agent | 已实现 | 在线依赖有效 API；离线依赖本地 Sherpa、llama.cpp 和 TTS 资产 |
| 连续语音和多命令 | 已实现 | 自动回归不能替代当前麦克风、声场和 VAD 的人工验证 |
| typed ROS 2 控制链 | 已实现 | Agent 到执行层使用自定义 msg/action，不用 JSON 作为运行时控制协议 |
| C++ 安全与调度 | 已实现 | ActionGuard 负责白名单和限幅，ActionScheduler 负责 FIFO、抢占和结果关联 |
| Gazebo 仿真动作 | 已实现 | 仿真通过不等于 UART/SPI 实体硬件通过 |
| 自动建图与导航 | 已实现工程闭环 | 本机仍需通过依赖准备、Gazebo、frontier 探索和 Nav2 的重型验收 |
| SLAM 后端与回环实验 | 已实现分层实验 | LiDAR 新回环仍默认 shadow-only，不宣称已安全写入生产图 |
| 动态障碍预测 | 已实现仿真与消融 | 当前证据以合成检测输入和 Gazebo/Nav2 为主 |
| LoRA/Q8 与离线指标 | 已有可复现实验 | 合成 holdout 不能代替真实语音分布准确率 |
| 真实硬件 | Adapter/mock | 不属于当前阶段完成项 |

“代码已实现”“自动门禁通过”“本机真实演示通过”是三种不同结论。当前机器上的最终完成度应以
[测试与验收手册](TESTING_AND_ACCEPTANCE.md) 和
[运行时证据状态](RUNTIME_EVIDENCE_STATUS.md) 为准，不能只根据功能文件存在或历史报告判断。

## 2. 推荐阅读路径

### 2.1 第一次运行

依次阅读本文的首次部署、[测试与验收手册](TESTING_AND_ACCEPTANCE.md)、
[WSL + PowerShell 开发说明](CODEX_WSL_POWERSHELL_SKILL.md) 和
[运行时证据状态](RUNTIME_EVIDENCE_STATUS.md)。

### 2.2 代码走读或面试

先看 [架构与模块说明](ARCHITECTURE_AND_KNOWLEDGE.md)，再沿
[语音到仿真代码走读](VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md) 打开源码；随后用
[学习笔记](LEARNING_NOTES.md)、[15 分钟汇报稿](PROJECT_PRESENTATION_15MIN.md) 和
[面试问答](INTERVIEW_QA.md) 解释取舍并组织表达。

### 2.3 SLAM、自动探索和导航

先看 [语音 SLAM/Nav2 演示](VOICE_SLAM_NAV_SHOWCASE.md)，再看
[SLAM 与导航工程笔记](SLAM_NAVIGATION_ENGINEERING.md)；公开数据和 ATE/RPE 证据见
[真实数据 SLAM 评估](REAL_WORLD_SLAM_EVALUATION.md)。

### 2.4 离线端侧部署

[离线运行时版本](OFFLINE_RUNTIME_VERSIONS.md) 说明固定依赖，
[Sherpa-ONNX 部署](SHERPA_ONNX_DEPLOYMENT.md) 说明 ASR 资产与排障，
[离线 Benchmark](OFFLINE_BENCHMARK_REPORT.md) 说明指标口径。

## 3. 首次部署与工作区激活

默认环境是 WSL Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Sim。首次部署必须先构建工作区，不能在
`install/setup.bash` 尚未生成时直接执行激活脚本。

```bash
cd /home/ubuntu/embodied_agent_ws
bash scripts/bootstrap.sh
source scripts/activate.sh
```

`bootstrap.sh` 会安装基础系统依赖、创建 `.venv`、安装 Python 依赖、执行
`colcon build --symlink-install`，并默认安装固定提交的 Explore Lite。日常重新进入终端时只需要：

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh
```

公共入口会通过 `scripts/lifecycle_utils.sh:embodied_resolve_workspace()` 从入口脚本自身推导当前 repo/worktree，并 export
`WORKSPACE`；同一终端切换 worktree 后若残留旧值会明确失败，此时先 `unset WORKSPACE`。CI 若确需
跨目录覆盖，需同时设置 `EMBODIED_ALLOW_WORKSPACE_OVERRIDE=true`。激活器会从同一目录读取
`.venv`、`install/setup.bash` 和 `.env`，并恢复调用终端原有的 shell 选项。

若 `source scripts/activate.sh` 失败，依次检查：

```bash
test -f /opt/ros/jazzy/setup.bash
test -f .venv/bin/activate
test -f install/setup.bash
```

缺少 `install/setup.bash` 时重新构建：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source scripts/activate.sh
```

激活后先确认 CLI 的当前公开入口，不从旧笔记复制模式名：

```bash
bash scripts/acceptance_test.sh --help
```

完整自动建图演示再运行部署一致性检查：

```bash
embodied_workspace_doctor true
```

高级、诊断和兼容入口可通过 `--help-all` 查询，但不作为本文的默认验收路径。

## 4. 自动建图与导航主演示

首次运行自动探索前安装项目固定提交的 Explore Lite：

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh
bash scripts/setup_frontier_exploration.sh
embodied_workspace_doctor true
bash scripts/acceptance_test.sh slam-nav-showcase-stage
```

启动带 Gazebo 和 RViz 的离线主演示：

```bash
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

系统就绪后只需说：

```text
小智，开始自动巡检建图
```

运行方法不是整场固定速度路线回放。机器人先执行 7 段可审计、受 ActionGuard 保护的 move/turn
脱角原语，避免出生在充电角时 frontier 规划被局部墙体困住；进入开阔区域后，Explore Lite 从在线
占据栅格提取未知—已知边界，按信息增益和路径代价自主选择 frontier，再通过 Nav2
`NavigateToPose` 规划、控制和避障。无可达 frontier 后，会话编排器保存 YAML/PGM 地图，切换到
AMCL 定位和 Nav2 导航，并执行入口、厨房、办公室语义任务。

自动任务的主要函数调用关系是：

```text
showcase_session_node.py: SessionOrchestratorNode._on_asr_final()
→ showcase_session.py: parse_session_command()
→ SessionOrchestratorNode._enqueue()
→ SessionOrchestratorNode._worker_loop()
→ SessionOrchestratorNode._start_mapping()
→ SessionOrchestratorNode._execute_request()
→ SessionOrchestratorNode._run_automatic_mission()
→ showcase_session.py: parse_mapping_bootstrap_route()
→ SessionOrchestratorNode._run_agent_text_action()  # 脱离充电角
→ StageProcessManager.start_explorer()
→ SessionOrchestratorNode._wait_for_frontier_completion()
→ SessionOrchestratorNode._save_map()
→ SessionOrchestratorNode._start_navigation()
→ SessionOrchestratorNode._run_agent_text_action()
```

仅初始脱角原语走 Agent → ActionGuard → ROS 2 Action 主链；之后的 frontier 目标由 Explore Lite
直接交给 Nav2，不经过 LLM ActionGuard，但仍受 Nav2
costmap、规划器、控制器和恢复行为约束。存图后的语义导航重新进入项目主控制链：

```text
AgentApplicationRuntime.accept_transcript()
→ AgentControlPlane / ContinuousCommandQueue
→ StreamingTurnRuntime / CommandNLU
→ /agent/action_candidate
→ ActionGuardNode::on_candidate()
→ ActionScheduler::enqueue()
→ ExecuteRobotCommand Action
→ CommandBehaviorTree::tick()
→ Nav2RobotExecutor
→ NavigateToPose / FollowWaypoints
```

详细状态机、topic、Action、取消和回退方法见
[真实感语音 SLAM/Nav2 演示](VOICE_SLAM_NAV_SHOWCASE.md)。任务中说“停下”“急停”或
“取消自动任务”应终止当前探索/导航并最终让 `/cmd_vel` 归零。

## 5. 验收层级

### 5.1 快速开发门禁

```bash
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh continuous-multi-command
```

它们证明仓库契约、主要单元测试和一句多命令队列行为，不证明 Gazebo 或真实麦克风可用。

### 5.2 机器人与算法阶段门禁

```bash
bash scripts/acceptance_test.sh robotics-gate
bash scripts/acceptance_test.sh nav2-stage
bash scripts/acceptance_test.sh slam-nav-showcase-stage
bash scripts/acceptance_test.sh slam-evaluation-stage
bash scripts/acceptance_test.sh dynamic-obstacle-stage
```

阶段门禁用于定位接口、状态机、指标数学和插件接线问题；其中的 mock/fixture 证据不能替代重型运行。

### 5.3 Gazebo 与真实语音

```bash
bash scripts/acceptance_test.sh gazebo
bash scripts/acceptance_test.sh continuous-offline
bash scripts/acceptance_test.sh continuous-online
```

真实语音验收应观察 ASR final、队列事件、Action feedback/result、机器人运动及最终零速。
在线模式还依赖有效 API 和网络；离线模式依赖本地模型资产。

### 5.4 自动建图导航与动态避障

```bash
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
bash scripts/acceptance_test.sh dynamic-obstacle-navigation
```

这一级必须观察在线 `/map`、地图产物、`map→odom`、Nav2 Action 成功、里程计变化和最终停车。

### 5.5 公开数据证据

```bash
bash scripts/acceptance_test.sh openloris-replay-stage
```

公开数据阶段用于证明回放 Adapter、来源和报告契约；小 fixture 通过不代表真实序列精度达标。

## 6. 顶层文档分类与权威性

| 类别 | 文档 | 权威边界 |
| --- | --- | --- |
| 当前行为 | [架构](ARCHITECTURE_AND_KNOWLEDGE.md)、[测试](TESTING_AND_ACCEPTANCE.md)、[主演示](VOICE_SLAM_NAV_SHOWCASE.md)、[运行证据](RUNTIME_EVIDENCE_STATUS.md) | 分别定义职责、PASS/FAIL、操作和已测/缺失事实 |
| 原理走读 | [调用链](VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md)、[学习笔记](LEARNING_NOTES.md)、[SLAM/Nav](SLAM_NAVIGATION_ENGINEERING.md)、[真实数据](REAL_WORLD_SLAM_EVALUATION.md)、[架构图](FINAL_ARCHITECTURE_DIAGRAMS.md) | 用于解释技术；辅助图不单独证明运行结果 |
| 离线部署 | [运行时版本](OFFLINE_RUNTIME_VERSIONS.md)、[Sherpa 部署](SHERPA_ONNX_DEPLOYMENT.md)、[Benchmark](OFFLINE_BENCHMARK_REPORT.md) | 版本、安装和指标各自负责 |
| 汇报 | [15 分钟稿](PROJECT_PRESENTATION_15MIN.md)、[面试问答](INTERVIEW_QA.md) | 用于表达，不替代验收 |
| 历史维护 | [版本路线](CHANGELOG_AND_ROADMAP.md)、[不足](PROJECT_GAPS_AND_OPTIMIZATION.md)、[架构审计](ARCHITECTURE_AUDIT.md)、[知识图谱](CODEBASE_KNOWLEDGE_GRAPH.md)、[开发说明](CODEX_WSL_POWERSHELL_SKILL.md)、[Nav2 审计](NAV2_VOICE_ACCEPTANCE_AUDIT.md) | 历史快照或专题资料，不作为当前 CLI/运行事实 |

当文档与代码不一致时，接口定义、当前源码、`acceptance_test.sh --help` 和新生成报告优先。

## 7. 文档维护规则

- 新功能必须同时更新架构职责、精确代码调用链、验收命令和事实边界。
- 技术说明至少包含：功能、关键文件/类/函数、输入、输出、调用前后关系、设计原因和替代方案。
- 运行指标必须链接可重生成的报告；历史数字要标注日期，不能写成永久事实。
- mock、fixture、Gazebo、真实麦克风、公开 bag 和实体硬件证据必须分栏。
- 运行时 JSON 只作为报告格式；ROS 2 控制接口继续使用 typed msg/srv/action。
- 公开命令以 `bash scripts/acceptance_test.sh --help` 为准；高级模式不得悄悄成为新人必跑步骤。
- 更新完整功能后再提交并触发 GitHub CI，避免为零散文档修改频繁推送。
