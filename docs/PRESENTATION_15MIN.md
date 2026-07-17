# 15 分钟项目汇报与代码演示

目标：用一句语音展示“自动建图 → 保存本次地图 → AMCL/Nav2 → 语义巡检”，再用同场景的
`slam-nav-e2e` 报告讲解确定性动态障碍重规划，并能打开关键代码解释设计。项目正式验收对象是
Gazebo/TurtleBot3，不把 UART/SPI seam 说成实机交付。

## 1. 演示前准备

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh slam-nav-showcase-stage
bash scripts/acceptance_test.sh wsl-microphone-preflight
```

主演示：

```bash
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

看到 Gazebo、LaserScan 和系统 ready 后，只说：`小智，开始自动巡检建图`。备用无麦克风门禁：

```bash
bash scripts/acceptance_test.sh slam-nav-e2e
```

保留最近一次成功报告、地图和三张截图；现场失败时切换到证据讲解，不连续重启大型进程。

## 2. 时间线

| 时间 | 内容 | 现场/代码 |
| --- | --- | --- |
| 0:00–1:00 | 问题与结果 | 说启动句，展示目标 |
| 1:00–2:15 | 总体架构 | `docs/ARCHITECTURE.md` |
| 2:15–3:30 | 语音前端 | VAD/ASR commit |
| 3:30–5:00 | Agent/队列 | NLU、多命令、FIFO、急停 |
| 5:00–6:30 | C++ 安全控制 | ActionGuard/Scheduler/BT |
| 6:30–8:15 | 自动任务事务 | mission executor/process adapter |
| 8:15–9:45 | frontier 建图 | map growth、结束判定 |
| 9:45–11:15 | 存图与 Nav2 | AMCL、TF、语义地点 |
| 11:15–12:45 | 回环/后端/动态障碍 | GTSAM、costmap |
| 12:45–14:00 | 验收证据 | JSON、Action result、零速 |
| 14:00–15:00 | 边界与总结 | 已完成/未完成 |

## 3. 可直接照讲的内容

### 3.1 项目目标（0:00–1:00）

> 这不是单独的语音识别 Demo。在线和离线 Agent 共用一条 typed ROS 2 控制面；用户一句话触发未知环境探索、存图、定位切换和多点巡检。LLM 只产生候选动作，不能直接写 `/cmd_vel`。

展示：Gazebo 场景、RViz 地图增长和终端 session state。

### 3.2 总体架构（1:00–2:15）

```text
Audio/VAD → ASR → Continuous Session + NLU/LLM
→ RobotCommand → C++ ActionGuard → ActionScheduler
→ ExecuteRobotCommand → BT/pluginlib → Gazebo/Nav2
```

代码锚点：`src/embodied_agent_interfaces/msg/RobotCommand.msg`、
`src/embodied_agent_interfaces/action/ExecuteRobotCommand.action`。强调 typed 接口支持字段校验、
rosbag、Action feedback/cancel/result，已删除 JSON 控制协议。

### 3.3 语音前端（2:15–3:30）

打开：
- `src/embodied_agent_cpp/src/audio_frontend_node.cpp`：PCM、增强、VAD 事件。
- `src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py`：
  `AsrEndpointRuntime` 管理 endpoint 去重、代次、延迟提交，以及恢复说话时取消待触发 timer。
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py` 与
  `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`：
  `_commit_asr_endpoint()` 把共享决策适配到在线/离线 ASR provider。
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_asr.py`：
  `SherpaZipformerAsr.push_audio()/commit()` 的离线 provider Adapter。

讲法：VAD endpoint 与 ASR final 是两个时刻；尾静音和 commit delay 保护数字/量词，短命令补全只作用于明确控制语义。

### 3.4 Agent、NLU 与连续队列（3:30–5:00）

打开 `agent_application_runtime.py:AgentApplicationRuntime.accept_transcript()`、
`agent_control_plane.py:AgentControlPlane.accept_transcript()/enqueue_command()`、
`agent_execution_runtime.py:AgentExecutionRuntime._run_worker()` 和 `command_nlu.py:CommandNLU.parse()`。
说明一次唤醒后的 session gate、重复/filler 过滤、batch_id、FIFO/TTL、执行中继续入队、急停抢占。
结果通过 `SequentialActionPublisher.notify_result()` 按 `command_id` 释放下一项；低置信度才回退 LLM，
安全不依赖 LLM 自觉。

### 3.5 C++ 安全与执行（5:00–6:30）

打开：
- `src/embodied_agent_cpp/src/action_guard_node.cpp:ActionGuardNode::on_candidate()`；
- `src/embodied_agent_cpp/src/action_scheduler.cpp`；
- `src/embodied_simulation/src/robot_command_policy.cpp:is_executable_robot_command()`；
- `src/embodied_simulation/src/simulation_control_node.cpp`。

ActionGuard 做白名单、限幅和字段互斥；Scheduler 用 command_id 关联 active goal，急停先 cancel 再清队列。readiness 只有 Agent→Guard→Scheduler 两段 DDS 均匹配才 ready，避免首条 volatile 命令在 discovery 窗口丢失。

### 3.6 自动任务事务（6:30–8:15）

打开：
- `showcase_session.py:ShowcaseSessionStateMachine`：合法状态转换；
- `mission_configuration.py:MissionConfiguration.load()`：任务 YAML 如何收紧为可执行配置；
- `mission_executor.py:AutomaticMissionExecutor.run()`：任务顺序；
- `slam_nav_evidence.py:build_automatic_mission_report()`：为何最终 PASS 不是日志判断；
- `stage_process_manager.py:StageProcessManager`：launch/map saver/清理；
- `agent_action_gateway.py:AgentActionGateway.run()`：候选与结果代次关联。

讲法：领域层不依赖 rclpy，ROS Node 只做 Adapter。`finally` 保证 explorer、里程采集和进程树被清理；取消与故障是显式结果，不是卡住。

### 3.7 Frontier 自动建图（8:15–9:45）

打开 `frontier_monitor.py:FrontierExplorationMonitor.wait()` 和 `mapping_evidence.py:MappingEvidenceTracker`。

Explore Lite 在 free/unknown 边界聚类候选并通过 Nav2 到达；SLAM Toolbox 负责位姿/地图，不负责探索决策。结束必须同时满足最短运行、已知/占用栅格、真实建图里程，并给出 `no_frontiers`、`coverage_plateau` 或 `time_budget_coverage`。

### 3.8 存图、定位与语义巡检（9:45–11:15）

map saver 生成本次 YAML/PGM 后完全关闭 mapping stage，再从本会话的 `map_prefix` 加载刚保存的
YAML，启动 map_server、AMCL 和 Nav2。报告只在事后记录地图来源，不参与运行配置。
`wait_navigation_ready()` 检查 NavigateToPose/FollowWaypoints server 与 lifecycle ACTIVE。C++
`Nav2RobotExecutor` 将入口/厨房/办公室转换成 PoseStamped。

### 3.9 后端优化、回环与动态障碍（11:15–12:45）

打开 `src/embodied_slam` 的 pose graph/GTSAM 与 loop verification，及 `src/embodied_navigation` 的 tracker/predicted costmap。

真人语音主演示会启用预测代价层，但不自动注入测试用 `crossing_cart`。`slam-nav-e2e` 的确定性横穿、
typed detection、路径变化与安全间距由以下 Module 协作产生：

- `tools/acceptance/probes/slam_nav/session_observer.py`：`SessionObserver` 采集 typed ROS 事实；
- `tools/acceptance/probes/slam_nav/dynamic_scenario.py`：`run_showcase_dynamic_navigation()` 执行
  障碍注入、重规划和失败恢复事务；
- `tools/acceptance/dynamic_route.py`：`select_replannable_route()`；
- `tools/acceptance/slam_nav_evidence.py`：`evaluate_dynamic_navigation()`。

讲法：回环不是相似就加边，而是候选检索→几何/scan-overlap 验证→一致性门→鲁棒核/可切换约束→图优化。动态障碍由观测关联、速度估计、未来占用投影进入 costmap；Nav2 仍负责最终规划控制。公开 bag/Gazebo 结果不能冒充真实场地漂移。

### 3.10 证据与总结（12:45–15:00）

打开 `logs/acceptance/slam_nav/<session_id>/slam_nav_e2e_report.json`，依次指出：地图 SHA256/时间、frontier goal、建图里程、AMCL/TF、lifecycle、语义 Action result、动态安全间距/unique plans、最终零速度。

代码只讲一条单向依赖：`session_orchestrator.py` 是唯一可执行入口并调用 ROS-free `cli.py` 参数
Interface；`SessionObserver` 是 typed ROS Adapter，`dynamic_scenario.py` 管场景事务，`artifacts.py` 管地图哈希与
摘要；只有 ROS-free `slam_nav_evidence.py` 能作出最终 PASS/FAIL。这样运行时采样、流程控制和验收标准
不会互相反向依赖。

> 项目价值在于把不确定语音与确定机器人控制解耦，并用强类型接口、状态机、生命周期、BT/pluginlib、Nav2 和可审计证据形成工程闭环。当前完成的是仿真平台；真实硬件、真实场地长期漂移和大规模训练仍是边界。

## 4. 追问代码锚点

| 追问 | 文件/函数 |
| --- | --- |
| 为什么不用 JSON | `RobotCommand.msg`、`ExecuteRobotCommand.action` |
| 如何防 LLM 危险动作 | `action_validator.cpp`、`ActionGuardNode::on_candidate()` |
| 多命令如何排队 | `command_nlu.py`、`agent_control_plane.py` |
| 结果会不会串台 | `AgentActionGateway`、`ActionScheduler` |
| 如何自动建图 | `AutomaticMissionExecutor`、`FrontierExplorationMonitor` |
| 如何证明不是旧地图 | `artifacts.py:map_artifact_sha256()` + `slam_nav_evidence.py` 的 provenance/checks |
| 回环怎么防误检 | loop verification、switchable constraint、GTSAM optimizer |
| 动态障碍怎么影响规划 | tracker → prediction → PredictedObstacleLayer |
| 为什么最终一定停车 | 急停、Action result、executor cleanup、报告零速检查 |

## 5. 现场清单

- 开始前：清理旧进程，确认 overlay、麦克风、磁盘和 API key（在线时）。
- 演示中：保留 Gazebo/RViz、session 日志和代码窗口；不临时改参数。
- 结束后：确认报告 `passed=true` 与最终零速；失败时展示 error 与边界。
