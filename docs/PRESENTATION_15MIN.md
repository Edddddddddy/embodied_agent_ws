# 15 分钟项目汇报与代码演示

目标：在 15 分钟内讲清“语音 Agent 为什么不能直接控制机器人、unknown-world 如何自主建图导航、
系统如何用 typed evidence 证明成功”。现场采用双证据，不把两条链路混成一次实验：

| 证据 | 展示内容 | 能证明什么 | 本轮状态 |
| --- | --- | --- | --- |
| 真人语音 known-world | 麦克风→Agent→typed Action→Gazebo/Nav2 | 真实声学输入与交互控制 | 本轮未重跑，现场前单独验收 |
| Gazebo unknown-world 报告 | frontier 建图→存图→AMCL→动态三点→动态障碍 | 无场景先验的自主闭环与硬指标 | fresh session `20260720T031306Z-1114546-814430c3` PASS |

不要说“上面的 PASS session 包含真人语音”；它没有。硬件 UART/SPI 也是 Adapter seam，不属于当前
Gazebo 交付。

## 1. 演示前准备

```bash
cd ~/embodied_agent_ws
source scripts/activate.sh
bash scripts/cleanup_simulation_processes.sh
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh wsl-microphone-preflight
```

现场真人语音演示：

```bash
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

说：`小智，开始自动巡检建图`。这是 known-world 稳定交互演示，允许语义地点；它不能用来证明未知
环境自主探索。正式 unknown-world 入口是：

```bash
USE_RVIZ=true bash scripts/acceptance_test.sh unknown-world-slam-e2e
```

完整自主任务耗时较长，15 分钟汇报应预先运行并展示报告、地图、轨迹与 `runtime.log`，不在台上等待
整场探索。当前基线报告：

```text
logs/acceptance/unknown_world_slam_nav/
└── 20260720T031306Z-1114546-814430c3/
    ├── unknown_world_map.yaml / .pgm
    ├── unknown_world_slam_e2e_report.json
    └── runtime.log
```

## 2. 时间安排

| 时间 | 内容 | 展示 |
| --- | --- | --- |
| 0:00–1:00 | 问题与结果 | Gazebo/RViz、双证据表 |
| 1:00–2:30 | 总体架构 | typed 主链图 |
| 2:30–4:00 | 在线/离线语音 | VAD、ASR commit、会话/NLU |
| 4:00–5:30 | C++ 安全执行 | Guard、Scheduler、Action、BT/pluginlib |
| 5:30–8:00 | unknown-world 建图 | frontier、恢复扫描、严格完成契约 |
| 8:00–10:45 | 存图、定位、目标准入 | generation、候选、ComputePath、原子 ledger |
| 10:45–12:15 | 路径证据与时钟 | producer/evaluator、SYSTEM_TIME 与 `/clock` |
| 12:15–13:30 | 回环与动态障碍 | GTSAM、预测 costmap、replan |
| 13:30–15:00 | 报告、边界、总结 | PASS 指标与未完成边界 |

## 3. 可直接照讲的主线

### 3.1 项目定位（0:00–1:00）

> 这不是 ASR 加 `/cmd_vel` 的脚本。在线和离线 Agent 共用 typed ROS 2 控制面；LLM 只能生成候选，
> C++ Guard、Scheduler、Action 和执行器拥有最终控制权。机器人在 unknown-world 任务中只读取在线
> 传感器和本次地图，静态真值只进入独立 evaluator。

先展示 Gazebo/RViz，再告诉听众：真人语音和自主建图是两份独立证据，接下来沿一条代码链解释。

### 3.2 Typed 语音控制链（1:00–5:30）

```text
AudioFrontendNode → online/offline ASR
→ AgentApplicationRuntime → AgentControlPlane → CommandNLU
→ RobotCommand → ActionGuardNode → ActionScheduler
→ ExecuteRobotCommand Action → CommandBehaviorTree → RobotExecutor
```

依次打开：

1. `src/embodied_agent_interfaces/msg/RobotCommand.msg` 和
   `src/embodied_agent_interfaces/action/ExecuteRobotCommand.action`：字段、feedback、cancel、result 都由
   schema 约束，控制面没有旧 JSON 协议。
2. `src/embodied_agent_cpp/src/audio_frontend_node.cpp:AudioFrontendNode`：VAD endpoint；在线/离线节点
   共用 `src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py:AsrEndpointRuntime`；节点侧
   `_commit_asr_endpoint()` 说明尾静音与 commit delay 为什么能保护数字和量词。
3. `agent_control_plane.py:accept_transcript()/enqueue_command()` 与
   `command_nlu.py:CommandNLU.parse()`：唤醒会话、filler/duplicate、batch、FIFO、短命令补全；低置信度
   才回退 LLM。
4. `action_guard_node.cpp:ActionGuardNode::on_candidate()`、`action_scheduler.cpp`：白名单、限幅、TTL、
   command_id 和急停抢占。
5. `command_behavior_tree.cpp:CommandBehaviorTree::tick()`、`robot_executor.hpp:RobotExecutor`：BT 只负责编排，
   pluginlib 后端负责 Gazebo 或 Nav2。

一句总结：不确定的语音/LLM 与确定的安全执行用 typed seam 隔开，旧 result 不能释放新命令，stop 会
取消 active goal、清队列并归零。

自主任务还有一条窄接口：`AgentActionGateway` 只接收开始/取消 mission intent，再交给任务事务层；它
不把 ASR 文本直接变成导航坐标。报告只在事后记录地图来源，不参与机器人在线决策。

### 3.3 Frontier 建图与严格完成（5:30–8:00）

打开：

- `mission_executor.py:AutomaticMissionExecutor.run()`：建图→存图→定位→三点导航事务；
- `frontier_monitor.py:FrontierExplorationMonitor._unknown_world_reason()`；
- `mission_executor.py:decide_epoch_recovery()`；
- `unknown_world_evidence.py:evaluate_frontier_completion()`。

讲法：SLAM Toolbox 负责地图/位姿，Explore Lite 负责 free/unknown 边界目标，Nav2 负责到达。unknown
profile 禁止 bootstrap、places、固定目标和静态图。timeout、地图平台期和 all-blacklisted 都不是完成。
只接受：

```text
no_frontiers + no_reachable_frontiers
frontier_attempts_exhausted_recoverable + frontier_attempts_exhausted_no_map_gain
```

两组都要求 available/active/blacklisted=0，accepted goal 全部有终态。第二组来自最后一次 360° 实时
扫描后地图没有足够增益，不是把“尝试耗尽”偷换成“环境没有 frontier”。

再展示本轮修复边界：`0.33m` 是整条 frontier 可逃逸连通域净空；Burger profile 的 `0.40m` 是
`1.5+0.4<3.0m` 激光 raytrace 内的观测容差，上游通用默认仍为 `0.30m`。进入容差前必须复核最新地图
仍安全；累计逃逸位移 `>=2×0.15m` 只打断“连续静止”熔断，失败 approach 仍保留。Nav2 真正生效的
`0.10m/30s` progress 参数写进 session YAML，不能只存在于临时 launch 文件。

### 3.4 保存图、AMCL 与实时目标准入（8:00–10:45）

打开 `showcase_session_node.py:start_navigation()`，指向中文注释：mapping 停止后递增
`_navigation_input_generation`，清空旧 map/pose，只接受同代保存图和 AMCL pose。然后沿以下调用讲：

```text
rank_mapped_goal_candidates()                 # 过量、确定性候选
→ select_mapped_navigation_goals()
→ _admit_goal_candidates()
→ _request_preflight_path() / ComputePath     # 实时 Nav2 costmap 准入
→ producer known-free 路径检查
→ NavigationGoalLedger.plan(3)                # 完整批次一次登记
→ run_navigation_goal() 再次 preflight
→ NavigateToPose + runtime /plan
```

参数是 `goal_clearance_m=0.40`、`minimum_goal_separation_m=1.50`、
`GridBased.allow_unknown=false`。采样器只做本次 known-free 连通域和目标净空，不复制 Nav2 inflation；
实时 planner 才拥有最终可达性。

ComputePath 的 `204/206/208` 只淘汰尚未登记的候选；TF、start、planner、timeout、cancel 是系统性 fatal。
候选必须凑齐 3 点后才 `ledger.plan()`。NavigateToPose 接受后若失败，必须保留失败，不能换点制造 3/3。
执行前再次 preflight，运行中 `/plan` 继续保护 admission/execution 之间的 TOCTOU 窗口。

### 3.5 双侧路径证据与跨时钟关联（10:45–12:15）

打开：

- `mapped_goal_sampler.py:inspect_path_occupancy()`：生产侧按半栅格加密路径；
- `sampled_goal_tracker.py:SampledGoalPlanTracker`：typed goal 与 `/plan` 关联；
- `unknown_world_evidence.py:evaluate_sampled_navigation()`：evaluator 从保存图独立重算。

关键坑：orchestrator typed 生命周期使用 `SYSTEM_TIME`，Nav2 在 `use_sim_time=true` 时 Path 使用 Gazebo
`/clock`，两者不能按数值比较。`_timestamps_share_clock_domain()` 只在同域比较 timestamp；跨域依赖
mission generation、唯一 active goal、map endpoint 和有界乱序缓存。指出函数旁的中文注释，说明
为什么不能把 admission plan 或上一目标迟到 replan 绑给当前目标。

producer 与 evaluator 都要求 unknown/occupied/map-outside 为 0，并核对 goal 坐标、typed lifecycle、
Nav2 status/error 和最小间距；这是“防危险”和“防自证”两种不同职责。

### 3.6 后端优化与动态障碍（12:15–13:30）

打开 `src/embodied_slam` 的 GTSAM pose graph、LiDAR loop verifier/gate，以及
`src/embodied_navigation` 的 tracker/predictor/costmap layer。

讲法：回环是候选召回→局部子图几何验证→一致性门控→鲁棒核/可切换约束→全局优化，不是相似就
加边。当前新回环默认 shadow，不能把证据完整说成真实场地精度发布。动态障碍先关联 track，再估计
速度，把未来占用写进 costmap；Nav2 产生新路径，退出事务还必须清 track/cost 和验证零速。

这里要区分两类验收证据：`slam-nav-e2e` 的确定性动态障碍阶段会注入 `crossing_cart`，用于稳定复现
重规划；真人语音演示不自动注入该障碍，因此不能用现场看到一次绕行替代确定性门禁，也不能把门禁
结果说成真人语音 session。

## 4. 展示 PASS 报告（13:30–14:30）

打开 fresh session `20260720T031306Z-1114546-814430c3` 的 schema v4 JSON，按以下顺序展示：

| 项目 | 实测 | 门槛 |
| --- | ---: | ---: |
| reachable free coverage | `99.67%` | `≥90%` |
| 四区域 coverage | 最低 office `97.76%` | 各 `≥85%` |
| reachable unknown | `0.33%` | `≤10%` |
| obstacle boundary recall / false-free | `80.93% / 0.21%` | `≥60% / ≤5%` |
| frontier Action | `20 accepted / 20 terminal`，剩余计数全 0 | 完整终态 |
| AMCL/Gazebo position error P95 | `0.154 m`（246 对齐样本） | `≤0.25 m`；至少 20 样本 |
| 动态采样导航 | `3/3` 成功；最小间距 `5.570 m` | `3/3`；`≥1.50 m` |
| 路径栅格 | producer/evaluator unknown、occupied、outside 均 0 | 全为 0 |
| 动态避障 | 净空 `0.0245→1.021 m`，28 个 unique plan | 必须 replan |
| 最终速度 | fresh `/cmd_vel=0` | 必须为 0 |

地图 YAML/PGM 带时间和 SHA256，报告同时检查 fresh session map，避免拿旧地图通过。

## 5. 收尾与边界（14:30–15:00）

> 项目价值不是堆叠 ASR、SLAM 和 Nav2 名词，而是明确每层所有权：LLM 只提议，C++ 安全层决定可否
> 执行，SLAM/Explore/Nav2 各做一件事，producer 和 evaluator 分别防危险与防自证。当前 Gazebo
> unknown-world 全链路已有可审计 PASS；真人语音 known-world 需要在现场环境单独复验。真实硬件、
> 真实场地长期漂移、真实动态感知和回环在线 commit 仍是明确边界。

## 6. 高频追问代码锚点

| 追问 | 回答入口 |
| --- | --- |
| 如何证明没有场景先验 | `unknown_world_contract.py:validate_unknown_world_mission()` |
| 为什么地图不是旧的 | `start_navigation()` generation + report provenance/hash |
| 目标怎么选、为什么可达 | `rank_mapped_goal_candidates()` → `_admit_goal_candidates()` |
| Nav2 失败会不会偷偷换点 | `run_navigation_goal()`；只有未登记候选的 204/206/208 可跳过 |
| 如何证明路径没穿 unknown | `inspect_path_occupancy()` + `evaluate_sampled_navigation()` |
| frontier 怎么算完成 | `_unknown_world_reason()` + `evaluate_frontier_completion()` |
| `/plan` 如何防串台 | `SampledGoalPlanTracker` 的 generation/endpoint/clock-domain 关联 |
| 回环怎么防误检 | loop verifier/gate + switchable constraint + shadow/commit |
| 动态障碍如何影响规划 | tracker → predictor → PredictedObstacleLayer → Nav2 replan |
| 为什么最终一定停车 | Action cancel/cleanup + fresh `/cmd_vel` evaluator gate |

## 7. 现场检查清单

- 演示前清理旧 ROS/Gazebo 进程，确认当前 overlay、麦克风和磁盘空间。
- 真人语音 known-world 必须现场跑出自己的结果，不借用 unknown-world PASS 报告。
- unknown-world 报告只展示对应 session 的地图、轨迹、JSON 和日志，不跨 session 拼证据。
- 不临场修改门槛；失败时展示原始 error、typed state 和报告 checks。
- 结束后确认 fresh `/cmd_vel` 为零，并执行清理脚本。
