# 15 分钟项目汇报与现场演示稿

本文是一份可以直接照着讲、照着演示、照着打开代码的中文讲稿。主演示目标是：

配套总图使用 [架构图与时序图](FINAL_ARCHITECTURE_DIAGRAMS.md)，精确代码链使用
[语音到仿真代码走读](VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md)；后者就是汇报时“从语音输入到仿真执行的代码走读地图”。

```text
一句“小智，开始自动巡检建图”
→ frontier 自主探索
→ 自动保存地图
→ 切换 AMCL/Nav2
→ 自动执行入口、厨房、办公室语义巡检
```

在线/离线语音 Agent 是这条任务的共同上游；现场默认使用离线模式降低网络变量，在线模式用于
补充证明 provider 可以替换。项目验收对象是 Gazebo/TurtleBot3，不把 UART/SPI mock 说成实体硬件。

## 1. 演示前 30 分钟准备

### 1.1 环境和固定依赖

```bash
cd /home/ubuntu/embodied_agent_ws
export WORKSPACE="$PWD"
source scripts/activate.sh
bash scripts/setup_frontier_exploration.sh
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh slam-nav-showcase-stage
```

如果 `source scripts/activate.sh` 失败，先确认 `/opt/ros/jazzy/setup.bash`、`.venv/bin/activate` 和
`install/setup.bash` 存在；缺少构建产物时先运行 `colcon build --symlink-install`。

### 1.2 麦克风和 Agent

离线主演示前先跑一次：

```bash
bash scripts/acceptance_test.sh continuous-offline
```

在线补充演示前先确认 `.env` 中 API key 有效，再运行：

```bash
bash scripts/acceptance_test.sh continuous-online
```

这两条只验证“语音 → Agent → 仿真动作”前置链路；自动建图导航仍要单独启动主演示。

### 1.3 主演示启动

```bash
HEADLESS=false USE_RVIZ=true \
  bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

等待终端显示系统就绪、Gazebo 中出现 TurtleBot3、RViz 中出现 LaserScan 后再开始汇报。保留一个
备用终端，用于查看状态或在麦克风失效时做文本注入。

### 1.4 必备保底材料

- 保留最近一次成功运行的 `logs/showcase/autonomous_runtime/automatic_mission_report.json`。
- 保留自动探索、地图生成和 Nav2 巡检三个阶段的截图或短视频。
- 保留项目生成的 YAML/PGM 地图；它只能证明上一次运行，不冒充本次现场结果。
- 现场超时后切换到证据讲解，不在观众面前反复重启大型进程。

## 2. 15 分钟总时间线

| 时间 | 页面 | 现场动作 | 核心结论 |
| --- | --- | --- | --- |
| 0:00–1:00 | 1. 问题与结果 | 说出唯一启动句 | 一句话触发完整自动任务，不是固定路线回放 |
| 1:00–2:15 | 2. 总体架构 | 保持 RViz/Gazebo 可见 | 在线/离线 Agent 共用 typed ROS 2 控制面 |
| 2:15–3:30 | 3. 语音前端 | 展示 ASR final 和 session 日志 | endpoint、commit delay 和 provider Adapter |
| 3:30–5:00 | 4. Agent 应用层 | 打开共享 Python runtime | 会话、NLU、多命令、队列和私有预解析路径 |
| 5:00–6:30 | 5. C++ 安全执行 | 打开 ActionGuard/Scheduler/BT | LLM 不直接控制 `/cmd_vel` |
| 6:30–8:15 | 6. 自动任务编排 | 地图在后台持续扩张 | 显式状态机、worker、取消和失败回退 |
| 8:15–9:45 | 7. Frontier 探索 | 指出 frontier 与规划轨迹 | Explore Lite 选目标，Nav2 负责规划控制避障 |
| 9:45–11:15 | 8. 存图和导航切换 | 观察地图产物与状态迁移 | 保存地图、进程切换、AMCL、语义导航 |
| 11:15–12:45 | 9. SLAM 工程深度 | 展示后端和回环代码 | 区分前端、回环验证、图优化和事实边界 |
| 12:45–14:00 | 10. 验收证据 | 展示 result、odom 和最终零速 | 报告字段与运行状态共同证明闭环 |
| 14:00–15:00 | 11. 总结与边界 | 回到系统总图 | 能讲清实现、取舍、不足和下一步 |

## 3. 分页讲稿

### 页面 1：问题、目标和一句话启动（0:00–1:00）

讲什么：

> 我做的不是单独的语音识别 Demo，而是一个在线/离线双 Agent 的 ROS 2 机器人系统。用户只说
> 一句“开始自动巡检建图”，系统就自主探索未知区域、保存地图、切换定位导航并完成语义巡检。
> 上游允许 ASR 和 LLM 有不确定性，下游通过强类型接口、C++ 安全网关和 Nav2 约束机器人行为。

现场动作：立即说：

```text
小智，开始自动巡检建图
```

展示代码：

- `src/embodied_agent_interfaces/action/ManageSlamSession.action`：高层会话 Action。
- `src/embodied_agent_interfaces/msg/SlamSessionState.msg`：可观测阶段快照。
- `src/embodied_slam_tools/embodied_slam_tools/showcase_session.py`：`SessionCommand`、`SessionPhase`。

关键技术：语音系统意图、typed Action、状态机、frontier exploration、SLAM、AMCL 和 Nav2。

失败时保底：若麦克风没有产生 ASR final，在备用终端注入同一系统意图：

```bash
ros2 topic pub --once /agent/asr_final std_msgs/msg/String \
  "{data: '小智，开始自动巡检建图'}"
```

同时说明：文本注入只证明 ASR 之后的链路，不算真人语音证据。

### 页面 2：总体架构和两条控制路径（1:00–2:15）

讲什么：

> 在线和离线只在 provider 数据面不同，连续会话、动作协议、安全网关和执行层是共用的。普通语义
> 动作走 Agent → ActionGuard → ROS 2 Action；frontier 探索目标由成熟探索器直接交给 Nav2，
> 不经过 LLM，但仍受 costmap、planner、controller 和恢复行为约束。

展示代码：

- `src/embodied_agent_interfaces/msg/RobotCommand.msg`。
- `src/embodied_agent_interfaces/action/ExecuteRobotCommand.action`。
- `src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py`：`AgentApplicationRuntime`。
- `src/embodied_agent_cpp/src/action_guard_node.cpp`：`ActionGuardNode`。
- `src/embodied_simulation/src/simulation_control_node.cpp`：`SimulationControlNode`。

关键技术：自定义 msg/action、Lifecycle、QoS、进程编排和控制路径隔离。强调“探索路径不经过
ActionGuard”是明确设计边界，而不是含糊声称所有 Nav2 goal 都经过 LLM 安全网关。

失败时保底：Gazebo 窗口被遮挡时切换到
[架构与模块说明](ARCHITECTURE_AND_KNOWLEDGE.md) 的数据流图，演示进程继续在后台运行。

### 页面 3：在线/离线语音输入如何汇合（2:15–3:30）

讲什么：

> C++ 音频前端发布 clean PCM、端点事件和音量指标。在线节点把音频送到云 ASR；离线节点把事件
> 送入本地 Sherpa worker。两条路径最终都调用共享应用层的 `accept_transcript()`，因此替换
> provider 不会复制会话、NLU 和安全逻辑。离线推理使用 llama.cpp，低延迟默认 TTS 是
> Sherpa-TTS；SummerTTS 作为命令行与常驻 C++ service Adapter 展示组件化接入，不把缓存命中
> 冒充首次真实合成延迟。

展示代码与调用关系：

```text
audio_frontend_node.cpp: AudioFrontendNode
→ /audio/clean_pcm + /audio/speech_started + /audio/speech_ended

OnlineAgentNode._on_clean_audio()
→ _on_speech_ended()
→ _commit_asr_endpoint()
→ _on_asr_final()
→ _accept_transcript()

OfflineAgentNode._on_audio()
→ _on_speech_ended()
→ _commit_asr_endpoint()
→ _on_asr_endpoint()
→ _enqueue_asr() / _run_asr()
→ _on_asr_final()
→ _accept_transcript()
```

关键文件：

- `src/embodied_agent_cpp/src/audio_frontend_node.cpp`。
- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`：`OnlineAgentNode`。
- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`：`OfflineAgentNode`。

关键技术：energy/Silero/WebRTC VAD Adapter、endpoint、ASR commit delay、Lifecycle 和 provider seam。

失败时保底：在线 API 波动就切离线；本地模型未加载就用在线。两者都失败时展示已运行的
`continuous-multi-command` 证据，但明确它不是麦克风证据。

### 页面 4：Agent 应用层、NLU 和队列（3:30–5:00）

讲什么：

> ASR final 不会直接进 LLM。应用层先做唤醒、重复 final 和 filler 过滤，再进行确定性 NLU、短命令
> 补全和多命令拆批。动作执行中收到的新命令进入 FIFO；急停走优先路径。online/offline 复用同一
> 应用用例，而不是互相复制 Node callback。

展示真实调用关系：

```text
AgentApplicationRuntime.accept_transcript()
→ AgentControlPlane.accept_transcript()
→ AgentApplicationRuntime.enqueue_continuous_command()
→ AgentControlPlane.enqueue_command()
→ AgentApplicationRuntime.run_queued_turn()
→ AgentApplicationRuntime._run_preparsed_turn()   # 私有确定性动作路径
  或 provider turn callback                       # 低置信度/对话路径
→ AgentApplicationRuntime.publish_actions()
```

展示代码：

- `src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py`：
  `accept_transcript()`、`run_queued_turn()`、私有 `_run_preparsed_turn()`、`publish_actions()`。
- `src/embodied_agent_core/embodied_agent_core/agent_control_plane.py`：
  `accept_transcript()`、`enqueue_command()`。
- `src/embodied_agent_core/embodied_agent_core/command_nlu.py`：`CommandNLU.parse()`。
- `src/embodied_agent_core/embodied_agent_core/streaming_turn.py`：`StreamingTurnRuntime.feed()`、
  `finish()`、`_select_actions()`。

关键技术：应用服务、依赖注入、不可变 turn/user context、FIFO/TTL、batch/command ID、确定性路径与
LLM fallback 分离。私有 `_run_preparsed_turn()` 是正确符号，不讲成不存在的公共 API。

失败时保底：执行：

```bash
bash scripts/acceptance_test.sh continuous-multi-command
```

用确定性回归展示一句话多动作、顺序执行和 result 关联。

### 页面 5：C++ ActionGuard、调度和仿真执行（5:00–6:30）

讲什么：

> Python 适合模型和语音编排，但可信动作的校验、调度和长动作生命周期放在 C++。ActionGuard 对
> 动作白名单、速度、角速度和时长做校验；ActionScheduler 保证同一时刻一个 active goal，普通命令
> FIFO，急停取消；ROS 2 Action 提供 feedback、cancel 和 result。

展示调用关系：

```text
/agent/action_candidate
→ ActionGuardNode::on_candidate()
→ /robot/action_command_typed
→ TypedActionBridgeNode::on_command()
→ ActionScheduler::enqueue()
→ TypedActionBridgeNode::process_events()
→ ExecuteRobotCommand Action goal
→ SimulationControlNode::handle_goal()
→ SimulationControlNode::control_tick()
→ CommandBehaviorTree::tick()
→ RobotExecutor plugin
→ SimulationControlNode::finish_active_action()
```

展示代码：

- `src/embodied_agent_cpp/src/action_guard_node.cpp`：`ActionGuardNode::on_candidate()`。
- `src/embodied_agent_cpp/src/action_scheduler.cpp`：`enqueue()`、`complete()`、`dispatch_next()`。
- `src/embodied_agent_cpp/src/typed_action_bridge_node.cpp`：`on_command()`、`process_events()`。
- `src/embodied_agent_cpp/src/typed_action_demo_client.cpp`：独立 C++ Action client，现场可展示
  goal、feedback、cancel、result 和 timeout 的标准 `rclcpp_action` 用法。
- `src/embodied_simulation/src/command_behavior_tree.cpp`：`CommandBehaviorTree::tick()`。
- `src/embodied_simulation/src/simulation_control_node.cpp`：`handle_goal()`、`control_tick()`、
  `finish_active_action()`。

关键技术：rclcpp Lifecycle、rclcpp_action、BehaviorTree.CPP、pluginlib、watchdog 和最终零速。

失败时保底：先运行 `bash scripts/acceptance_test.sh cpp-action-client` 展示纯 C++ Action 生命周期，
再运行 `bash scripts/acceptance_test.sh gazebo`；或展示最近一次 Action feedback/result
报告；不能用“topic 发布成功”代替机器人运动和 Action 成功。

### 页面 6：自动任务状态机和进程生命周期（6:30–8:15）

讲什么：

> 自动任务不是一段 shell 串行命令。领域状态机负责合法迁移，ROS 节点负责 typed Action、状态发布
> 和队列，独立 worker 执行建图、存图和进程切换这些有副作用的操作。这样 ROS callback 不会被
> `map_saver_cli` 阻塞，失败也能回到 MAPPING 或 NAVIGATING 重试。

展示完整自动调用链：

```text
SessionOrchestratorNode._on_asr_final()
→ showcase_session.parse_session_command()
→ SessionOrchestratorNode._enqueue()
→ SessionOrchestratorNode._worker_loop()
→ SessionOrchestratorNode._execute_request()
→ SessionOrchestratorNode._run_automatic_mission()
→ parse_mapping_bootstrap_route()
→ SessionOrchestratorNode._run_agent_text_action(move/turn)
→ StageProcessManager.start_explorer()
→ SessionOrchestratorNode._wait_for_frontier_completion()
→ SessionOrchestratorNode._save_map()
→ SessionOrchestratorNode._start_navigation()
→ SessionOrchestratorNode._wait_for_navigation_action_servers()
→ SessionOrchestratorNode._run_agent_text_action()
```

展示代码：

- `src/embodied_slam_tools/embodied_slam_tools/showcase_session.py`：
  `parse_session_command()`、`parse_mapping_bootstrap_route()`、
  `ShowcaseSessionStateMachine.validate()`、`transition()`。
- `src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py`：
  `SessionOrchestratorNode`。
- `src/embodied_slam_tools/embodied_slam_tools/stage_process_manager.py`：
  `StageProcessManager`。

关键技术：显式有限状态机、有界队列、worker、readiness generation、进程组关闭、幂等/重复意图过滤、
可取消 Action 和失败回退。

失败时保底：说“取消自动任务”演示取消路径；若自动进程异常，展示状态回到 MAPPING/NAVIGATING，
而不是伪造 MISSION_COMPLETED。

### 页面 7：Frontier 自动探索方法（8:15–9:45）

讲什么：

> 我没有把固定路线包装成自动建图，也没有声称自己重写了成熟探索器。系统固定了 Explore Lite
> 版本，把 frontier 选点接入 Nav2，并在项目层实现任务状态、地图质量门、覆盖平台期判定、取消和
> 后续导航编排。Explore Lite 负责“下一目标在哪里”，Nav2 负责“如何安全到达”。

展示代码和配置：

- `config/frontier_exploration.repos`：固定第三方来源和 commit。
- `scripts/setup_frontier_exploration.sh`：可重复安装与构建。
- `src/embodied_simulation/config/frontier_exploration.yaml`：frontier 和规划参数。
- `showcase_session_node.py`：`_on_map()`、`_on_explore_status()`、
  `_wait_for_frontier_completion()`、`_cancel_automatic_motion()`。

关键技术：OccupancyGrid frontier、信息增益/路径代价、Nav2 `NavigateToPose`、已知/占用栅格阈值、
地图增长平台期和不可达 frontier 处理。`_on_map()` 统计 known/occupied cells；
`_wait_for_frontier_completion()` 同时检查 ExploreStatus、最短运行时间、地图质量和增长停滞。

失败时保底：若探索仍在继续，直接展示正在增长的 `/map` 和 Nav2 轨迹；这已经证明自动探索在运行。
若 9:45 尚未切换阶段，则切到预先保存的报告/视频继续讲，不为赶时间手工伪造完成状态。

### 页面 8：保存地图、AMCL 和语义巡检（9:45–11:15）

讲什么：

> 探索完成后，编排器先保存 YAML/PGM，再有序停止 mapping graph，启动保存地图、AMCL 和 Nav2。
> 它等待新的 readiness generation 和导航 Action server，避免把上个阶段的 transient-local 状态
> 误认成新阶段就绪。随后用文本重新进入 Agent 主链，最终发出 typed `navigate_to` 和
> `follow_waypoints`。

展示调用关系：

```text
StageProcessManager.save_map()
→ SessionOrchestratorNode._save_map()
→ SessionOrchestratorNode._start_navigation()
→ _wait_for_new_ready() / _wait_for_navigation_action_servers()
→ _run_agent_text_action("去入口", "navigate_to")
→ _run_agent_text_action("依次去厨房、办公室", "follow_waypoints")
→ Agent / ActionGuard / ExecuteRobotCommand
→ Nav2RobotExecutor.send_navigate_goal()
→ Nav2RobotExecutor.send_follow_goal()
→ evaluate_follow_waypoints_result()
```

展示代码：

- `src/embodied_simulation/src/nav2_places.cpp`：`load_nav2_places_yaml()`、
  `Nav2Places::to_pose_stamped()`。
- `src/embodied_simulation/src/nav2_robot_executor.cpp`：`Nav2RobotExecutor`、
  `send_navigate_goal()`、`send_follow_goal()`。
- `src/embodied_simulation/src/nav2_result_policy.cpp`：`evaluate_follow_waypoints_result()`；
  区分 ROS Action 协议成功与所有巡检点实际完成。
- `src/embodied_simulation/config/showcase_places.yaml` 与 `showcase_mapping_places.yaml`：
  世界坐标和建图坐标下的语义地点。

关键技术：map saver、AMCL `map→odom`、Lifecycle readiness、语义地点解析、NavigateToPose、
FollowWaypoints、command ID 与 result 关联。

失败时诊断：地图保存失败应保留 mapping 供重试；Nav2 未 ACTIVE 时展示明确超时/失败状态。
导航演示必须加载本次保存地图，不使用预生成静态地图作为保底证据。

### 页面 9：SLAM 后端、回环和动态障碍深度（11:15–12:45）

讲什么：

> 启动 SLAM Toolbox 不等于自己实现了 SLAM。我的工程工作包括可重复漂移模型、Ceres/GTSAM 后端
> A/B、候选回环、局部子图几何验证、多帧一致性和发布门。前端负责产生约束，后端在位姿图上优化；
> 错回环会把整张图拉坏，所以当前新 LiDAR 回环默认 shadow-only，未满足多序列精度门槛前不写图。

展示代码：

- `src/embodied_slam/src/gtsam_pose_graph.cpp`：`GtsamPoseGraphOptimizer`。
- `src/embodied_slam/src/gtsam_scan_solver.cpp`：`GtsamScanSolver` Adapter。
- `src/embodied_slam/src/lidar_loop_runtime.cpp`：在线候选生命周期。
- `src/embodied_slam/src/lidar_loop_verifier.cpp`：局部子图几何验证。
- `src/embodied_slam/src/lidar_loop_constraint_gate.cpp`：时序/序列门控和 commit 决策。
- `src/embodied_navigation/src/dynamic_obstacle_tracker.cpp`：`DynamicObstacleTracker::update()`。
- `src/embodied_navigation/src/predicted_obstacle_layer.cpp`：Nav2 costmap plugin。

关键技术：scan matching、pose graph、Prior/Between factors、鲁棒核、可切换约束、回环
precision/recall、动态目标关联、CV/Kalman/IMM 和预测占据。

失败时保底：不现场运行大型 OpenLORIS 数据集；展示已经保存的、带来源哈希的报告，并明确
“证据完整”和“算法指标达标”不是同一件事。

### 页面 10：如何证明全链路完成（12:45–14:00）

讲什么：

> 我把证据分成单元测试、stage gate、Gazebo 重型运行、真实麦克风和公开 bag。主演示通过不能只看
> 小车动了，还要看到状态机、地图质量、地图文件、定位 TF、Nav2 Action result 和最终零速度。

现场检查：

- `/slam/session_state` 经过 `AUTOMATIC_MAPPING`、`AUTOMATIC_NAVIGATING`、`MISSION_COMPLETED`。
- 在线 `/map` 达到任务配置中的已知/占用栅格门槛。
- YAML/PGM 地图真实存在并能被新进程加载。
- AMCL 发布 `map→odom`，Nav2 server 进入 ACTIVE。
- `navigate_to` 和 `follow_waypoints` 都返回成功，而不是只发布 goal。
- `/odom` 与规划动作一致，结束后 `/cmd_vel` 线速度和角速度均为零。
- 报告写入 `logs/showcase/autonomous_runtime/automatic_mission_report.json`。

展示命令：

```bash
bash scripts/acceptance_test.sh robotics-gate
```

关键技术：分层证据、typed 状态、Action 终态、TF、map metadata、odom 和 fail-closed 报告。

失败时保底：本次若只完成探索但未完成导航，现场结论就是“探索阶段通过、完整任务未通过”；随后展示
上一次完整报告作为历史证据，不能把两次运行拼成一次 PASS。

### 页面 11：总结、边界和下一步（14:00–15:00）

讲什么：

> 第一，我把在线/离线语音 Agent 接到同一套 ROS 2 强类型控制链，而不是让 LLM 直接发速度。
> 第二，我用 C++ ActionGuard、Scheduler、Action、BT 和 pluginlib 建立可取消、可观察的安全执行层。
> 第三，我把 frontier 探索、SLAM 存图、AMCL 和 Nav2 语义巡检编排成一句话自动任务，并用地图、
> TF、Action result、odom 和零速度证明闭环。

事实边界：

- 这是 Gazebo/TurtleBot3 仿真闭环，不是实体底盘验收。
- Explore Lite 是成熟第三方 frontier 实现；项目贡献是版本固定、任务编排、质量门、取消和后续链路。
- 在线/离线真实语音准确率取决于当前 API、模型、麦克风和环境，mock 不替代真人证据。
- GTSAM/Ceres 和回环有可复现实验，但新 LiDAR 回环仍默认 shadow-only。
- 动态障碍以合成感知输入和仿真规划控制为主，不宣称完成人群真实感知。

下一步：优先做真实 rosbag/真实环境的跨场景建图漂移评估、自动任务恢复策略和更稳定的现场语音，
而不是继续堆更多无证据的动作。

展示内容：回到 [架构与模块说明](ARCHITECTURE_AND_KNOWLEDGE.md) 的总图，同时打开
[运行时证据状态](RUNTIME_EVIDENCE_STATUS.md)，把“系统结构”和“当前证据”并列呈现，不再引入新代码。

关键技术：端到端接口分层、provider 可替换性、fail-closed 安全、自动任务编排和分层可复现证据。

失败时保底：无论现场哪个重型模块失败，都回到架构图和对应失败状态，讲清“在哪一层失败、为什么、
已有哪条回归测试、下一步如何定位”。这比重复重启或隐藏失败更能体现工程能力。

## 4. 现场演示清单

### 开始前

- [ ] 当前终端已显式 `export WORKSPACE="$PWD"` 并成功激活环境。
- [ ] `core` 和 `slam-nav-showcase-stage` 已通过。
- [ ] Explore Lite 固定版本已安装，Gazebo/RViz 没有孤儿进程干扰。
- [ ] 离线或在线前置语音链已经单独跑通。
- [ ] 主演示已经启动并显示系统就绪。
- [ ] 备用终端、历史报告、地图和截图均已准备。

### 演示中

- [ ] 只说一次“小智，开始自动巡检建图”。
- [ ] 指出 ASR final、session phase、frontier goal、路径和地图扩张。
- [ ] 讲清 Explore Lite 与项目自研编排的职责边界。
- [ ] 讲清探索路径与语义动作路径是否经过 ActionGuard 的区别。
- [ ] 观察存图、进程切换、AMCL TF 和 Nav2 Action result。
- [ ] 超时即切历史证据，不手工伪造完成状态。

### 结束后

- [ ] 确认状态为 `MISSION_COMPLETED`，否则按真实阶段报告。
- [ ] 确认地图 YAML/PGM 可加载。
- [ ] 确认导航和巡检均有成功 result。
- [ ] 确认最终 `/cmd_vel` 为零。
- [ ] 汇报时明确 Gazebo、真实麦克风、公开 bag 和实体硬件证据边界。

## 5. 常见追问的代码锚点

| 追问 | 首先打开 | 关键符号 |
| --- | --- | --- |
| 在线/离线如何复用 | `agent_application_runtime.py` | `AgentApplicationRuntime.accept_transcript()` |
| 确定性 NLU 如何绕开 LLM 延迟 | `agent_application_runtime.py` | 私有 `_run_preparsed_turn()` |
| 多命令为什么不乱序 | `agent_control_plane.py`、`continuous_voice.py` | `enqueue_command()`、FIFO/command ID |
| LLM 为什么不能直接控车 | `action_guard_node.cpp` | `ActionGuardNode::on_candidate()` |
| 急停如何抢占 | `action_scheduler.cpp` | `enqueue()`、`clear_all()`、`dispatch_next()` |
| 长动作如何反馈和取消 | `typed_action_bridge_node.cpp` | `process_events()`、`dispatch_goal()`、`request_cancel()` |
| BT 如何驱动执行器 | `command_behavior_tree.cpp` | `CommandBehaviorTree::tick()` |
| 自动任务在哪里 | `showcase_session_node.py` | `_run_automatic_mission()` |
| 如何判断探索完成 | `showcase_session_node.py` | `_on_map()`、`_wait_for_frontier_completion()` |
| 如何切换到定位导航 | `showcase_session_node.py` | `_save_map()`、`_start_navigation()` |
| 语义地点如何成为 Nav2 goal | `nav2_places.cpp`、`nav2_robot_executor.cpp` | `to_pose_stamped()`、`send_navigate_goal()` |
| 如何解释后端优化 | `gtsam_pose_graph.cpp` | `GtsamPoseGraphOptimizer` |
| 为什么回环仍不开启写图 | `lidar_loop_constraint_gate.cpp` | shadow/commit 两阶段决策 |

更完整的文件、函数和上下游接口见
[语音到仿真代码走读](VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md)；主演示的运行与验收细节见
[真实感语音 SLAM/Nav2 演示](VOICE_SLAM_NAV_SHOWCASE.md)。
