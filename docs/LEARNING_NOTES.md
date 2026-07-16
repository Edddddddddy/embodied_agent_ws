# 技术学习笔记：从语音 Agent 到自动建图导航

本文按稳定技术主题组织，不记录容易过期的开发流水账。每个主题统一说明功能、真实代码位置、
上下游调用、设计原因、替代方案、失败边界和测试。运行方法和 PASS 标准以
[测试与验收手册](TESTING_AND_ACCEPTANCE.md) 为准。
精确的文件、函数和 topic/action 上下游见
[语音到仿真代码走读](VOICE_TO_SIMULATION_CODE_WALKTHROUGH.md)。

```text
麦克风 / PCM
→ AEC、VAD、endpoint
→ 在线或离线 ASR
→ 会话、NLU/LLM、队列和用户上下文
→ typed RobotCommand
→ C++ ActionGuard、ActionScheduler
→ ExecuteRobotCommand Action
→ BehaviorTree.CPP、pluginlib Executor
→ Gazebo / SLAM / AMCL / Nav2
```

自动建图导航还包含应用生命周期控制路径：

```text
“开始自动巡检建图”
→ SLAM 会话状态机
→ Explore Lite frontier + Nav2
→ map saver
→ 保存地图 + AMCL/Nav2
→ Agent typed 语义导航与巡检
```

文件或接口存在只能证明结构完成，不能替代运行证据。以下所有结论都要区分单元测试、mock、
Gazebo、真实麦克风和公开 bag。

## 1. WSL 工作区、colcon overlay 与可重复激活

### 【功能】

把 ROS 2 Jazzy、Python 虚拟环境、当前工作区的 colcon 产物和 `.env` 组合为同一运行环境。
显式 `WORKSPACE` 让主仓库与 Git worktree 不会误用彼此的安装树或配置。

### 【关键文件/类/函数】

- `scripts/bootstrap.sh`：安装依赖、创建 `.venv`、安装 requirements、执行 colcon build。
- `scripts/activate.sh`：依次 source ROS、venv、当前 overlay 和 `.env`。
- `scripts/ros_dds_env.sh`：统一 DDS transport 环境变量。
- `package.xml`、`CMakeLists.txt`、`setup.py`：ament/colcon 包发现与安装契约。

### 【上游 → 处理 → 下游】

```text
/opt/ros/jazzy/setup.bash（underlay）
→ .venv + PYTHONPATH
→ $WORKSPACE/install/setup.bash（overlay）
→ .env
→ ros2 run / ros2 launch / pytest / acceptance_test.sh
```

### 【为什么这样设计】

ROS 2 生成的 Python console script 常使用系统 Python shebang，而模型依赖安装在 venv。
激活脚本显式暴露 venv site-packages，避免“当前 shell 能 import，ros2 run 却不能 import”。
`WORKSPACE` 作为唯一根目录，也比在每个脚本中猜 `pwd` 更适合 worktree。

### 【与替代方案区别】

- 只 source `/opt/ros/jazzy`：看不到项目接口和节点。
- 只激活 venv：ament index 找不到 ROS package。
- 全局 pip 安装：短期简单，容易污染系统 ROS Python ABI。
- 容器：隔离更强，但 WSLg 麦克风和 Gazebo GUI 接线成本更高。

### 【失败/安全边界】

`install/setup.bash` 不存在表示尚未构建；worktree 未设置 `WORKSPACE` 可能加载主工作区旧产物。
不要把删除整个 `build/install/log` 作为第一反应，应先确认路径、overlay 和包版本。

### 【对应测试】

```bash
export WORKSPACE="$PWD"
source scripts/activate.sh
bash scripts/acceptance_test.sh core
```

## 2. C++ ActionGuard 与 ActionScheduler

### 【功能】

在模型与执行器间建立可信边界：Guard 校验和限幅候选动作；Scheduler 管理单 active goal、FIFO、
优先取消、失败清队列、watchdog 和 command ID 结果关联。

### 【关键文件/类/函数】

- `src/embodied_agent_cpp/src/action_guard_node.cpp`：`ActionGuardNode::on_candidate()`。
- `src/embodied_agent_cpp/src/action_validator.cpp`：`ActionValidator::validate()`。
- `src/embodied_agent_cpp/src/action_scheduler.cpp`：`ActionScheduler::enqueue()`、`complete()`、
  `clear_all()`、`dispatch_next()`。
- `src/embodied_agent_cpp/src/typed_action_bridge_node.cpp`：`on_command()`、`process_events()`、
  `dispatch_goal()`、`request_cancel()`。
- `src/embodied_agent_cpp/src/typed_action_demo_client.cpp`：独立 `rclcpp_action` client 示例，展示
  goal、feedback、result、cancel 和 timeout 的标准调用方式。
- `src/embodied_agent_cpp/include/embodied_agent_cpp/guarded_command_outbox.hpp`：`GuardedCommandOutbox`。

### 【上游 → 处理 → 下游】

```text
/agent/action_candidate → ActionGuardNode::on_candidate()
→ ActionValidator::validate() → /robot/action_command_typed 或 rejection
→ TypedActionBridgeNode::on_command() → ActionScheduler::enqueue()
→ ExecuteRobotCommand goal/cancel → typed feedback/result/diagnostics
```

### 【为什么这样设计】

模型层变化快且可能幻觉，控制 policy 需要稳定、可单测、低延迟。Guard 回答“能否执行”，Scheduler
回答“何时执行、取消谁、结果属于谁”。outbox 只覆盖 discovery 短窗口，并用容量和 TTL 防旧命令回放。

### 【与替代方案区别】

LLM 直接 `/cmd_vel` 没有 schema、限幅和取消；Python 调度开发快，但 C++ 更贴近 ROS 2 执行生命周期；
把校验与调度塞进一个节点会让纯策略和并发状态难以独立测试。

### 【失败/安全边界】

Guard 不能修复“左”被识别成“右”这种合法但错误的语义，只保证参数边界。deactivate/cleanup 必须
取消 active goal、清 pending、发布终态并停车。priority 明确区分用户急停和计划 STOP。

### 【对应测试】

```bash
colcon test --packages-select embodied_agent_cpp --event-handlers console_direct+
bash scripts/acceptance_test.sh cpp-action-client
bash scripts/acceptance_test.sh gazebo
```

## 3. BehaviorTree.CPP、pluginlib 与 Gazebo 执行层

### 【功能】

Action server 接收长动作，BehaviorTree 依次做校验、安全检查、执行和确认；pluginlib 在 Mock、
Gazebo、Nav2 Executor 间切换，上游接口保持不变。

### 【关键文件/类/函数】

- `src/embodied_simulation/src/simulation_control_node.cpp`：`handle_goal()`、`control_tick()`、
  `update_active_action()`、`finish_active_action()`。
- `src/embodied_simulation/src/command_behavior_tree.cpp`：`CommandBehaviorTree::start()`、`tick()`、
  `cancel()`；`ValidateCommandNode`、`CheckSafetyNode`、`ExecuteCommandNode`、`ConfirmResultNode`。
- `src/embodied_simulation/include/embodied_simulation/robot_executor.hpp`：`RobotExecutor`。
- `src/embodied_simulation/src/gazebo_robot_executor.cpp`：`GazeboRobotExecutor::execute()`、`step()`。
- `src/embodied_simulation/src/simulation_controller.cpp`：`SimulationController::update_scan()`、`step()`。
- `src/embodied_simulation/src/simulation_ros_io.cpp`：`publish_velocity()`、`publish_zero_velocity()`。

### 【上游 → 处理 → 下游】

```text
ExecuteRobotCommand goal → SimulationControlNode::handle_goal()
→ ActiveActionRuntime + CommandBehaviorTree::tick()
→ RobotExecutor::execute()/step() → GazeboRobotExecutor
→ /cmd_vel → Gazebo /odom + /scan → Action result
```

### 【为什么这样设计】

Action server 管外部协议，BT 表达业务顺序，Executor 隐藏后端，Controller 封装速度和雷达安全。
mock 验证状态机、Gazebo 验证物理运动、Nav2 验证规划控制，无需复制 Action/Lifecycle 逻辑。

### 【与替代方案区别】

switch/case 在小动作域简单，扩展取消和终态后难维护；Nav2 BT 适合一次导航，不适合销毁整套 mapping
graph；每个后端独立节点隔离强，却会重复 Action server、诊断和超时逻辑。

### 【失败/安全边界】

雷达无效、紧急障碍、取消、超时或插件异常都必须发布零速度。`/cmd_vel` 有数据不等于成功，还需
odom、Action result 和最终停车。mock PASS 不代表 Gazebo 时钟、TF 和物理插件正常。

### 【对应测试】

```bash
colcon test --packages-select embodied_simulation --event-handlers console_direct+
bash scripts/acceptance_test.sh gazebo
```

## 4. 自动 frontier 建图、map saver、AMCL 与 Nav2

### 【功能】

一句语音启动自动任务：Explore Lite 从 OccupancyGrid 选择 frontier，Nav2 自主到达；覆盖完成后
保存地图、关闭 mapping stage、启动 AMCL/Nav2，再通过 Agent typed 链执行入口和多地点巡检。

### 【关键文件/类/函数】

- `config/frontier_exploration.repos`、`scripts/setup_frontier_exploration.sh`：固定第三方 commit。
- `src/embodied_simulation/config/frontier_exploration.yaml`、`showcase_workplace_mission.yaml`。
- `src/embodied_slam_tools/embodied_slam_tools/showcase_session.py`：`parse_session_command()`、
  `parse_mapping_bootstrap_route()`、`ShowcaseSessionStateMachine.validate()`、`transition()`。
- `src/embodied_slam_tools/embodied_slam_tools/mission_executor.py`：
  `AutomaticMissionExecutor.run()` 封装自动探索、存图、定位切换和语义巡航事务。
- `src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py`：
  `wait_for_frontier()`、`save_map()`、`start_navigation()`、
  `run_agent_action()`。
- `src/embodied_slam_tools/embodied_slam_tools/mapping_evidence.py`：
  `MappingEvidenceTracker` 原子收集地图增长、真实里程、LiDAR 首帧和探索结束状态。
- `src/embodied_slam_tools/embodied_slam_tools/stage_process_manager.py`：
  `StageProcessManager` 隔离 launch、Explore Lite、map saver 与进程树回收副作用。
- `src/embodied_simulation/src/nav2_places.cpp`、`nav2_robot_executor.cpp`：
  `Nav2Places::to_pose_stamped()`、`send_navigate_goal()`、`send_follow_goal()`。
- `src/embodied_simulation/src/nav2_result_policy.cpp`：
  `evaluate_follow_waypoints_result()` 区分 Action 协议成功与多航点业务完整成功。
- `src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py`：`RewrittenYaml` 生成项目级
  Nav2 参数副本，为 WSL/Gazebo 调整进度半径和时间窗，不修改系统安装文件。

### 【上游 → 处理 → 下游】

```text
_on_asr_final() → parse_session_command() → _enqueue() → _worker_loop()
→ _execute_request() → AutomaticMissionExecutor.run()
→ run_agent_action(move/turn bootstrap route)
→ StageProcessManager.start_explorer() → /explore/status + /map
→ wait_for_frontier() → save_map() → start_navigation()
→ run_agent_action() → Agent/Guard/Action → NavigateToPose/FollowWaypoints
→ evaluate_follow_waypoints_result() → success / blocked + missed detail
```

### 【为什么这样设计】

Explore Lite 选择未知边界，Nav2 负责安全到达，编排器负责结束条件、存图、进程切换和业务任务。
领域状态机不依赖 ROS/subprocess，worker 执行副作用，readiness generation 防止误用旧 stage 快照。
出生点位于充电角时，先用 move/turn-only bootstrap route 进入中央门洞；这些动作仍经过 Agent、
ActionGuard 和 ROS 2 Action，且不包含未知地图上的语义目标。它解决初始 frontier 被外墙边缘主导，
但不会替代后续自主选点。
Nav2 默认 0.5 m/10 s 的进度检查对低实时率 WSL 仿真过于临界，因此 launch 将它改为
0.10 m/30 s；这仍能发现真正卡死，又不会把低速有效运动误判为无进展。
结束原因分为 `no_frontiers`、`coverage_plateau` 和 `time_budget_coverage`；最后一种只在
300 秒预算到期且已知/占用栅格均达标时成立，因此“任务有时间上限”不等于“超时也算成功”。

### 【与替代方案区别】

固定速度路线确定性高但不是自主探索；SLAM Toolbox/Cartographer 估计地图和位姿，不决定探索目标；
已知 waypoint 适合巡检，未知地图需要 frontier 或其他 exploration policy。

### 【失败/安全边界】

frontier goal 由 Explore Lite 直接发 Nav2，不经过 LLM ActionGuard，但仍受 costmap、planner、controller
约束。地图质量达标且长期不增长可按平台期结束。取消/异常必须停止 explorer 和机器人，并回到
MAPPING 或 NAVIGATING 可恢复状态。`FollowWaypoints` 只有 `ResultCode=SUCCEEDED`、`error_code=0`
且 `missed_waypoints=0` 才是业务成功；部分到达不得进入 COMPLETED。

### 【对应测试】

```bash
bash scripts/acceptance_test.sh slam-nav-showcase-stage
HEADLESS=false USE_RVIZ=true bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

## 5. SLAM 后端、GTSAM 与 LiDAR 回环发布门

### 【功能】

把 scan matcher 的节点/约束交给 Ceres/GTSAM 优化 SE(2) 位姿图；旁路生成 Top-K 回环候选，经过
局部子图几何验证和时序/序列门控后决定 shadow 或 commit，并用 ATE/RPE/precision/recall 评价。

### 【关键文件/类/函数】

- `src/embodied_slam/src/gtsam_pose_graph.cpp`：`GtsamPoseGraphOptimizer::optimize()`、
  `SwitchableBetweenFactor`。
- `src/embodied_slam/src/gtsam_scan_solver.cpp`：`GtsamScanSolver::AddNode()`、`AddConstraint()`、`Compute()`。
- `src/embodied_slam/src/lidar_loop_runtime.cpp`：`LiveLidarLoopDetector::ingest()`。
- `src/embodied_slam/src/lidar_submap_builder.cpp`：短时局部子图。
- `src/embodied_slam/src/lidar_loop_verifier.cpp`：`LiveLidarLoopVerifier::verify()`。
- `src/embodied_slam/src/lidar_loop_constraint_gate.cpp`：`LidarLoopConstraintGate::evaluate()`、`reset()`。
- `src/embodied_slam/src/instrumented_async_slam_toolbox_node.cpp`：Karto commit Adapter。

### 【上游 → 处理 → 下游】

```text
LaserScan + odom/TF → Karto nodes/constraints → GtsamScanSolver
→ GtsamPoseGraphOptimizer.optimize() → 优化轨迹/地图/ATE/RPE
/scan + /slam/odom → Top-K candidate → submap ICP verification
→ LidarLoopConstraintGate::evaluate() → shadow decision 或 Karto commit result
```

### 【为什么这样设计】

前端决定约束，后端联合优化位姿，二者不能混为一谈。纯 optimizer 与 Karto Adapter 分离便于固定图
A/B。描述子负责召回，ICP/重叠负责几何，多帧/多假设减少偶然匹配；policy 与 commit 分离可先在
真实数据 shadow 运行，错误边不会拉坏地图。

### 【与替代方案区别】

GTSAM 的 factor graph 适合鲁棒核/可切换约束；Ceres 是通用最小二乘和 slam_toolbox 生态对照；g2o
轻量经典但不是本项目主实现。单一最高分候选直接写图风险高，地图“看起来直”也不能替代指标。

### 【失败/安全边界】

后端不能创造正确回环；错误边会扭曲全图。真实比较必须固定 bag、前端、时间关联、配置和哈希。
当前新 LiDAR 回环保持 `commit_enabled=false`；PASS 可能只证明证据完整，不表示精度达到发布门槛。

### 【对应测试】

```bash
colcon test --packages-select embodied_slam --event-handlers console_direct+
bash scripts/acceptance_test.sh slam-evaluation-stage
bash scripts/acceptance_test.sh openloris-replay-stage
```

## 6. 动态障碍关联、运动模型与 Nav2 预测层

### 【功能】

把检测关联为稳定 track，比较 current-only、CV、Kalman、IMM，再把未来占用写入 Nav2 costmap，
使规划器在障碍进入机器人路线前提前重规划。

### 【关键文件/类/函数】

- `src/embodied_navigation/src/dynamic_obstacle_tracker.cpp`：`DynamicObstacleTracker::update()`、
  `CurrentOnlyEstimator`、`SmoothedConstantVelocityEstimator`、`KalmanEstimator`、`ImmEstimator`。
- `src/embodied_navigation/src/gated_observation_assignment.cpp`：全局门限关联。
- `src/embodied_navigation/src/constant_velocity_predictor.cpp`：`predict_constant_velocity()`。
- `src/embodied_navigation/src/predicted_obstacle_layer.cpp`：`on_obstacles()`、`updateBounds()`、
  `updateCosts()`、`reset()`。

### 【上游 → 处理 → 下游】

```text
PoseArray / typed detections → global gated assignment
→ DynamicObstacleTracker::update() → track state/velocity/covariance/TTL
→ predict_constant_velocity() → DynamicObstacleArray
→ PredictedObstacleLayer → future lethal cost → Nav2 replan
```

### 【为什么这样设计】

数据关联和运动估计是两个问题：先保持身份，再比较模型。tracker 用统一 `update()` 隐藏四种模型，
costmap plugin 只消费统一轨迹；旧 bounds 被保留用于清除过期占用，避免留下永久“鬼墙”。

### 【与替代方案区别】

greedy nearest 简单但多目标会抢同一 track；全局门限关联更稳定。CV 轻量可解释，Kalman/IMM 能表达
噪声和模式切换但参数更复杂。只在控制器急停能防撞，不能提前改变全局路径。

### 【失败/安全边界】

重型导航证据使用确定性合成感知输入，不等于真实检测器或人群模型。二维 costmap 压平时间维会较
保守；协方差未标定时不能包装成真实传感器收益。track TTL、旧 bounds 清理和最终急停都要验收。

### 【对应测试】

```bash
colcon test --packages-select embodied_navigation --event-handlers console_direct+
bash scripts/acceptance_test.sh dynamic-obstacle-stage
bash scripts/acceptance_test.sh dynamic-obstacle-navigation
```

## 7. 可观测性、分层测试与事实证据

### 【功能】

把音频、ASR、session、NLU、队列、Action、BT、仿真、SLAM 和 Nav2 变成可观察事件；以单测、
repository contract、stage、Gazebo、真人麦克风和公开 bag 分层证明。

### 【关键文件/类/函数】

- `scripts/continuous_voice_monitor.py`：`ContinuousVoiceMonitor`、`MonitorStats.format_summary()`、
  `format_advice()`、`AsrNluSampleRecorder`。
- `src/embodied_agent_middleware/src/system_readiness_node.cpp`：`publish_readiness()`。
- `src/embodied_simulation/src/simulation_ros_io.cpp`：ACK、BT status、health、diagnostics。
- `scripts/showcase_release_gate.py`：`GateCommand`、`_run_command()`、`main()`。
- `scripts/generate_architecture_facts.py`：`build_facts()`、`render_markdown()`。
- `scripts/acceptance_test.sh`、`tests/repository/`、各包 `test/`、`tests/integration/`。

### 【上游 → 处理 → 下游】

```text
typed events + TF/map/odom/cmd_vel
→ monitor/readiness/integration probes
→ command ID、状态迁移和指标聚合
→ terminal + JSON/Markdown report
→ release/demo/robotics gate → CI 或人工结论
```

### 【为什么这样设计】

链路跨音频、网络、Python、DDS、C++、Gazebo 和 Nav2，仅看最终“不动”无法定位。typed 状态能区分
没录音、ASR 拒绝、队列满、Guard 拒绝、Action 失败、Nav2 aborted 和 TF 缺失；失败报告也要保留。

### 【与替代方案区别】

只看 INFO 日志难做断言；只做单测不能证明 DDS/TF/Gazebo；只做重型 E2E 反馈慢。测试金字塔让
单测定位、stage 验接口、重型/真人/公开数据提供最终证据。

### 【失败/安全边界】

mock、fixture、Gazebo、真人麦克风、公开 bag 和实体硬件不能互相替代。报告 PASS 可能只表示契约
完整，算法发布仍要检查 metric/release decision。历史报告必须绑定日期、commit、配置和输入来源。

### 【对应测试】

```bash
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh robotics-gate
bash scripts/acceptance_test.sh slam-nav-showcase-stage
```

## 8. DDS、QoS、Lifecycle 与系统 readiness

### 【功能】

为命令、事件、状态、传感器、音频和诊断定义不同 DDS 语义；用 Lifecycle 管理 provider 和 Action
资源；用组件健康聚合判断整套 launch 是否真正可以接收任务。

### 【关键文件/类/函数】

- `src/embodied_agent_middleware/include/embodied_agent_middleware/qos_profiles.hpp`：
  `command_qos()`、`event_qos()`、`state_qos()`、`sensor_qos()`、`audio_qos()`。
- `src/embodied_agent_core/embodied_agent_core/ros_qos.py`：Python 同名 QoS。
- `src/embodied_agent_core/embodied_agent_core/agent_lifecycle_runtime.py`：
  `AgentLifecycleRuntime.activate()`、`deactivate()`、`release()`。
- `src/embodied_agent_middleware/include/embodied_agent_middleware/component_health_registry.hpp`：
  `ComponentHealthRegistry`。
- `src/embodied_agent_middleware/src/system_readiness_node.cpp`：`SystemReadinessNode`。

### 【上游 → 处理 → 下游】

```text
节点 configure/activate/deactivate
→ ComponentHealth + typed state/event
→ ComponentHealthRegistry
→ SystemReadinessNode::publish_readiness()
→ launch、会话编排器和验收探针决定是否放行任务
```

命令/事件使用 reliable + volatile；当前状态使用 reliable + transient-local；scan/PCM 使用
best-effort + volatile；diagnostics 使用 reliable。命令不使用 transient-local，避免节点重启后
重放旧动作。

### 【为什么这样设计】

状态晚加入订阅者需要拿到最新快照，所以用 transient-local；高频 PCM/scan 更重视低延迟，消费
落后时应丢旧帧。Lifecycle 让“进程存在”和“资源已准备”成为不同状态，停机可以先停车、取消
goal，再释放线程和 provider。

### 【与替代方案区别】

- 所有 topic 都 reliable：音频积压会放大端到端延迟。
- 所有 topic 都 best-effort：控制结果丢失会破坏队列关联。
- 普通 Node：启动简单，无法表达资源 configure/activate/deactivate 边界。
- 只检查 PID：进程活着不代表模型、Action server 或 TF 已就绪。

### 【失败/安全边界】

DDS discovery 有时间窗；已校验命令只由有容量和 TTL 的 outbox 暂存。Fast DDS SHM 锁和过大的
`ROS_DOMAIN_ID` 属于环境故障，不应通过放宽安全规则规避。readiness 快照不是永久有效心跳，
重型冷启动要使用合适的 stale window。

### 【对应测试】

```bash
bash scripts/acceptance_test.sh core
bash scripts/acceptance_test.sh robotics-gate
```

## 9. 音频 AEC、VAD、endpoint 与 ASR commit

### 【功能】

从 WSL/PulseAudio 获取 PCM，执行回声抑制和音频指标统计，检测说话开始/结束，并在 endpoint 后
延迟提交 ASR，减少“左转九十度”被截成“左转”的尾部漏识别。

### 【关键文件/类/函数】

- `src/embodied_agent_cpp/src/audio_frontend_node.cpp`：`AudioFrontendNode`。
- `src/embodied_agent_cpp/src/audio_processing.cpp`：`EnergyVad::is_speech()`、
  `SpeechEndpointDetector::update()`、`SilenceDetector::update()`、`NlmsEchoCanceller::process()`。
- `src/embodied_voice_frontend/embodied_voice_frontend/silero_vad_sidecar.py`：
  `StreamingVadEndpoint`。
- `src/embodied_voice_frontend/embodied_voice_frontend/webrtc_vad_node.py`：`WebRtcVadNode`、`_on_audio()`。
- `src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py`：`AsrEndpointRuntime`。

### 【上游 → 处理 → 下游】

```text
PulseAudio source + TTS reference
→ NlmsEchoCanceller / AudioFrontendNode
→ /audio/clean_pcm + frontend metrics
→ energy、Silero 或 WebRTC VAD
→ /audio/speech_started + /audio/speech_ended
→ AsrEndpointRuntime 延迟 commit
→ online WebSocket commit 或 offline ASR queue event
```

### 【为什么这样设计】

音频采集、声学 endpoint 和语义理解属于不同变化方向。C++ 前端保证低开销 PCM 处理与统一指标，
成熟 VAD 作为 sidecar 可替换；Agent 只响应稳定 endpoint。commit delay 用少量延迟换取数字、量词
等尾部完整性。

### 【与替代方案区别】

- 固定静音 0.4 秒：响应快，短停顿和尾音容易截断。
- 只用能量 VAD：部署简单，对噪声和远场说话适应较弱。
- Silero/WebRTC VAD：泛化更好，增加模型/依赖和采样格式要求。
- 完整 WebRTC AEC：更成熟，WSL 音频路由与参考时钟接入更复杂。

### 【失败/安全边界】

NLMS 是轻量回声抑制，不等于生产级双讲 AEC。VAD provider 缺失时必须明确降级，不能静默声称
使用成熟模型。RMS/peak 很低时先检查 source 和输入增益，不要把阈值降到环境底噪以下。

### 【对应测试】

```bash
pytest -q src/embodied_voice_frontend/test
colcon test --packages-select embodied_agent_cpp --event-handlers console_direct+
```

## 10. 在线 ASR、LLM 与流式 TTS Adapter

### 【功能】

在线节点把音频发送给 Qwen/DashScope 兼容 ASR，把上下文交给 OpenAI-compatible LLM，增量解析
speech/action 标签，并把可说句子交给 Qwen TTS；provider 差异不侵入共享控制面。

### 【关键文件/类/函数】

- `src/embodied_online_agent/embodied_online_agent/online_agent_node.py`：`OnlineAgentNode`、
  `_on_clean_audio()`、`_commit_asr_endpoint()`、`_on_asr_final()`、`_accept_transcript()`、`_run_turn()`。
- `src/embodied_online_agent/embodied_online_agent/providers/qwen_asr.py`：
  `QwenRealtimeAsr.start()`、`push_audio()`、`commit()`。
- `src/embodied_online_agent/embodied_online_agent/providers/openai_compatible_llm.py`：
  `OpenAiCompatibleLlm.stream()`。
- `src/embodied_online_agent/embodied_online_agent/providers/qwen_tts.py`：
  `QwenRealtimeTts.synthesize()`。
- `src/embodied_agent_core/embodied_agent_core/streaming_turn.py`：`StreamingTurnRuntime.feed()`、`finish()`。

### 【上游 → 处理 → 下游】

```text
/audio/clean_pcm
→ QwenRealtimeAsr.push_audio()/commit()
→ OnlineAgentNode._on_asr_final()
→ AgentApplicationRuntime.accept_transcript()
→ OpenAiCompatibleLlm.stream()
→ StreamingTurnRuntime.feed()/finish()
→ QwenRealtimeTts.synthesize() + typed action candidate
```

### 【为什么这样设计】

节点负责 ROS/Lifecycle 接线，provider 负责云协议，`StreamingTurnRuntime` 负责标签协议、分句和动作
选择。API SDK 更新不会迫使会话、记忆、队列和 ActionGuard 一起变化。token 和可说句子分开，便于
分别观察首 token、首音频和完整动作。

### 【与替代方案区别】

- 单次 HTTP：实现简单，无法提供 ASR partial、LLM token 和低等待 TTS。
- 节点直接解析 SDK event：接入快，测试必须连接真实云端。
- provider Adapter：多一层接口，换来 mock、重试和 API 替换能力。
- LLM 自由输出动作：泛化强、协议不稳定，仍需 parser 和 Guard。

### 【失败/安全边界】

网络、配额、key、限流和模型升级都可能失败。在线延迟必须由本次报告证明，不能永久引用历史
`<1s`。LLM 输出只是候选，任何动作仍要通过 typed transport 和 C++ ActionGuard。

### 【对应测试】

```bash
bash scripts/acceptance_test.sh continuous-online
pytest -q src/embodied_agent_core/test/test_streaming_turn.py
```

## 11. 离线 Sherpa ASR、llama.cpp 与 TTS 双缓冲

### 【功能】

在无云环境使用 Sherpa-ONNX ZipFormer、llama.cpp GGUF 和 Sherpa-TTS/SummerTTS；通过文本/音频
双缓冲让 LLM 与 TTS 并行，并记录首 token、decode 和首音频指标。

### 【关键文件/类/函数】

- `src/embodied_offline_agent/embodied_offline_agent/offline_agent_node.py`：`OfflineAgentNode`、
  `_enqueue_asr()`、`_run_asr()`、`_on_asr_final()`、`_run_turn()`。
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_asr.py`：
  `SherpaZipformerAsr.push_audio()`、`commit()`、`_decode_ready()`。
- `src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py`：
  `LlamaCppLlm.warmup()`、`stream()`。
- `src/embodied_offline_agent/embodied_offline_agent/providers/sherpa_tts.py`：`SherpaVitsTts.synthesize()`。
- `src/embodied_offline_agent/embodied_offline_agent/double_buffer.py`：`DoubleBuffer.put()`、`get()`、`abort()`。
- `src/embodied_offline_agent/embodied_offline_agent/pseudo_streaming_tts.py`：
  `PseudoStreamingTtsPipeline._tts_worker()`、`_audio_worker()`。
- `src/embodied_offline_agent/embodied_offline_agent/offline_turn_runtime.py`：
  `OfflineStreamingTurnRuntime.run()`。

### 【上游 → 处理 → 下游】

```text
clean PCM + endpoint
→ SherpaZipformerAsr
→ AgentApplicationRuntime / OfflineStreamingTurnRuntime
→ LlamaCppLlm.stream()
→ 文本 DoubleBuffer → TTS worker
→ 音频 DoubleBuffer → audio worker
→ /audio/tts_pcm
```

### 【为什么这样设计】

ASR、LLM 和 TTS 的计算特征不同。有界双缓冲隔离生成速度与播放速度，避免 TTS 阻塞 LLM token；
abort/close 让取消和停机有明确语义。provider 测试可以使用 fake runtime，不必下载大模型。

### 【与替代方案区别】

- LLM 完成后再 TTS：简单，首音频等待长。
- 原生流式 TTS：延迟更低，模型必须支持增量状态。
- 分句伪流式：适配现有模型，第一句仍需整句合成。
- SummerTTS 常驻 C++ service：减少加载开销，未命中缓存仍可能较慢。

### 【失败/安全边界】

模型、tokenizer、GGUF 架构与运行时版本必须匹配。队列满时不能无限占用内存；取消必须 abort 两个
buffer。Q8/LoRA 合成 holdout 不等于真实麦克风准确率，伪流式不能表述为原生流式 TTS。

### 【对应测试】

```bash
pytest -q src/embodied_offline_agent/test
bash scripts/acceptance_test.sh continuous-offline
```

## 12. 连续会话、多命令 NLU 与执行队列

### 【功能】

支持一次唤醒后连续输入，把一句话解析成多个动作，在前一动作执行时继续接收命令，并用优先 stop、
TTL、duplicate/filler 过滤和 command ID 保证长期控制不乱序。

### 【关键文件/类/函数】

- `src/embodied_agent_core/embodied_agent_core/continuous_voice.py`：
  `ContinuousVoiceSession`、`ContinuousCommandQueue`。
- `src/embodied_agent_core/embodied_agent_core/agent_control_plane.py`：
  `AgentControlPlane.accept_transcript()`、`enqueue_command()`。
- `src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py`：
  `AgentApplicationRuntime.accept_transcript()`、`run_queued_turn()`、私有 `_run_preparsed_turn()`。
- `src/embodied_agent_core/embodied_agent_core/command_nlu.py`：`CommandNLU.parse()`。
- `src/embodied_agent_core/embodied_agent_core/command_completion.py`、`command_fallback.py`。
- `src/embodied_agent_core/embodied_agent_core/agent_execution_runtime.py`：`AgentExecutionRuntime`。

### 【上游 → 处理 → 下游】

```text
ASR final
→ AgentApplicationRuntime.accept_transcript()
→ AgentControlPlane.accept_transcript()
→ wake/session、normalization、completion、NLU batch
→ enqueue_command() → run_queued_turn()
→ _run_preparsed_turn() 或 provider turn
→ publish_actions()
→ 等待同 command_id 的 Action result
```

### 【为什么这样设计】

输入与执行速度不同，busy 时丢命令会像卡住，并行执行又会产生运动冲突。单 worker + FIFO 保证
顺序，batch ID 表示同一句多动作，command/request ID 关联结果。高置信度 NLU 直接执行，低置信度
才交给 LLM，兼顾延迟和泛化。

### 【与替代方案区别】

- 按“然后/再”切字符串：难处理否定、自然表达和组合动作。
- 小型字符模型 + 槽位规则：轻量可解释，领域外泛化有限。
- 全部 function calling：能力强，在线成本和延迟高，离线小模型协议不稳。
- 并行动作：吞吐高，移动、转向和导航无法安全并发。

### 【失败/安全边界】

`停下/急停` 不排队，必须取消 active goal 并清 pending；计划 STOP 不能误当用户急停。过期命令
不得很久后执行，旧 result 不得唤醒下一 command ID。否定句和疑问句不应猜测执行。

### 【对应测试】

```bash
pytest -q src/embodied_agent_core/test/test_continuous_voice.py \
  src/embodied_agent_core/test/test_command_nlu.py
bash scripts/acceptance_test.sh continuous-multi-command
```

## 13. 声纹身份、用户记忆与行为偏好

### 【功能】

把声纹身份、录入、用户偏好和交互历史组合为 turn 级不可变上下文，使“默认慢一点”等习惯确定性
影响动作参数，同时防止低置信度身份污染个人画像。

### 【关键文件/类/函数】

- `src/embodied_voice_frontend/embodied_voice_frontend/speaker_identity_node.py`：
  `SpeakerIdentityNode`、`classify_speaker_scores()`、`_on_speech_ended()`。
- `src/embodied_agent_core/embodied_agent_core/user_context_runtime.py`：
  `UserContextRuntime.snapshot()`、`handle_command()`、`record_interaction()`。
- `src/embodied_agent_core/embodied_agent_core/memory_command_service.py`：`MemoryCommandService.handle()`。
- `src/embodied_agent_core/embodied_agent_core/user_memory.py`：
  `UserMemoryStore.profile()`、`set_preference()`、`record_interaction()`、`prompt_summary()`。
- `src/embodied_agent_core/embodied_agent_core/user_preferences.py`：动作偏好应用逻辑。

### 【上游 → 处理 → 下游】

```text
clean PCM + speech_ended
→ SpeakerIdentityNode
→ /agent/speaker_identity
→ UserContextRuntime.update_identity()
→ 命令入队时 snapshot()
→ system prompt + deterministic preferences
→ 已选择动作 → record_interaction()
```

### 【为什么这样设计】

声纹会在 turn 期间异步更新。入队时冻结 `UserContextSnapshot`，可保证 prompt、偏好和 interaction
属于同一用户。记忆只记录最终被策略接受的动作，不让被安全层拦截的模型输出污染画像。

### 【与替代方案区别】

- 全局共享记忆：简单，多人串写。
- 每次读取当前身份：实时，同一 turn 可能前后换用户。
- 向量数据库：适合开放知识召回；固定控制偏好用结构化 profile 更可审计。
- 云声纹：可能更准，引入隐私、网络和费用。

### 【失败/安全边界】

low-confidence/unknown 不可写个人 profile；多人、多房间 FAR/FRR 尚未充分评测。记忆不是安全授权，
用户应能清理数据，偏好修改后的动作仍必须经过 ActionGuard。

### 【对应测试】

```bash
pytest -q src/embodied_agent_core/test/test_user_context_runtime.py \
  src/embodied_agent_core/test/test_user_memory.py \
  src/embodied_voice_frontend/test/test_speaker_identity_node.py
```

## 14. Typed msg/srv/action 作为跨进程契约

### 【功能】

用 rosidl schema 表达动作、队列、执行、VAD/KWS、健康、SLAM 回环和动态障碍，使 Python/C++ 节点
在编译与 discovery 阶段共享字段、枚举和时间戳语义。

### 【关键文件/类/函数】

- `src/embodied_agent_interfaces/msg/RobotCommand.msg`、`RobotCommandFeedback.msg`、
  `RobotCommandResult.msg`。
- `src/embodied_agent_interfaces/msg/CommandContext.msg`、`CommandQueueEvent.msg`、
  `CommandExecutionEvent.msg`、`NluParseEvent.msg`。
- `src/embodied_agent_interfaces/action/ExecuteRobotCommand.action`、`ManageSlamSession.action`。
- `src/embodied_agent_interfaces/srv/SynthesizeSpeech.srv`。
- `src/embodied_agent_core/embodied_agent_core/ros_action_transport.py`：`action_command_to_message()`。
- `src/embodied_agent_core/embodied_agent_core/ros_event_transport.py`：
  `queue_event_to_message()`、`execution_event_to_message()`、`nlu_parse_to_message()`。

### 【上游 → 处理 → 下游】

```text
Python 领域 ActionCommand / queue event
→ transport Adapter
→ rosidl Python/C++ message
→ DDS
→ C++ Guard、Scheduler、simulation 或 monitor
→ typed feedback/result
→ Adapter 恢复报告字典
```

### 【为什么这样设计】

跨进程协议不能靠每个节点自行拼字典。typed message 固定字段、数组和枚举，C++/Python 由同一 IDL
生成。报告层仍可序列化 JSON，不让文件格式反向污染实时控制协议。

### 【与替代方案区别】

- `String + JSON`：原型快，无编译期 schema，字段漂移运行时才发现。
- protobuf/gRPC：跨语言强，不是 ROS graph 原生工具链。
- ROS service：适合短请求响应，不适合可取消长动作。
- ROS Action：有 goal/feedback/result/cancel，适合移动、导航和会话切换。

### 【失败/安全边界】

typed 只保证结构，不保证语义安全；速度、地点和任务状态仍要校验。修改 IDL 后必须重建所有依赖
overlay，避免 Python 读取旧生成类型。JSON 只保留在报告、数据集和硬件协议边界。

### 【对应测试】

```bash
pytest -q src/embodied_agent_core/test/test_ros_action_transport.py \
  src/embodied_agent_core/test/test_ros_event_transport.py
bash scripts/acceptance_test.sh core
```

## 15. 复盘主线

数据面：在线/离线语音进入同一 `AgentApplicationRuntime`，经过 NLU/LLM、typed message、C++
Guard/Scheduler、Action、BT/pluginlib，最终驱动 Gazebo 或 Nav2。

应用生命周期：`SessionOrchestratorNode` 把 frontier 探索、地图质量判断、map saver、进程切换、
AMCL/Nav2 readiness 和语义巡检组织成可取消、有状态、有失败回退的任务。

算法与证据：GTSAM/Ceres 负责后端图优化，LiDAR 回环保持 shadow/commit 分层，动态障碍通过关联、
运动估计和 costmap 预测接入 Nav2；每项结论都对应测试与事实边界。

最重要的工程原则是：模型输出不是控制权，接口存在不是运行证据，单场景正向结果不是泛化结论。
