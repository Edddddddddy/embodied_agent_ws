# SLAM / Nav2 学习笔记

覆盖自动 frontier 建图、地图保存、AMCL/Nav2 切换、位姿图与回环、动态障碍预测和重规划。主演示证据以 [测试手册](../TESTING.md) 为准。

## 1. 自动 frontier 建图、map saver、AMCL 与 Nav2

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
- `src/embodied_slam_tools/embodied_slam_tools/mission_configuration.py`：
  `MissionConfiguration.load()` 把任务 YAML、默认值、bootstrap 约束和验收阈值收紧成冻结值对象；
  Node 不再逐项理解 YAML key。
- `src/embodied_slam_tools/embodied_slam_tools/frontier_monitor.py`：
  `FrontierExplorationMonitor.wait()` 统一覆盖阈值、地图平台期、Explorer 健康、超时和取消判定。
- `src/embodied_slam_tools/embodied_slam_tools/agent_action_gateway.py`：
  `AgentActionGateway.run()` 用回调代次关联候选与结果，拒绝 Agent 重启后复用 ID 的旧结果。
- `src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py`：
  作为 ROS Adapter 提供 `save_map()`、`start_navigation()`、`run_agent_action()`。
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
- `tools/acceptance/progress.py`：`AcceptanceProgress.start()`、`stage()`、`stop()`；把重型验收的
  ROS 状态压缩成可讲解的阶段和心跳，完整证据仍保存在 session 报告与 `runtime.log`。
- `tools/acceptance/slam_nav_evidence.py`：`evaluate_dynamic_navigation()`、
  `build_automatic_mission_report()`；以无 ROS 值对象集中定义动态净空、新地图、定位、完整巡航和
  最终停车的 PASS 条件。
- `tests/integration/slam_nav/test_voice_slam_session_orchestrator.py`：`SessionProbe._on_state()`、
  `run_showcase_dynamic_navigation()`、`print_report()`；从 typed topic/Action 收集真实证据并输出摘要。

### 【上游 → 处理 → 下游】

```text
_on_asr_final() → parse_session_command() → _enqueue() → _worker_loop()
→ _execute_request() → AutomaticMissionExecutor.run()
→ run_agent_action(move/turn bootstrap route)
→ StageProcessManager.start_explorer() → /explore/status + /map
→ FrontierExplorationMonitor.wait() → save_map() → start_navigation()
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
验收心跳与业务状态分离：心跳只证明探针仍存活，最终 PASS 由 `SlamNavEvidence` 根据新地图、
TF/AMCL、Action result、动态规划和零速度等硬证据统一决定，不能用“日志还在刷”代替功能成功。
配置与证据都采用冻结值对象，是为了让 ROS Node/探针成为薄 Adapter；替换 YAML 默认值或报告规则时，
只修改一个深模块并运行无 ROS 单测，而不是同时检查 launch、Node 和 1400 行集成脚本。

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

## 2. SLAM 后端、GTSAM 与 LiDAR 回环发布门

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

## 3. 动态障碍关联、运动模型与 Nav2 预测层

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
- `tools/acceptance/dynamic_route.py`：`select_replannable_route()` 把候选路线选择、失败恢复和
  `route_attempts` 审计集中为一个可单测策略。
- `tests/integration/slam_nav/test_voice_slam_session_orchestrator.py`：
  `run_showcase_dynamic_navigation()` 只接受“静态路径可达、预测代价已写入且新路径净空确有提升”
  的候选；拒绝后停放 Gazebo 障碍、等待 track TTL 并确认旧 cost 清除，再尝试下一条路线。

### 【上游 → 处理 → 下游】

```text
PoseArray / typed detections → global gated assignment
→ DynamicObstacleTracker::update() → track state/velocity/covariance/TTL
→ predict_constant_velocity() → DynamicObstacleArray
→ PredictedObstacleLayer → future lethal cost → Nav2 replan
```

### 【为什么这样设计】

数据关联和运动估计是两个问题：先保持身份，再比较模型。tracker 用统一 `update()` 隐藏四种模型，
costmap plugin 只消费统一轨迹；旧 bounds 被保留用于清除过期占用，避免留下永久“鬼墙”。验收路线
不能只看 `ComputePathToPose` 是否返回成功，还必须比较注入障碍前后的最小净空；否则规划器返回同一路径
也会被误报为“已重规划”。候选失败后的显式恢复保证下一候选不受上一条 track/cost 残留污染。

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
