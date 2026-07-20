# SLAM / Nav2 学习笔记

本笔记围绕“代码在哪里、为何这样设计、替代方案有什么差异、失败边界是什么”组织。系统总览见
[架构文档](../ARCHITECTURE.md)，执行命令和硬门槛见 [测试手册](../TESTING.md)。

## 1. Unknown-world 建图到导航事务

### 代码在哪里

- `src/embodied_slam_tools/embodied_slam_tools/mission_executor.py`
  - `UnknownWorldMissionExecutor.run()`：建图、存图、切换 AMCL/Nav2、动态三点导航的事务入口。
  - `AutomaticMissionExecutor.run()`：known-world 演示事务，不拥有 unknown-world 自主闭环。
  - `_explore_with_bounded_recovery()`、`decide_epoch_recovery()`：恢复扫描、地图增益和 epoch 预算。
- `src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py`
  - `SessionOrchestratorNode._on_asr_final()`：只把开始/取消话术转成 mission intent。
  - `start_navigation()`：停止 mapping、递增 navigation generation、清理旧 map/pose，再启动保存图定位栈。
  - `select_mapped_navigation_goals()`：等待同代 `/map` 和 AMCL pose，组织候选与实时准入。
  - `run_navigation_goal()`：执行前 preflight、NavigateToPose 和运行中路径生产证据。
  - `AgentActionGateway`：关联扫描/STOP primitive 的 Action goal/result，不拥有语音 callback 或任务阶段。
- `src/embodied_slam_tools/embodied_slam_tools/stage_process_manager.py:StageProcessManager`：launch、
  map saver 与进程树副作用。
- `src/embodied_slam_tools/embodied_slam_tools/mission_configuration.py:MissionConfiguration.load()`：把
  YAML 收紧成冻结配置，Node 不再散读 key。

主调用链：

```text
ManageSlamSession / voice intent
→ UnknownWorldMissionExecutor.run()
→ start_explorer() → FrontierExplorationMonitor.wait()
→ save_map()
→ start_navigation()：new navigation generation
→ wait_navigation_ready()
→ select_mapped_navigation_goals()
→ run_navigation_goal() × 3
→ mission_outcome + typed evidence
```

### 为何这样设计

SLAM Toolbox 估计地图和位姿，Explore Lite 决定去哪里扩图，Nav2 决定怎样安全到达；三者职责不同。
任务事务把阶段所有权集中起来，避免 mapping SLAM 与 AMCL 同时发布 `map→odom`。`start_navigation()`
中的中文注释解释了代次边界：mapping 停止后才清缓存并递增代次，防止 transient-local 旧地图和新
AMCL pose 被误拼成一次合法输入。

初始/恢复扫描与安全 STOP 走 Agent → Guard/Scheduler → BT/pluginlib primitive 链；运行时地图目标经过
orchestrator 的 known-free/ComputePath 准入后直接走 typed Nav2 Action。前者需要统一限幅和抢占，后者
需要保留 Nav2 的规划、反馈和取消语义，所以不人为合并成一条速度控制路径。

### 与替代方案区别

- 固定 waypoint/bootstrap 路线稳定，但它验证 known-world 回归，不证明未知环境自主探索。
- 一个 launch 同时启动 SLAM 与 AMCL 简单，但会产生 TF 所有权冲突。
- 固定 `sleep` 不能证明 Action server/lifecycle 真正 ready；当前用 readiness、Action server 和
  lifecycle ACTIVE 逐层确认。

### 失败边界

任一阶段 timeout、取消、保存图失败、TF/lifecycle 异常或导航失败都终止整个事务。`phase` 表示当前
ROS 栈阶段，`mission_outcome` 表示任务 SUCCEEDED/FAILED/CANCELED，二者不能混用。只有一次完整
`unknown-world-slam-e2e` session 才能组合成正式证据，分阶段 PASS 不能拼成 E2E PASS。

取消已接受或迟到接受的 Nav2 goal 必须按 `cancel → fresh typed priority STOP → Nav2 terminal` 完成；
pending goal response 也有独立 safety budget。只有整个序列可证明终态时才发布 `MISSION_CANCELED`；
response/result 超时、cancel 或 STOP 失败会触发 stage fail-safe，并以 `MISSION_FAILED` 结束。这样不会把
“已经返回给调用者”误当成“机器人已经停止”。

## 2. 运行时采样导航目标准入：从地图候选到 NavigateToPose

### 代码在哪里

- `src/embodied_slam_tools/embodied_slam_tools/mapped_goal_sampler.py`
  - `rank_mapped_goal_candidates()`：从起点所在 known-free 连通域返回过量、确定性候选。
  - `_clearance_offsets()`：按 cell 方块边缘计算净空，兼容 OccupancyGrid float32 resolution。
  - `inspect_path_occupancy()`：以半栅格步长加密整条路径。
- `src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py`
  - `_admit_goal_candidates()`：候选筛选、间距检查和完整批次原子性。
  - `_request_preflight_path()`：调用 `ComputePathToPose`，区分候选错误与系统错误。
  - `_path_occupancy_for_goal()`：把 admission 路径绑定到当次保存图快照。
- `src/embodied_slam_tools/embodied_slam_tools/mapping_evidence.py:NavigationGoalLedger`：只在完整
  3 点凑齐后 `plan()`，随后记录 REQUESTED/ACCEPTED/EXECUTING/终态与 producer 路径计数。
- `tools/acceptance/unknown_world_evidence.py:evaluate_sampled_navigation()`：evaluator 从保存图独立
  重算目标、间距、路径和 Action 终态。
- 参数：`unknown_world_slam_mission.yaml` 的 `goal_clearance_m`、`minimum_goal_separation_m`；
  `navigation_overrides.yaml` 的 `GridBased.allow_unknown: false`。数值和 PASS 门槛以
  [测试手册](../TESTING.md) 为唯一事实源。

完整链路：

```text
保存图 /map + AMCL pose（同一 navigation generation）
→ rank_mapped_goal_candidates() 生成过量候选
→ select_mapped_navigation_goals()
→ _admit_goal_candidates()
   → ComputePathToPose
   → producer inspect_path_occupancy()
→ 凑齐恰好 3 点后 NavigationGoalLedger.plan() 一次登记
→ 每点执行前再次 ComputePath preflight
→ NavigateToPose + runtime /plan
→ evaluator evaluate_sampled_navigation()
```

### 为何这样设计

地图预筛和 Nav2 准入是两层不同能力。预筛低成本证明候选来自本次 known-free 连通域并满足配置净空；
Nav2 才拥有实时 inflation、footprint 和 costmap 信息。采样器故意不复制完整 Nav2 costmap
算法，否则两份实现会随参数变化而漂移。

候选过量返回，只有实时可达点进入最终集合。候选被拒绝前不能写入 ledger；凑齐 3 点后原子
`plan()`，防止候选不足却遗留半批次 PLANNED 证据。接受集合内部再检查配置的最小间距，被拒绝
候选不会成为 spacing anchor。

执行前再次 preflight 是对 TOCTOU 的防护：admission 到 NavigateToPose 之间 costmap 仍可能变化。
NavigateToPose 接受以后发生的 failure 必须保留失败，不能临时换候选把真实失败改写成 3/3 成功。

### 与替代方案区别

- 预定义 places 泄露场景；随机坐标可能落在 unknown、孤岛或墙边。
- 只检查 goal cell 不能发现两个自由端点之间穿越 unknown/occupied；当前生产侧和 evaluator 都对
  path 加密采样。
- 一开始只选 3 个 farthest 点会因实时 costmap 拒绝而整批失败；过量候选 + admission 在“登记前”
  允许安全替换，同时保持已接受任务的失败真实性。
- 只做一次 ComputePath 存在 admission/execution 时序窗口；当前执行前复检并观察运行中 `/plan`。

### 失败边界

`ComputePathToPose` 的 `GOAL_OUTSIDE_MAP(204)`、`GOAL_OCCUPIED(206)`、`NO_VALID_PATH(208)` 只说明当前
尚未登记的 endpoint 不合适，可淘汰后继续。`UNKNOWN(200)`、planner/TF/start 错误、timeout、server
不可用和 cancel 可能影响所有候选，必须 fatal。路径含任意 unknown/occupied/map-outside 也只在 admission
阶段淘汰候选；一旦 goal 已被 NavigateToPose 接受，同类问题必须作为该 goal 失败记录。

`GridBased.allow_unknown=false` 让 planner 行为与证据契约一致；若开启，planner 可能通过机器人从未观察
的区域找到“更短路径”，既不安全也无法审计。

## 3. Frontier 完成、恢复扫描与终态证据

### 代码在哪里

- 第三方补丁与安装：`patches/m-explore-ros2/`、`scripts/setup_frontier_exploration.sh`。
- `frontier_monitor.py:FrontierExplorationMonitor._unknown_world_reason()`：读取 typed provider 状态，
  加入 idle grace、地图稳定和恢复预算。
- `mission_executor.py:decide_epoch_recovery()`：确认扫描后比较 known cell 增益。
- `mapping_evidence.py:MappingEvidenceTracker`：累计 accepted/terminal goal，而恢复重启不能抹账。
- `tools/acceptance/unknown_world_evidence.py:evaluate_frontier_completion()`：最终严格 reason pair 与计数。
- `patches/m-explore-ros2/0012-execution-clearance-contract.patch`：`0.33 m` 可逃逸连通域及临界通道测试。
- `patches/m-explore-ros2/0013-frontier-progress-recovery-contract.patch`：progress anchor、分层失败记忆、
  terminal 后熔断。
- `patches/m-explore-ros2/0014-observable-approach-escape-contract.patch`：可观测半径、最新安全复核和
  “有效逃逸打断连续静止”契约。
- `scripts/prepare_frontier_nav2_params.py:with_frontier_progress_checker()`：把 Nav2 实际使用的
  `0.10 m / 30 s` progress checker 写入本 session 参数 YAML。

### 为何这样设计

Explore Lite 的 `no_frontiers` 是 provider 对当前地图的事实；任务层的 `no_reachable_frontiers` 是在
idle grace 后对可达性和 Action 生命周期的判断。若一个 epoch 的物理 approach 都耗尽，provider 只能
发布 `frontier_attempts_exhausted_recoverable`；任务层做一次传感器驱动的 360° 扫描，等待地图稳定，
只有增益低于阈值才发布 `frontier_attempts_exhausted_no_map_gain`。

“等待地图稳定”不是一个可无限延长的定时器：soft quiet 从最后一次地图增长计算，用来吸收尾部更新；
hard budget 从调用开始计算且增长不会重置，用来限制噪声或持续增长造成的无限等待。quiet 未满足继续等，
hard deadline 到达则失败，因此 timeout 不会被伪装成收敛。

### 为什么 traversal clearance 与 observation tolerance 必须分开

`frontier_approach_clearance=0.33 m` 会侵蚀整片可通行连通域，负责拒绝 Burger 无法可靠逃出的临界盲袋；
`frontier_approach_reached_tolerance=0.40 m` 只服务 unknown-world profile，表示机器人已经进入 LiDAR
可观测半径。后者不是 Nav2 成功容差，也不能反过来用于侵蚀整条通路。数值都叫“距离”却解决不同问题：
统一取大值会切断可用门洞，统一取小值又会让控制器在膨胀区边缘反复 timeout。

进入 `0.40 m` 并不直接写 visited。`evaluateActiveFrontierGoal()` 和 `Explore::makePlan()` 会先按最新地图
复核 approach 仍为安全已知自由空间；导航期间若该栅格变成 unknown 或 occupied，安全事实优先，按
`UNSAFE_TARGET` 释放。相比“只看欧氏距离”，这避免把尚未观测或已变化的 frontier 伪装成已完成。

### 为什么不能只看“离目标更近了”

局部规划器绕开桌角时常先横移，U 型路径甚至会暂时远离目标。补丁 0013 的
`evaluateActiveFrontierGoal()` 同时维护两个进展信号：

- 到 frontier approach 的距离比历史最优值至少改善一个栅格误差量；
- 相对 `progress_anchor` 的实际位移达到 `0.15 m`。

任一成立就刷新 steady-clock watchdog，并把当前位姿设为新 anchor；横移时不会用更大的目标距离覆盖历史
最优值。相比只读 Nav2 feedback 的方案，这个判定不依赖特定 controller 的 feedback 字段，Humble/Jazzy
均可复用；相比累计轮速里程，它使用 map 位姿，不会把原地轮滑或控制抖动当成持续进展。

这里还有两层互补的 watchdog。Explore 每移动 `0.15 m` 刷新本目标进展；若一个最终 timeout 的目标从
起点累计有效逃逸至少 `2 × 0.15 m`，它会清零“连续静止 timeout”计数，因为机器人已经离开上一卡点。
但 `recordApproachFailed()` 仍保留这个落点的失败记忆，避免立即原地重试。Nav2 的
`SimpleProgressChecker` 使用更细的 `0.10 m / 30 s` 控制器门槛；生成脚本把值写入 session YAML，而不是
只依赖进程退出即消失的 launch rewrite，保证报告能复核真实运行参数。

### 为什么 timeout 只封禁 approach

`FrontierAttemptMemory.recordApproachFailed()` 记录实际导航落点，
`recordIdentityFailed()` 只留给明确的全局规划不可达证据。controller timeout 可能是局部拥堵或落点不佳，
不能证明会随地图增长漂移的整段 frontier identity 永久不可达。与传统“失败就把 centroid 加黑名单”相比，
分层记忆既避免马上重试同一落点，也允许同一未知边界生成新的安全 approach。

### 为什么熔断必须等 Action terminal

`ProgressTimeoutCircuitBreaker.observeTerminal()` 只在 NavigateToPose result 到达后计数。cancel request 与
result 存在竞态，Nav2 仍可能晚到 `SUCCEEDED`；若请求取消时就重启 explorer，会出现两个控制 owner。
连续两次真实 timeout 后，Explore 发布 `frontier_progress_stalled_recoverable`，上层还必须核对
`active=0` 与 `accepted==succeeded+aborted+canceled`，再执行 typed STOP 和恢复扫描。扫描有地图增益才允许
新 epoch；无增益显式失败，不能把“机器人卡住”解释为“地图完成”。

正式 PASS 只接受两组严格配对：

```text
no_frontiers + no_reachable_frontiers
frontier_attempts_exhausted_recoverable + frontier_attempts_exhausted_no_map_gain
```

无论哪组，都必须 available=0、active=0、blacklisted=0，且 accepted goal 全部进入
succeeded/aborted/canceled 终态。

### 与替代方案区别

timeout 只说明预算耗尽；coverage plateau 只说明暂时没扩图；all-blacklisted 反而说明还有已知边界未
解决。三者都不能直接冒充建图完成。确认扫描使用实时 LiDAR，不读取 truth map 或固定路线；它是信息
增益判据，而不是场景先验。

### 失败边界

reason 交叉配对、blacklisted>0、active>0、available>0 或 accepted≠terminal 均失败。扫描产生足够地图
增益但 recovery budget 已耗尽也必须失败，因为不能在还有新信息时宣称收敛。

## 4. Strong typed evidence、时钟域与独立评分

### 代码在哪里

- `embodied_agent_interfaces/msg/SlamSessionState.msg`
- `embodied_agent_interfaces/msg/FrontierExplorationEvidence.msg`
- `embodied_agent_interfaces/msg/SlamNavigationGoalEvidence.msg`
- `tools/acceptance/probes/slam_nav/sampled_goal_tracker.py:SampledGoalPlanTracker`
- `tools/acceptance/unknown_world_evidence.py:build_unknown_world_report()`
- `tools/acceptance/scenarios/unknown_world_contract.py:validate_unknown_world_mission()`

### 为何这样设计

日志字符串会改文案、会丢帧，也不能表达 late join 快照；typed msg 固定字段、Action 状态、Nav2
error_code、时间和 producer path 统计。evaluator 再使用保存图、Gazebo truth 和场地区域计算覆盖、定位
误差和路径安全，不相信 producer 自报成功。

路径关联有一个容易忽略的时钟问题：orchestrator 在 Gazebo stage 外常驻，typed goal 生命周期使用
`SYSTEM_TIME`；Nav2 的 Path 在 `use_sim_time=true` 下使用 Gazebo `/clock`。这两种数值不可直接排序。
`SampledGoalPlanTracker._timestamps_share_clock_domain()` 只在同域时比较 timestamp；跨域则依靠 mission
generation、唯一 active goal、map endpoint 和有界 TTL 缓存关联。`update_state()` 与 `observe_plan()`
附近的中文注释说明了跨 topic 乱序为什么必须在同一把锁下重放。

### 与替代方案区别

- 用接收时间排序会受 executor 调度影响；盲绑“当前 active goal”会把 admission plan 或上一目标迟到
  replan 归错任务。
- 只相信 producer 形成自证循环；当前 evaluator 重新加载 map 并重新采样路径。
- 用 AMCL 轨迹拟合 world→map 真值变换会用被测结果校准答案；当前只用 evaluator 已知出生位姿变换。

### 失败边界

多 active goal、mission generation 不匹配、endpoint 不匹配、无 runtime plan、producer 与 evaluator
路径结论不一致、Action 非 SUCCEEDED 或 error_code≠0 都失败。新增 msg 字段会改变 ROS interface type
hash，必须全量重建依赖包，旧 overlay/bag 不能作为当前 wire contract 的正式证据。

## 5. SLAM 后端优化与回环

### 代码在哪里

- `src/embodied_slam/src/gtsam_pose_graph.cpp:GtsamPoseGraphOptimizer::optimize()`
- `src/embodied_slam/src/gtsam_scan_solver.cpp:GtsamScanSolver::AddNode()/AddConstraint()/Compute()`
- `src/embodied_slam/src/lidar_loop_runtime.cpp:LiveLidarLoopDetector::ingest()`
- `src/embodied_slam/src/lidar_loop_verifier.cpp:LiveLidarLoopVerifier::verify()`
- `src/embodied_slam/src/lidar_loop_constraint_gate.cpp:LidarLoopConstraintGate::evaluate()`
- `src/embodied_slam/src/instrumented_async_slam_toolbox_node.cpp`：Karto commit Adapter。

### 为何这样设计

前端 scan matching 产生相对约束，后端联合优化全局位姿。回环链是候选召回→局部子图几何验证→
时序/一致性门控→鲁棒核或 switchable constraint→优化；相似度高不等于可以直接向图里加边。policy 与
commit 分离，可先 shadow 收集 precision/recall，再决定是否改在线地图。

### 与替代方案区别

GTSAM factor graph 便于表达鲁棒核和可切换约束；Ceres 是通用非线性最小二乘及 slam_toolbox 生态
基线；g2o 轻量经典，但不是当前主实现。Cartographer/ORB-SLAM3 能快速提供系统，不等于掌握约束、
漂移和回环误匹配的工程边界。

### 失败边界

后端不能创造正确约束；错误回环会扭曲全图。当前新 LiDAR 回环默认 `commit_enabled=false`，shadow
PASS 只说明证据链完整，不代表真实场地精度达到发布门槛。A/B 必须固定 bag、前端、配置和哈希。

## 6. 动态障碍预测与重规划

### 代码在哪里

- `embodied_navigation/src/dynamic_obstacle_tracker.cpp:DynamicObstacleTracker::update()`
- `embodied_navigation/src/gated_observation_assignment.cpp`：全局门限关联。
- `embodied_navigation/src/constant_velocity_predictor.cpp:predict_constant_velocity()`
- `embodied_navigation/src/predicted_obstacle_layer.cpp:updateBounds()/updateCosts()`
- `tools/acceptance/probes/slam_nav/dynamic_scenario.py:run_showcase_dynamic_navigation()`
- `tools/acceptance/unknown_world_evidence.py` 的 dynamic navigation 判定。

### 为何这样设计

先做数据关联保持 track identity，再做 CV/Kalman/IMM 运动估计，最后把未来占用投影到 costmap。plugin
只消费统一轨迹，不了解检测器；旧 bounds 必须保留用于清除过期占用，避免“鬼墙”。验收同时比较动态
障碍加入前后的路径净空、unique plan、Action result、里程和最终零速。

### 与替代方案区别

greedy nearest 在多目标交叉时容易换 ID；全局门限分配更稳定。只靠局部控制器急停可以防撞，却不能
提前改变全局路线。二维 costmap 压平时间维较保守，但比引入完整时空规划器更适合当前项目规模。

### 失败边界

当前重型证据使用确定性合成 detection，不等于真实人群检测。它在自主三点导航完成后，由 acceptance
probe 注入固定 challenge goal/actor；这是外部测试刺激，不参与探索或三点目标采样，也不能算自主策略
选择的第四个目标。动态场景退出仍需完成障碍归位、空 detection、tracker/costmap 清除和 fresh 零速；
mission 取消则使用第 1 节的 `cancel → fresh typed STOP → Nav2 terminal` 安全事务。场景成功但清理失败时，
整个 gate 仍失败。

## 7. 证据状态与阅读顺序

fresh session `20260720T031306Z-1114546-814430c3` 已证明当前安全/恢复契约下的 schema v4 六阶段链路：
reachable coverage `99.67%`、区域最低 `97.76%`、unknown `0.33%`、障碍召回/false-free
`80.93% / 0.21%`，AMCL P95 `0.154 m`，3/3 运行时目标成功且最小间距 `5.57 m`，动态净空
`0.0245 m → 1.021 m`，最终 fresh 零速。修复只改变探索器的生产策略与运行证据，没有下调 schema v4
严格 evaluator；完整阈值、路径 `unknown/occupied/map-outside` 合同与报告位置只在
[测试手册](../TESTING.md) 维护。

建议阅读顺序：`unknown_world_slam_mission.yaml` → `mission_executor.py` → `mapped_goal_sampler.py` →
`showcase_session_node.py` → `sampled_goal_tracker.py` → `unknown_world_evidence.py`。先理解运行时不变量，再
看 evaluator 阈值；不要从成功 JSON 反推并复制一套策略逻辑。

这份 unknown-world 报告与真人语音 known-world 演示是双证据：前者证明自主建图导航，后者证明
语音交互控制。本轮未重跑真人语音，因此汇报时必须分别说明，不能宣称同一 session 同时证明两者。
