# SLAM / Nav2 学习笔记

本笔记围绕“代码在哪里、为何这样设计、替代方案有什么差异、失败边界是什么”组织。系统总览见
[架构文档](../ARCHITECTURE.md)，执行命令和硬门槛见 [测试手册](../TESTING.md)。

## 1. Unknown-world 建图到导航事务

### 代码在哪里

- `src/embodied_slam_tools/embodied_slam_tools/mission_executor.py`
  - `UnknownWorldMissionExecutor.run()`：动态捕获起点、建图、返航、存图、切换 AMCL/Nav2 和三点导航。
  - `AutomaticMissionExecutor.run()`：known-world 演示事务，不拥有 unknown-world 自主闭环。
  - `_explore_with_bounded_recovery()`、`decide_epoch_recovery()`：恢复扫描、地图增益和 epoch 预算。
  - `_assess_time_budget_saturation()`：硬预算后的 final probe、地图静默与有界饱和判定。
- `src/embodied_slam_tools/embodied_slam_tools/showcase_session_node.py`
  - `SessionOrchestratorNode._on_asr_final()`：只把开始/取消话术转成 mission intent。
  - `start_navigation()`：停止 mapping、递增 navigation generation、清理旧 map/pose，再启动保存图定位栈。
  - `select_mapped_navigation_goals()`：等待同代 `/map` 和 AMCL pose，组织候选与实时准入。
  - `run_navigation_goal()`：执行前 preflight、首次计划评分和路径证据采集，再把 Action 生命周期委托给事务。
  - `capture_mapping_start_pose()`、`quiesce_frontier()`、`run_mapping_return_goal()`：动态起点、Explorer
    Action 总账排空和 mapping-stage 返航。
  - `AgentActionGateway`：关联扫描/STOP primitive 的 Action goal/result，不拥有语音 callback 或任务阶段。
- `src/embodied_slam_tools/embodied_slam_tools/nav2_motion_transaction.py`
  - `Nav2MotionTransaction.execute()`：统一 NavigateToPose/BackUp 的发送、取消、安全 STOP 和终态证明。
  - `SampledNavigate`、`MappingReturn`、`RecoveryBackup`：不泄漏 ROS Future 的冻结 intent。
- `src/embodied_slam_tools/embodied_slam_tools/nav2_motion_ros.py:RosNav2MotionAdapter`：
  在纯事务端口与 rclpy ActionClient、ROS clock、quiescence ledger 之间转换。
- `src/embodied_slam_tools/embodied_slam_tools/stage_process_manager.py:StageProcessManager`：launch、
  map saver 与进程树副作用。
- `src/embodied_slam_tools/embodied_slam_tools/mission_configuration.py:MissionConfiguration.load()`：把
  YAML 收紧成冻结配置，Node 不再散读 key。

主调用链：

```text
ManageSlamSession / voice intent
→ UnknownWorldMissionExecutor.run()
→ 初始运动前捕获 map→base_link 起点
→ start_explorer() → FrontierExplorationMonitor.wait()
→ strict frontier 终结，或 hard-budget bounded saturation
→ quiesce Explorer → final probe → typed STOP
→ ComputePathToPose + NavigateToPose 返回动态起点
→ save_map()
→ start_navigation()：new navigation generation
→ wait_navigation_ready()
→ select_mapped_navigation_goals()
→ run_navigation_goal() × 3
→ Nav2MotionTransaction.execute(SampledNavigate)
→ RosNav2MotionAdapter → NavigateToPose ActionClient
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

返航必须发生在 mapping stage 内。若先存图、切到 AMCL 阶段或重启 Gazebo，再看到机器人位于出生点，
只能证明重启重置了模型，不能证明机器人从建图终点真实导航回来。当前先执行返航 Action，复核 TF 与新鲜
零速度，等待 SLAM/回环尾帧稳定后才存图。同步 map_saver 可能耗时数秒，因此保存返回后还要再次走
typed STOP 并读取订阅到的新零速；这条 post-save witness 通过后才允许切换导航 stage。

### 与替代方案区别

- 固定 waypoint/bootstrap 路线稳定，但它验证 known-world 回归，不证明未知环境自主探索。
- 在 YAML 写死“返航点 (0,0)”简便，但起点坐标会随 world/spawn 改变；当前在首次运动前动态捕获 TF。
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

### 1.1 真人语音怎样与 unknown-world 形成同一条证据链

#### 代码在哪里

- `voice_unknown_world_slam_e2e.py:main()`、`UnknownWorldRunProfile.live_voice()`：选择 online/offline 真人
  profile，复用同一 SLAM/Nav2 runner 和门槛。
- `showcase_session_node.py:_on_wake_event()`：真人模式只接受带 command 的 `KIND_WAKE/KIND_CONTINUE`；
  synthetic 模式才使用 raw ASR。
- `session_observer.py:begin_live_voice_trigger_window()`：MAPPING 后采集 audio、endpoint、WakeEvent 和 ASR，
  且真人模式不创建伪 `/agent/asr_final` publisher。
- `live_voice_trigger.py:finalize_trigger_report()`、
  `voice_trigger_evidence.py:VoiceTriggerEvidenceWindow.build_envelope()`：把语音事实和同 session schema v4
  核心报告组合成可单测的 schema v1 envelope。

主调用链：

```text
voice-unknown-world-slam-e2e offline|online
→ UnknownWorldRunProfile.live_voice()
→ SessionOrchestratorNode(command_input_source=wake_event)
→ SessionObserver(synthetic_asr_enabled=false)
→ MAPPING 后打开 volatile 语音窗口并提示真人口令
→ AudioFrontendStatus + speech_started/ended + WakeEvent + ASR final
→ WakeEvent 授权任务 → UnknownWorldMissionExecutor.run()
→ schema v4 SLAM/Nav2 core_report
→ VoiceTriggerEvidenceWindow.build_envelope()
→ schema v1 同 session 联合 PASS/FAIL
```

#### 为什么不让编排器直接消费裸 ASR

Agent 同时发布裸 ASR 和经过 wake/session gate 的 WakeEvent。若编排器同时消费二者，同一句话可能重复
触发；若只读 raw ASR，未唤醒语音、filler 或测试 publisher 都能启动长任务。因此
`command_input_source` 是互斥部署策略：真人只消费已授权 WakeEvent，synthetic 门禁才消费确定性 raw
ASR。探针订阅 ASR 只为证据观察，不驱动生产任务。

#### 为什么用 schema v1 envelope 内嵌 schema v4

把语音字段直接塞进 schema v4 会破坏无麦克风 SLAM/Nav2 门禁；把两次不同 session 的 PASS 拼在汇报里，
又没有因果证据。因此 envelope 分两层：

- 外层 schema v1：真实音频、闭合 VAD endpoint、WakeEvent、自动任务 final 与 agent mode。
- 内嵌 schema v4：地图质量、frontier 终止、AMCL、3 个运行时目标、动态重规划和最终零速。

两层必须复用同一 `session_id`，任一 check 失败则联合报告失败；MAPPING 后才打开 volatile 事件窗口，
排除启动期旧 final。

#### 与替代方案区别和失败边界

- “语音先测一次、SLAM 再测一次”快，但没有因果和同会话证据；当前只接受一条命令启动同一事务。
- 用时间戳猜 raw ASR 是否经过唤醒很脆弱；当前生产侧直接消费强类型 `WakeEvent`。
- 仅检查 RMS 会把持续噪声当语音；还需要闭合 endpoint、可解析 final 和已授权 WakeEvent。
- 外层 PASS 不得覆盖核心失败；`_verify_profile_report()` 会复核 schema/session/checks/sections。

截至本笔记更新，联合入口已实现但真人现场 PASS 尚未生成；历史 v4 不能外推为真人联合结果。

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
- `mission_executor.py:UnknownWorldMissionExecutor._explore_with_bounded_recovery()`：碰撞检查 BackUp、
  odom 位移验证与一次性 final confirmation epoch。
- `exploration_saturation.py:SaturationEvidenceTracker`、`assess_bounded_frontier_saturation()`：跨 epoch
  保留峰值地图、terminal goal、路径和 final probe 的 ROS-free 判定。
- `mapping_return.py:evaluate_return_to_start()`：返航 Action、位姿、时间线和零速度的纯领域裁决。
- `showcase_session_node.py:quiesce_frontier()`、`run_mapping_return_goal()`：ROS 控制面静默和真实返航。
- `mapping_evidence.py:MappingEvidenceTracker`：累计 accepted/terminal goal，而恢复重启不能抹账。
- `unknown_world_slam_mission.yaml`：`frontier_idle_grace_s=20.0`、`min_growth_cells=40`、
  `min_growth_ratio=0.002`。
- `tools/acceptance/unknown_world_evidence.py:evaluate_frontier_completion()`：最终严格 reason pair 与计数。
- `tools/acceptance/unknown_world_evidence.py:evaluate_approximate_completion()`、
  `evaluate_return_to_start_evidence()`：验收层近似完成与返航复核。
- `patches/m-explore-ros2/0012-execution-clearance-contract.patch`：`0.33 m` 可逃逸连通域及临界通道测试。
- `patches/m-explore-ros2/0013-frontier-progress-recovery-contract.patch`：progress anchor、分层失败记忆、
  terminal 后熔断。
- `patches/m-explore-ros2/0014-observable-approach-escape-contract.patch`：可观测半径、最新安全复核和
  “有效逃逸打断连续静止”契约。
- `scripts/prepare_frontier_nav2_params.py:with_frontier_progress_checker()`：把 Nav2 实际使用的
  `0.10 m / 30 s` progress checker 写入本 session 参数 YAML。

### 为何这样设计

Explore Lite 的 `no_frontiers` 是 provider 对当前地图的事实；任务层的 `no_reachable_frontiers` 是在
idle grace 后对可达性和 Action 生命周期的判断。goal terminal 后可能短暂出现 available=active=0、
blacklisted>0；`20 s` idle grace 让 provider 的下一次 makePlan 消费终态，避免任务层过早重启 Explorer。
generic all-blacklisted 本身不授权移动。若一个 epoch 的物理 approach 都耗尽，provider 必须发布 typed
`frontier_attempts_exhausted_recoverable`；待 `active=0`、`accepted==terminal` 后，任务层才执行碰撞检查
BackUp，并用 odom 位移证明观察位置确实改变，再做传感器驱动的 360° 扫描和地图稳定等待。独立的 typed
no-clearance 原因也可走相同的 relocation 契约，普通平台期/blacklisted 不可绕过它。

扫描后只有绝对增益与相对增益均低于有效增长门槛，才发布
`frontier_attempts_exhausted_below_material_gain`。

当前门槛由 `unknown_world_slam_mission.yaml` 的 `min_growth_cells=40` 与
`min_growth_ratio=0.002` 共同定义，等价于
`required=max(40, ceil(known_before*0.2%))`。这避免完整大图上几十个边缘栅格抖动被误判成新房间。
`attempts_exhausted` 中的 residual blacklist 是本 epoch 已尝试失败的候选子集；只有执行过有碰撞检查的
Nav2 BackUp、完成多视角恢复且 Action 账本排空后，才允许由独立地图质量门禁最终确认收敛。

若最后一个恢复预算刚好被消耗，而扫描又同时达到 `40 cells + 0.2%`，直接失败会浪费已经获得的新地图，
直接完成则 Explorer 从未消费 post-scan frontier。任务层因此只授权一次 recovery-budget-neutral final
confirmation epoch：使用独立 `240 s` 绝对 deadline，不 BackUp、不再扫描、不创建第二个确认轮。该轮只有 provider
`no_frontiers` 或 typed attempts exhaustion 可以完成；其他 recovery reason、active/Action 账本异常或二次
非终结恢复都失败。

“等待地图稳定”不是一个可无限延长的定时器：soft quiet 从最后一次地图增长计算，用来吸收尾部更新；
hard budget 从调用开始计算且增长不会重置，用来限制噪声或持续增长造成的无限等待。quiet 未满足继续等，
hard deadline 到达则失败，因此 timeout 不会被伪装成收敛。

### 未知场地大小时怎样判断“扫得差不多”

运行时看不到真值和总面积，因而不能计算“已经完成 85%”。硬预算只触发一个中性的 assessment request，
不能直接授权成功。任务层先通过 `/explore/resume=false` 暂停 Explore Lite，等待
`exploration_paused + active=0 + accepted=terminal`，再回收进程；这样 late Action result 不会在进程被杀后
丢失，也不会留下两个速度 owner。

随后只执行一次 final probe。`SaturationEvidenceTracker` 使用峰值 known cells，避免回环导致当前已知
栅格变少时伪造负收益；同时用 terminal goal 数归一化 epoch 增益，避免“epoch 跑得久”天然看起来收益更高。
默认必须满足最近 `2` 个连续低收益 epoch、每轮 `3` 个 terminal goal、累计路径 `20m`、残余 available
frontier `<=4`、final probe 低增益、地图静默 `15s`、账本排空和 probe 后新鲜 typed STOP。恢复次数余量仅用于
诊断；provider 未请求恢复时，不会为了把计数降到零而制造额外运动。

生产侧只记录 `time_budget_exhausted + bounded_saturation evidence`。evaluator 再使用 truth map 独立验证
原有 `90/85/10` 地图质量与障碍质量；因此这是“运行时收益递减 + 离线质量门禁”的双层证据，不是把
timeout 或视觉上的残余白角改名为成功。

### 为什么 Explorer 停止后还要返回起点

起点在初始扫描前从 `map→base_link` 动态捕获。探索结束后，返航先用 `ComputePathToPose` 确认本次图上
存在 known-free 路径，再执行 `NavigateToPose`；Action succeeded 仍不够，还要连续 TF 样本进入 XY/yaw
容差并收到返航后的零速度。返航会再次观测起点附近并可能触发回环，所以还要等待地图静默后再保存；
保存完成后的第二次 STOP 用于建立最终报告的新鲜零速边界，而不是给旧时间戳“续期”。

`mapping_return.py` 把这些条件压成纯领域契约，并验证时间线
`capture < return start < return finish < map save`。evaluator 逐项复核 checks，不能只相信 producer 的
`passed` 位。这样 strict frontier 和 bounded saturation 使用同一返航安全门槛。

直接把 `max_cmd_vel_age_s` 从 1 秒调大看似能绕过慢 map_saver，但它会让真正陈旧的停车证据也通过。
当前保留严格阈值，并以“返航后预存图停车 + 存图后最终停车”的双确认协议消除耗时耦合。

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

strict frontier PASS 只接受以下配对：

```text
no_frontiers + no_reachable_frontiers
frontier_attempts_exhausted_recoverable + frontier_attempts_exhausted_below_material_gain
frontier_attempts_exhausted_recoverable + frontier_attempts_exhausted_after_final_confirmation
```

无论哪组，都必须 available=0、active=0，且 accepted goal 全部进入 succeeded/aborted/canceled 终态。
`no_frontiers` 必须 blacklisted=0；typed attempts exhaustion 允许 residual
`blacklisted<=detected`，但最终地图覆盖、分区覆盖、unknown 和障碍质量仍可独立否决。

bounded saturation 不要求 available=0，但 residual 必须在配置上限内，且同时满足跨 epoch、final probe、
账本、返航和 evaluator 地图质量证据；它不会改变 `evaluate_frontier_completion()` 的 strict 语义。

### 与替代方案区别

timeout 只说明预算耗尽；单次 coverage plateau 只说明暂时没扩图；all-blacklisted 反而说明还有已知边界
未解决。三者都不能单独冒充建图完成。当前要求多 epoch 收益、物理路径、Action ledger、最终探测和独立
地图质量共同成立；确认扫描使用实时 LiDAR，不读取 truth map 或固定路线。

### 失败边界

reason 交叉配对、`no_frontiers` 时 blacklisted>0、attempts exhaustion 时 blacklisted>detected、active>0、
available>0 或 accepted≠terminal 均失败。预算边界扫描显著扩图时必须经过唯一 final confirmation；若确认
轮不是 `no_frontiers` 或 typed attempts exhaustion，也必须失败。bounded saturation 缺少任一低收益 epoch、
final probe、静默、STOP、返航或 evaluator 地图质量证据同样失败。

## 4. Strong typed evidence、时钟域与独立评分

### 代码在哪里

- `embodied_agent_interfaces/msg/SlamSessionState.msg`
- `embodied_agent_interfaces/msg/FrontierExplorationEvidence.msg`
- `embodied_agent_interfaces/msg/SlamMappingCompletionEvidence.msg`
- `embodied_agent_interfaces/msg/SlamNavigationGoalEvidence.msg`
- `tools/acceptance/probes/slam_nav/sampled_goal_tracker.py:SampledGoalPlanTracker`
- `tools/acceptance/unknown_world_evidence.py:build_unknown_world_report()`
- `tools/acceptance/unknown_world_evidence.py:evaluate_approximate_completion()`、
  `evaluate_return_to_start_evidence()`
- `tools/acceptance/scenarios/unknown_world_contract.py:validate_unknown_world_mission()`
- `tools/acceptance/probes/slam_nav/artifacts.py:build_failure_report()`：失败也保存最后一帧 typed
  frontier/Action ledger。

### 为何这样设计

日志字符串会改文案、会丢帧，也不能表达 late join 快照；typed msg 固定字段、Action 状态、Nav2
error_code、时间和 producer path 统计。evaluator 再使用保存图、Gazebo truth 和场地区域计算覆盖、定位
误差和路径安全，不相信 producer 自报成功。

失败报告同样保留 `frontier.telemetry`：detected/available/active/blacklisted 与 accepted/succeeded/
aborted/canceled 都来自 typed 消息。这样恢复失败能直接审计 Action 总账，不必从可变的 `detail` 日志反推。

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
- `tools/acceptance/probes/slam_nav/dynamic_scenario.py:path_relative_motion_positions()`
- `tools/acceptance/probes/slam_nav/dynamic_cost_evidence.py:PredictedCostLatch`
- `tools/acceptance/unknown_world_evidence.py` 的 dynamic navigation 判定。

### 为何这样设计

先做数据关联保持 track identity，再做 CV/Kalman/IMM 运动估计，最后把未来占用投影到 costmap。plugin
只消费统一轨迹，不了解检测器；旧 bounds 必须保留用于清除过期占用，避免“鬼墙”。验收同时比较动态
障碍加入前后的路径净空、unique plan、Action result、里程和最终零速。

动态 actor 不再放到固定地图坐标，而是沿本次 baseline path 搜索双侧具有足够横向净空的 anchor，再按
切向速度生成观测序列；场景/起点变化时仍在验证“有解路径上的预测阻塞”。预测层 TTL 可能早于多次
ComputePath 重试结束，因此首次和最大 lethal cost 在等待窗口内锁存，route-commit 即时 cost 只作衰减
诊断，不能倒写已经观测到的历史事实。

### 与替代方案区别

greedy nearest 在多目标交叉时容易换 ID；全局门限分配更稳定。只靠局部控制器急停可以防撞，却不能
提前改变全局路线。二维 costmap 压平时间维较保守，但比引入完整时空规划器更适合当前项目规模。

### 失败边界

当前重型证据使用确定性合成 detection，不等于真实人群检测。它在自主三点导航完成后，由 acceptance
probe 注入固定 challenge goal/actor；这是外部测试刺激，不参与探索或三点目标采样，也不能算自主策略
选择的第四个目标。动态场景退出仍需完成障碍归位、空 detection、tracker/costmap 清除和 fresh 零速；
fresh 零速必须晚于本次动态 Action 的 terminal boundary。mission 取消则使用第 1 节的
`cancel → fresh typed STOP → Nav2 terminal` 安全事务。场景成功但清理失败时，
整个 gate 仍失败。

## 7. Nav2 运动事务：把安全复杂度藏在一个深模块里

### 代码在哪里

- `nav2_motion_transaction.py:Nav2MotionTransaction.execute()`：唯一业务入口。
- `nav2_motion_transaction.py:_execute_motion()`：统一发送、等待与取消路径。
- `nav2_motion_transaction.py:_resolve_pending_goal_safely()`：同步收口迟到接受。
- `nav2_motion_transaction.py:_cancel_stop_and_prove_terminal()`：执行
  `cancel -> priority STOP -> terminal`。
- `nav2_motion_ros.py:RosNav2MotionAdapter`：把纯事务端口适配到
  `NavigateToPose`、`BackUp`、ROS clock 和现有 quiescence ledger。
- `showcase_session_node.py:run_navigation_goal()`：保留候选 preflight；Node 层采集
  `/plan`、按 occupancy 评分并写入 ledger，事务再读取 ledger 强制执行运行期路径
  安全门，随后完成 `SampledNavigate` 的 Action 生命周期。
- `showcase_session_node.py:run_mapping_return_goal()`：提交 `MappingReturn`，成功后
  再验证 TF 返航误差及独立停车证据。
- `showcase_session_node.py:run_recovery_backup()`：把恢复动作提交为
  `RecoveryBackup`，位移增益判定仍由 Node 负责。
- `test_nav2_motion_transaction.py`：28 个纯接口级事务测试。
- `test_nav2_motion_ros.py`：5 个 ROS Adapter 转换与边界测试。
- `test_showcase_session_node.py`：只验证 Node 的 preflight、证据 owner 和委托 seam，
  不再伪造内部 Future 状态机。

### 调用链怎样走

```text
SessionOrchestratorNode
  -> 候选目标 preflight / 计划证据
  -> Nav2MotionTransaction.execute(intent, deadline, is_cancelled)
       -> RosNav2MotionAdapter.wait_for_server()
       -> RosNav2MotionAdapter.send_goal()
       -> Nav2 response / result
       -> 异常时 cancel -> typed priority STOP -> readable terminal
       -> quiescence ledger terminal
  -> Node 继续处理地图、TF、采样目标或恢复增益证据
```

对调用者只有一个行为 Interface：

```text
execute(
  intent: SampledNavigate | MappingReturn | RecoveryBackup,
  *,
  is_cancelled,
  deadline_monotonic,
) -> None
```

三个冻结 intent 只描述“要执行什么”，不暴露 ROS Future、goal handle 或清理次序。
成功返回 `None`；拒绝、运动失败、安全失败和并发分别抛出
`Nav2GoalRejected`、`Nav2MotionFailed`、`Nav2SafetyFailure` 和
`Nav2TransactionBusy`；server 不可用或业务 deadline 耗尽使用 `TimeoutError`，
用户取消沿用 `AutomaticMissionCancelled`。

### 为什么不是简单把函数挪到另一个文件

如果 Node 仍要决定何时读 response、怎样处理 late accept、先 cancel 还是先 STOP，
新文件就只是浅转发层。当前事务自己拥有以下不变量，因此删除它会把复杂度重新泄漏
给所有调用者：

1. 同时最多一笔运动事务；并发进入显式失败。
2. 正常执行使用调用者给出的绝对 deadline；安全清理使用一次创建、不可续期的有界
   cleanup deadline，不能在每个步骤重新获得完整超时。
3. response timeout 或用户取消后仍等待并收口迟到的 accepted handle。
4. accepted goal 的异常严格执行
   `cancel -> 独立 priority STOP -> readable terminal`；即使剩余预算为 0，也会先
   发出 STOP，再按 fail-closed 报告证据不足。
5. terminal 无法证明时停止 navigation stage，并抛出 `Nav2SafetyFailure`。
6. 主错误保持为主诊断，cancel、STOP、terminal 或 stage cleanup 错误作为附加信息。
7. quiescence 只在明确拒绝或可读 terminal 后结算；sampled ledger terminal 恰好
   记录一次，并保留真实 Nav2 终态。

`RosNav2MotionAdapter` 是基础设施 seam：事务测试用 scripted Adapter 控制每一种
竞态，生产才绑定 rclpy ActionClient。这样测试验证的是生产同一 Interface，而不是
重新写一套“看起来像 ROS”的 Node 白盒流程。

### 与替代方案的区别

- **继续放在 Node**：改动少，但三个动作类型会继续复制安全清理，Future 竞态只能靠
  大量脆弱白盒测试覆盖。
- **直接做 `DefaultDemoSession`**：长期 Locality 更好，可统一语音、typed Action 与
  shell；但会同时触及启动、控制权、证据和关闭语义，当前迁移风险更大。
- **先做 phase DSL**：扩展上限高，但现在只有 mapping/navigation 两个深 phase，
  尚没有第三个行为证明 registry、artifact 和 effect 类型值得公开。

因此本轮先落地运动事务；`DefaultDemoSession` 保留为下一层候选，
`MissionProgram`/phase DSL 暂缓。ROS topic、Action、Service、QoS、
`SlamSessionState` 与 typed evidence schema 均未改变。

### 失败边界与验证

纯接口测试覆盖成功、拒绝、执行失败、用户取消、response timeout、late accept、
STOP 失败、terminal 缺失、callback 异常和并发边界；ROS Adapter 测试覆盖 goal
转换、时间戳、result wrapper 与零预算 STOP。Node 旧 Future 白盒测试已由这些接口
测试替代，而不是叠加保留。

包级测试、repository contracts 和 `acceptance_test.sh core` 证明代码与 ROS
契约；未知地图重型 E2E 才能证明真实 Gazebo/Nav2 中的运动、取消、终态和最终零速。
未来 `DefaultDemoSession` 落地时还需增加
`RUN_DEFAULT -> MISSION_COMPLETED -> STOP -> STOPPED` 的会话级证据。

## 8. 证据状态与阅读顺序

历史 strict baseline session `20260721T072342Z-2344751-5452a492` 证明了 schema v4 六阶段业务链路，
但它早于本轮 `Nav2MotionTransaction`，不能替代当前分支的重型回归。该历史 session 的 reachable
coverage `99.81%`、区域最低 `98.76%`、unknown `0.19%`、障碍召回/false-free `85.21% / 1.69%`，
AMCL P95 `0.120 m`，返航位置/角度误差 `0.0188 m / 0.1828 rad`，3/3 运行时目标成功，动态净空
`0.020 m → 0.998 m`，最终 fresh 零速。该次地图自然满足 strict frontier；bounded saturation 是
残余前沿仍存在时的备用收口路径，不能因本次未触发而删掉。生产与 schema v4 evaluator 阈值均未下调。完整阈值、
路径 `unknown/occupied/map-outside` 合同与报告位置只在
[测试手册](../TESTING.md) 维护。

本轮事务重构已由 clean commit `bc65b8f` 的独立 unknown-world session
`20260724T143855Z-274112-d0619407` 验证；根因闭环与完整指标见
[工程日志](../development/multimodal_showcase/ENGINEERING_LOG.md)。

建议阅读顺序：`unknown_world_slam_mission.yaml` → `exploration_saturation.py` / `mapping_return.py` →
`mission_executor.py` → `showcase_session_node.py` → `nav2_motion_transaction.py` →
`nav2_motion_ros.py` → `mapped_goal_sampler.py` → `sampled_goal_tracker.py` →
`unknown_world_evidence.py`。先理解运行时不变量，再
看 evaluator 阈值；不要从成功 JSON 反推并复制一套策略逻辑。

这份 unknown-world 报告与真人语音 known-world 演示是双证据：前者证明自主建图导航，后者证明
语音交互控制。本轮未重跑真人语音，因此汇报时必须分别说明，不能宣称同一 session 同时证明两者。
