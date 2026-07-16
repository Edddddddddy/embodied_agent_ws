# SLAM 建图、后端优化与定位导航工程笔记

## 1. 本阶段解决了什么

本阶段形成了可度量的第一条闭环：

```text
Gazebo LaserScan + 参考里程计
  -> 可配置、固定 seed 的漂移注入
  -> slam_toolbox 激光前端
  -> Ceres 或项目 GTSAM 位姿图后端
  -> 5 cm OccupancyGrid + 地图保存
  -> 新进程加载地图
  -> AMCL map->odom
  -> Nav2 全局路径/局部控制
  -> 动态障碍位置关联与速度估计
  -> 未来占用 Nav2 costmap layer
  -> 全局重规划与局部避障
  -> NavigateToPose result + cmd_vel 归零
```

它与“只启动 Cartographer、ORB-SLAM3 或 slam_toolbox 看见一张图”的区别是：输入漂移、
参考轨迹、后端校正轨迹、地图覆盖和导航结果都有结构化证据，可以说明漂移来自哪里、
回环约束如何进入图、后端优化修正了多少，以及保存地图是否真的能够复用。

面向现场展示的四区域真实感场景、语音探索、地图保存、坐标对齐和语义导航步骤见
[VOICE_SLAM_NAV_SHOWCASE.md](VOICE_SLAM_NAV_SHOWCASE.md)。该路径负责“可观看的完整 Demo”，
本文件后续章节负责“可量化的算法证据”，两者不相互冒充。

## 2. 关键文件与代码流程

| 技术点 | 关键文件/函数 | 设计理由 |
| --- | --- | --- |
| 受控里程计漂移 | `src/embodied_slam/src/drift_model.cpp`：`DriftModel::update/reset` | 尺度误差、每米航向偏置和高斯噪声均可配置；固定 seed 让两个后端收到可重复输入 |
| 漂移 TF/传感器隔离 | `odom_drift_injector_node.cpp`：`on_odometry/on_scan` | 创建 `slam_odom -> slam_base_link -> slam_laser` 独立 TF 树，不污染 Gazebo 参考 `/odom` |
| 固定闭环路线 | `closed_loop_controller.cpp`、`closed_loop_driver_node.cpp`：`update/step` | 同一四边形路线用于 Ceres/GTSAM A/B；雷达近障停车仍保留 |
| Ceres 基线 | `src/embodied_slam/config/slam_mapping_ceres.yaml` | 使用 slam_toolbox 官方默认支持的 Ceres + Huber，作为稳定参照 |
| GTSAM 深模块 | `gtsam_pose_graph.cpp`：`GtsamPoseGraphOptimizer::optimize` | 用纯 Pose2/constraint 接口隔离 GTSAM，支持 none/Huber/Cauchy、硬门控和逐回环 switch |
| ScanSolver Adapter | `gtsam_scan_solver.cpp`：`AddNode/AddConstraint/Compute` | 将 karto 节点、相对位姿和协方差适配为 GTSAM Prior/Between factors，通过 pluginlib 注入 slam_toolbox |
| 固定图后端消融 | `gtsam_graph_optimize.cpp`、`run_gtsam_robust_kernel_ablation.py` | 累计去重的前端图只生成一次，四种后端复用相同 SHA256 输入，隔离异步前端波动 |
| 非局部边一致性门控 | `gtsam_pose_graph.cpp`：`violates_consistency_gate` | 不用真值，比较候选约束与优化前图预测；硬拒绝明显异常边，中等残差仍交给鲁棒核 |
| 可切换回环约束 | `gtsam_pose_graph.cpp`：`SwitchableBetweenFactor/evaluateError` | 为每条非局部边联合优化独立可信度；错误边可软关闭，正确边保留，不修改前端图输入 |
| 多序列 switch 消融 | `compare_gtsam_switchable_sequences.py`：`compare` | 要求两份独立 graph SHA、同一先验/阈值，并同时观察“压低错误边”和“保留正常边” |
| 协方差防护 | `make_positive_definite` | 对称化协方差并钳制特征值，防止走廊等退化几何给出奇异矩阵导致求解器崩溃 |
| 地图/轨迹报告 | `tests/integration/slam_nav/test_slam_mapping_baseline.py`：`build_report` | 同时统计原始 ATE、闭环误差、`map->odom` 校正轨迹和已知地图面积 |
| 后端 A/B | `tools/evaluation/compare_slam_backends.py`：`compare` | 检查两次路线与漂移尺度一致，再比较校正 ATE、闭环误差、覆盖面积和时间 |
| 公开 bag 回放 | `bag_source.py`：`inspect_bag/iter_events`；`replay_node.py`：`replay` | 懒加载 ROS 1/2 消息，发布单调 `/clock`、隔离 TF 与 scan；不把 9 GB bag 读入内存 |
| 轨迹记录 | `trajectory_recorder.py`：`_on_tf/_record` | 显式组合 `map→odom→base`，逐样本 flush，避免只记录后端校正量或退出丢证据 |
| 真实后端 A/B | `compare_openloris_backends.py`：`compare` | 先验证样本窗/覆盖率可比，再描述 ATE/RPE 差异，不预设某个求解器必胜 |
| 退化分段 | `analyze_slam_degradation.py`：`analyze` | 与全局报告共享时间关联/SE(2) 对齐；运动学类别自动统计，动态遮挡必须人工标注 |
| 证据固化 | `build_openloris_experiment_manifest.py`：`build_manifest` | 将数据/配置/commit/日志/指标哈希绑定，防止脱离上下文引用数字 |
| 地图复用 | `localization_navigation.launch.py` | 关闭 SLAM，加载保存的 YAML/PGM，启动官方 Nav2/AMCL 生命周期栈 |
| 定位规划验收 | `test_slam_localization_navigation.py` | 明确等待 `map->odom` 和 BT Navigator ACTIVE，再检查 plan、Action result、odom 与零速 |
| 动态目标跟踪 | `dynamic_obstacle_tracker.cpp`：`DynamicObstacleTracker::update` | 对标准 `PoseArray` 检测做全局门控关联，并在 CurrentOnly、常速度、Kalman、IMM 四种模型间消融，统一处理置信度和超时淘汰 |
| 运动预测深模块 | `constant_velocity_predictor.cpp`：`predict_constant_velocity` | 与 ROS 解耦的纯函数；按时间步生成未来占用圆，并随预测时域膨胀不确定性半径 |
| Nav2 预测层 | `predicted_obstacle_layer.cpp`：`on_obstacles/updateBounds/updateCosts` | pluginlib Layer 将未来轨迹写入全局 costmap；旧 bounds 参与清除，空观测仍保持 costmap current |
| 动态避障验收 | `test_predicted_dynamic_obstacle_navigation.py` | 测量 track 速度、未来 cell cost、重规划前后路径净空、Nav2 result、里程和零速 |

## 3. 回环检测与后端优化怎么讲

### 3.1 前端不是后端

激光前端负责决定“哪些扫描可能相关”，并通过 scan matching 估计相对位姿与协方差。
顺序相邻帧形成里程计/局部匹配边；机器人回到旧区域时，前端先按空间距离与扫描链长度
筛选候选，再做粗分辨率相关匹配和细匹配。只有响应分数、方差等门槛通过，才新增一条
跨时间的回环边。配置入口在两个 `slam_mapping_*.yaml` 的
`loop_search_*`、`loop_match_*` 和 `correlation_search_*` 参数。

后端接收的是二维位姿图：节点是扫描位姿 `x_i=(x,y,theta)`，边是测量
`z_ij` 和信息矩阵。目标是最小化所有边的加权残差：

```text
argmin Σ ρ( || Log( z_ij^-1 * (x_i^-1 * x_j) ) ||²_Ωij )
```

首节点 Prior factor 消除整张图可任意平移/旋转的规范自由度。Huber 在小残差区保持二次损失、
大残差区转为线性；Cauchy 对极大残差进一步降权，更适合已接受图中存在明显离群边的情况。
优化结果不会重写轮速里程计，而是更新 `map->odom`，所以局部控制仍保持
连续，地图和全局位姿则被整体校正。

本项目的 `loop_only` 实际按 node ID 间隔识别“非局部边”，用于后端防护而非正式回环标签。
正式 precision 必须使用 Karto 原生 closure callback 与独立真值相对位姿残差；两套口径不能混用。

鲁棒核与一致性门控解决的区间不同：鲁棒核为每条残差连续赋权，适合中等异常；一致性门控在
优化前计算候选边测量和当前图预测的 SE(2) 创新量，只拒绝超过平移/偏航阈值的明显异常非局部边。
它不读取真值，能够在线运行，但当前图已经严重漂移时也可能误拒绝真正纠偏的回环。因此项目配置
默认关闭门控，只在固定图消融中展示收益与风险，不以单条序列自动选择生产阈值。

可切换约束在回环残差前乘一个标量 `s_ij`，并用先验把它拉向 1：

```text
argmin Σlocal ||r_ij(x)||² + Σloop ||s_ij r_ij(x)||² + ||1 - s_ij||² / σ_s²
```

`SwitchableBetweenFactor::evaluateError` 同时提供 source pose、target pose 和 switch 的 Jacobian；
`PriorFactor<double>` 防止所有 switch 无条件归零。硬门控是在优化前做离散拒绝，Huber/Cauchy 是
按残差统一定义的 M-estimator，而 switch 是“每条回环一项”的联合潜变量：它能让明显错误边接近
0，也能让一致边保持接近 1。代价是增加变量/因子和非凸性，先验过弱会误关真回环，先验过强则
退化成普通 BetweenFactor。因此项目默认关闭，仅在固定图上使用相同参数做 A/B。

真实两序列结果：`corridor1-1` 中 80/858 条 switch 低于 0.5，Cauchy ATE 由 1.2236 m 降至
1.0323 m；`corridor1-2` 只有一条非局部边，switch 为 0.9976，ATE 无实质变化。按 2316 个匹配
位姿加权后，Switchable+Cauchy 为 0.9196 m，相比 Gaussian/Cauchy 分别下降 43.28%/15.57%。
这些数字说明后端对“已接受边”更稳，不说明前端 closure precision/recall 变好。

### 3.2 GTSAM、Ceres、g2o 的差异

- Ceres 是通用非线性最小二乘库，自动求导和线性求解器选择成熟；slam_toolbox 默认
  插件稳定，适合作基线，但图结构、变量流形需要插件自己组织。
- GTSAM 直接表达 factor graph，PriorFactor、BetweenFactor、noise model 和 Pose2
  语义清楚，也便于进一步切换 iSAM2 增量优化。本项目当前使用批量 LM，先保证与 Ceres
  比较公平；后续可在图规模变大时引入增量更新。
- g2o 同样面向图优化，顶点/边接口直接、机器人社区使用广；相比 GTSAM 的 factor/value
  抽象更贴近手写图结构。本阶段没有把“安装 g2o”当成果，因为还没有项目实现和数据证据。

### 3.3 为什么回环不能只看“地图变直了”

回环链路至少要拆成四步讲：候选检索、几何验证、加图约束、全局优化。候选检索追求召回，
几何验证用 scan matching 分数和协方差抑制假阳性；通过后才添加跨时间 Between factor，
最后由鲁棒核后端分摊累计漂移。本项目用固定路线和固定 seed 控制输入，再同时报告优化前后
ATE 与闭环误差，避免只凭 RViz 截图判断。OpenLORIS `corridor1-1` 已能计算 accepted closure
precision/event recall，并暴露了错误 closure；但它不是逐个 rejected candidate 的完整标注集，
因此不能声称已得到全候选 PR 曲线。

### 3.4 为什么回环前端要保留 Top-K 多假设

重复走廊中，单帧最高质量候选未必是真地点。旧策略先为每个 query 贪心选一个候选，再维护唯一
时序轨迹；一旦误候选得分更高，正确的第二/第三名会在积累跨帧证据前被丢弃。新
`LidarLoopSequenceConsistency::observeBatch` 同时接收一个 query 的 Top-K：只有 query/candidate
时间都向前、两侧时间增量近似一致、ICP 相对位姿变化连续时才延伸假设；确认长度优先保留，状态
上限 64 条。它接近 SeqSLAM 的序列判别思想，但输入是几何验证后的 typed 候选，不依赖图像或
训练模型。

两条 OpenLORIS 走廊序列使用同一组 `3 confirmations / 2.0 s query gap / 0.25 s pair-age /`
`0.35 m / 0.20 rad` 参数：相对单轨门，逐序列 precision 从 12.50%/50.00% 提高到
33.33%/60.00%，聚合假接受 11→6，真接受保持 5。它证明多假设优于“先贪心、再时序”，但聚合
conditional recall 仍仅 2.99%，所以 `commit_enabled=false` 不变。与 PCM 最大团相比，本实现
计算和在线状态更轻，但只检查局部序列连续性，无法验证任意两条远距闭环的全局环路一致性；与
学习式地点识别相比，它可解释且无需训练集，但对整段结构重复仍缺少语义判别力。

## 4. 预测动态障碍如何进入 Nav2

```text
PoseArray detections
  -> DynamicObstacleTracker（关联 ID、估计 vx/vy、置信度/超时）
  -> DynamicObstacleArray typed topic
  -> ConstantVelocityPredictor（0~2 s 未来点 + 时域不确定性膨胀）
  -> PredictedObstacleLayer（pluginlib）
  -> global_costmap lethal cells -> inflation
  -> NavFn/BT Navigator 重规划 -> controller
```

跟踪器提供 CurrentOnly、常速度、Kalman 和 IMM 四种可解释模型，并通过固定输入做消融。
代价层当前仍按发布的位置与速度做常速度外推：优势是 CPU 开销小、参数清楚、可独立单测；
缺点是急转、急停和多人交叉时预测误差较大。相比只把当前检测点写入 obstacle layer，预测层
能在行人尚未走到机器人直线路径前提前让路。相比 TEB/MPPI 内部的时空轨迹优化，当前实现
作用在全局二维代价地图，接入简单但时间维被压平；后续可发布协方差或多模态轨迹并接入支持
时空障碍的局部控制器。

### 4.1 current-only / CV / Kalman / IMM 消融

外部接口仍只有 `DynamicObstacleTracker::update()`；`motion_model` 参数选择四个内部 Adapter：

- `current_only`：只保留当前观测，速度为零，用作“不预测”基线。
- `constant_velocity`：有限差分速度加指数平滑，计算最轻，但停车后容易过冲。
- `kalman`：二维位置—速度状态分别进行协方差预测和观测更新，抑制测量噪声。
- `imm`：低运动/机动两个 Kalman 模型按转移概率交互，混合状态与协方差，再用观测似然更新模型概率。

固定 91 帧转向、停车、短遮挡场景的 C++ 实测如下；耗时只用于说明量级，不设置 CI 性能门槛：

| 模型 | 位置 RMSE/m | 0.75 s 预测 RMSE/m | 遮挡 RMSE/m | 停车预测 RMSE/m |
| --- | ---: | ---: | ---: | ---: |
| current-only | 0.0628 | 0.3378 | 0.1944 | 0.0399 |
| constant velocity | 0.0417 | 0.2434 | 0.0654 | 0.1560 |
| Kalman | 0.0602 | 0.2717 | 0.1663 | 0.1180 |
| IMM | 0.0383 | 0.2274 | 0.0457 | 0.0389 |

四轮 Gazebo/Nav2 横穿场景也全部通过：future cell cost=254、动态路径净空约
0.934～0.979 m、里程计移动约 1.85～1.89 m、导航成功且最终零速。短短 4 帧启动时 IMM
速度估计偏保守，因此不能把上表解释为“IMM 在所有阶段总是最优”；RMSE 报告与导航闭环报告
必须分开表述。

## 5. 当前量化结果

一次固定闭环路线约 9.6 m，地图分辨率 0.05 m。阶段实测示例：

| 指标 | Ceres | GTSAM |
| --- | ---: | ---: |
| 原始里程计 ATE RMSE | 0.3500 m | 0.3498 m |
| 校正后 ATE RMSE | 0.1063 m | 0.1096 m |
| 原始闭环误差 | 0.7398 m | 0.8073 m |
| 校正后闭环误差 | 0.2008 m | 0.2812 m |
| 已知地图面积 | 23.47 m² | 23.52 m² |

仿真物理存在细微非确定性，因此门禁不预设 GTSAM 必须胜过 Ceres；它要求两者的路线、
原始漂移和地图覆盖可比，且都显著降低误差。真实环境结论必须继续用真实 rosbag 验证。

地图复用验收中，AMCL 发布 `map->odom`，Nav2 生成最长 81 点路径，机器人里程计移动
1.84 m，`NavigateToPose` 成功后 `/cmd_vel` 回零。报告位于 `logs/`，不会提交二进制地图。

预测动态障碍重型验收中，横穿轨迹的估计速度为约 0.443 m/s，1 秒未来位置的 costmap
代价为 254；基线路径到预测点的净空约 0.011 m，注入预测层后的规划净空约 0.976 m。
随后停止检测让 track 按 TTL 清除，机器人重规划并成功到达 2.1 m 目标，最终速度归零。
证据写入 `logs/dynamic_obstacle_navigation_report.json`。

四模型重型消融把横穿检测日程移到
`src/embodied_navigation/config/dynamic_obstacle_crossing_scenario.json`。汇总报告只有在场景、地图
栅格和 Nav2 参数 SHA256 全部一致时才通过，避免“只切模型”的 A/B 实验实际混入地图或参数变化。
最近一轮四种模型均完成 lethal cost、净空提升、到达和最终零速；动态路径净空分别约为
0.942/0.976/0.978/0.980 m。该证据的感知输入是确定性 `PoseArray`，不是 Gazebo 物理行人或
真实检测器输出；报告位于 `logs/dynamic_obstacle_navigation_ablation.json/.md`。

多目标数据关联不再按 track 遍历顺序贪心占用观测。`gated_observation_assignment.cpp` 先用运动
模型得到全部预测位置，再把“轨迹—观测”构造成带 0.5 m 门限的矩形代价矩阵；每条轨迹有一个
私有未匹配 dummy，最后用匈牙利算法求全局最小代价。固定冲突场景中，旧贪心策略只更新 1/2 条
现有轨迹并产生 1 条碎片轨迹，身份位置 RMSE 为 0.2915 m；全局策略更新 2/2 条轨迹、无碎片，
身份位置 RMSE 为 0。该结果只证明数据关联不依赖遍历顺序，不代表真实检测器准确率。

## 6. 验收命令

```bash
bash scripts/acceptance_test.sh mapping-stage
bash scripts/acceptance_test.sh slam-benchmark
bash scripts/acceptance_test.sh slam-gtsam-benchmark
bash scripts/acceptance_test.sh slam-ab-benchmark
bash scripts/acceptance_test.sh slam-navigation
bash scripts/acceptance_test.sh dynamic-obstacle-stage
bash scripts/acceptance_test.sh dynamic-obstacle-navigation
bash scripts/acceptance_test.sh dynamic-obstacle-navigation-ablation
```

其中 `mapping-stage` 适合日常提交前执行；其余会启动 Gazebo。`slam-navigation` 依赖
`slam-benchmark` 生成的 `logs/slam_ceres_map.yaml/.pgm`。

## 7. 公开数据评估层

当前已经补齐独立于 Gazebo 的轨迹评估 seam：`evaluate_slam_trajectory.py` 读取
TUM/OpenLORIS 格式，按估计时间戳插值真值，做不估计尺度的 SE(2) 对齐，再报告 ATE、
1 秒 RPE、路径长度比、最差时间窗、终点漂移和真值回访恢复率。可选 `embodied_slam_tools`
用 `rosbags` 把 ROS 1/2 bag 流式重放为 `/clock`、隔离 TF、LaserScan 和 Odometry，直接驱动
slam_toolbox；指标内核仍保持无 ROS 依赖。

```bash
bash scripts/acceptance_test.sh slam-evaluation-stage
bash scripts/acceptance_test.sh openloris-groundtruth
bash scripts/acceptance_test.sh openloris-replay-stage
OPENLORIS_RANGE_ONLY=true bash scripts/acceptance_test.sh openloris-rosbag-setup
bash scripts/acceptance_test.sh openloris-slam-ab
```

完整方法和 OpenLORIS 数据边界见
[REAL_WORLD_SLAM_EVALUATION.md](REAL_WORLD_SLAM_EVALUATION.md)。

## 8. 自动 frontier 建图到定位导航

自动建图不是“启动 SLAM 后按固定路线走一圈”。当前实现只在充电角执行一段固定、可审计且受
ActionGuard 保护的脱角原语；进入开阔区后，未知区域由 frontier 自主选点。地图估计、探索决策、
运动规划和阶段切换被分成所有权明确的模块：

```text
/scan + odom + tf
→ SLAM Toolbox 更新 /map
→ parse_mapping_bootstrap_route() 解析安全脱角原语
→ _run_agent_text_action() 经 Agent/ActionGuard/ROS 2 Action 执行
→ Explore Lite 检测 unknown/free 边界并选择 frontier
→ Nav2 NavigateToPose 规划、控制、避障
→ 无可达 frontier / 地图 plateau
→ map_saver 生成 YAML/PGM
→ 关闭 SLAM，启动 map_server + AMCL
→ map→odom ready
→ Nav2RobotExecutor 执行语义地点和巡检
```

关键代码前后关系：

| 功能 | 文件与函数 | 上游 | 下游 |
| --- | --- | --- | --- |
| 高层意图 | `showcase_session_node.py:SessionOrchestratorNode._on_asr_final()` | `/agent/asr_final` | `parse_session_command()`、`_enqueue()` |
| 状态编排 | 同文件 `_worker_loop()`、`_start_mapping()`、`_execute_request()`、`_run_automatic_mission()` | command queue | mapping/bootstrap/explorer/save/navigation 阶段 |
| 初始脱角 | `showcase_session.py:parse_mapping_bootstrap_route()`、编排器 `_run_agent_text_action()` | mission YAML 的 7 段 move/turn | ActionGuard → ROS 2 Action；完成后才启动 explorer |
| 进程生命周期 | `stage_process_manager.py:StageProcessManager.start()`、`start_explorer()`、`save_map()` | orchestrator | launch、Explore Lite、map_saver |
| 探索结束判定 | `_wait_for_frontier_completion()` | `/map`、explorer 进程和超时 | `_save_map()` |
| 定位切换 | `_start_navigation()` | 保存地图 | map_server、AMCL、Nav2 readiness |
| 语义巡检 | `_run_agent_text_action()` | mission plan 文本 | Agent→Guard→Action→Nav2 executor |

### 8.1 frontier 的核心原理

在占据栅格中，frontier 是“已知自由栅格与未知栅格的边界”。Explore Lite 聚类边界点，对候选区域
计算可达性、潜在信息收益和路径代价，并把目标作为 Nav2 Action 发送。地图增长后旧 frontier 会消失，
新 frontier 会出现，因此它是闭环重规划，不是预先写死 waypoints。项目固定第三方 commit，并由
`scripts/setup_frontier_exploration.sh` 构建，避免演示时依赖漂移。

探索 goal 不经过机器人动作 ActionGuard：它由 Explore Lite 直接交给 Nav2，受 costmap、planner、
controller、recovery、任务超时和取消约束。存图后的“去入口/巡检厨房办公室”重新进入
`RobotCommand → ActionGuard → ExecuteRobotCommand → Nav2RobotExecutor`。两条安全链不同，不能在
汇报中混为一谈。

### 8.2 与其他方案的区别

| 方案 | 负责什么 | 与当前方案的区别 |
| --- | --- | --- |
| 固定路线/teleop | 给底盘轨迹 | 可复现但不根据未知区域决策，只适合控制回归 |
| Explore Lite + SLAM Toolbox | 2D frontier 决策 + 2D 激光图优化 | 当前默认，部署轻、可解释、适合 TurtleBot3 |
| Cartographer | 子图、scan matching、pose graph | 能替换 SLAM 后端，但本身不等于自动探索任务 |
| ORB-SLAM3 | 视觉/视觉惯性位姿与地图 | 适合相机场景，仍需探索、占据地图和 Nav2 接口层 |
| Nav2 | 目标到路径/速度 | 不负责发现未知区域，也不负责保存/切换地图 |

因此“跑起 Cartographer/ORB-SLAM3”不能直接宣称完成自动建图导航；至少还要证明探索目标生成、地图
增长、地图保存、定位切换、规划结果、障碍层和最终停车。

### 8.3 完成判定与安全停止

无可达 frontier 是主要完成信号，地图已知栅格 plateau、最小已知/占用栅格和总超时用于防止第三方
节点异常时无限等待。取消事件会停止 explorer/Nav2 goal，进程管理器先温和终止进程组，超时才升级
信号。每次自动门禁还必须验证最终 `/cmd_vel=0`，避免“报告成功但机器人仍在运动”。

对应门禁：

```bash
bash scripts/acceptance_test.sh slam-autonomous-mission-stage  # 状态机和取消
bash scripts/acceptance_test.sh slam-autonomous-mission        # 真 Gazebo/frontier/SLAM/Nav2
```

## 9. 事实边界和下一步

- 已完成：仿真受控漂移、闭环建图、Ceres/GTSAM 后端、地图保存、AMCL、目标规划和预测动态避障。
- 未完成：真实传感器标定误差、轮滑与跨设备/跨序列泛化；长走廊和动态遮挡已有单序列分段证据，
  但不足以代表多环境统计结论。
- 已完成工具：OpenLORIS topic contract、ROS 1→ROS 2 SLAM 回放、map-frame 轨迹记录、
  Ceres/GTSAM A/B、ATE/RPE/回访统计和阈值门禁。
- 已完成实验：`office1-1` 接线基线与 `office1-7` 回访序列都保存 SHA256/commit/config/日志/
  轨迹 manifest；`office1-7` Ceres/GTSAM ATE 均约 10.0 cm，轨迹恢复 2/2 个回访事件，但
  accepted 非局部图边为 0，不能作为“回环前端成功”的证据。
- 已完成：玻璃/动态遮挡人工标注，以及 current-only、CV、Kalman、IMM 的跟踪器与
  Gazebo/Nav2 同场景消融。
- 已完成：真实 `office1-7` 的 6 组 accepted-edge 参数消融和可追溯 SLAM-only bag。即使将 chain
  降到 1、coarse/fine response 降到 0.05、协方差上限放到 100，46 条 accepted edge 仍全为
  相邻边，故失败边界位于 karto 候选生成/验证层，不能归因于 GTSAM。
- 已完成：`corridor1-1` 1834 节点/2751 约束固定图的 none/Huber/Cauchy 后端消融；Cauchy
  非局部边配置 ATE 1.2236 m，较 Gaussian 下降 32.90%。该结果只证明后端离群抑制，不代表
  回环前端 precision 或 recall 改善。
- 已完成：GTSAM 逐回环 switch 与两序列固定图消融；错误边可软关闭，独立序列中的正常边保持
  接近 1。在线默认关闭，尚未证明新的 Karto 约束写入可改善最终 occupancy map。
- 下一步：针对错误 closure 做可学习地点判别，扩展跨序列 lifelong/relocalization，并评估带时间维
  的局部动态障碍控制器。
