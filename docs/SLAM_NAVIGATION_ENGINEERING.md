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

## 2. 关键文件与代码流程

| 技术点 | 关键文件/函数 | 设计理由 |
| --- | --- | --- |
| 受控里程计漂移 | `src/embodied_slam/src/drift_model.cpp`：`DriftModel::update/reset` | 尺度误差、每米航向偏置和高斯噪声均可配置；固定 seed 让两个后端收到可重复输入 |
| 漂移 TF/传感器隔离 | `odom_drift_injector_node.cpp`：`on_odometry/on_scan` | 创建 `slam_odom -> slam_base_link -> slam_laser` 独立 TF 树，不污染 Gazebo 参考 `/odom` |
| 固定闭环路线 | `closed_loop_controller.cpp`、`closed_loop_driver_node.cpp`：`update/step` | 同一四边形路线用于 Ceres/GTSAM A/B；雷达近障停车仍保留 |
| Ceres 基线 | `config/slam_mapping_ceres.yaml` | 使用 slam_toolbox 官方默认支持的 Ceres + Huber，作为稳定参照 |
| GTSAM 深模块 | `gtsam_pose_graph.cpp`：`GtsamPoseGraphOptimizer::optimize` | 用纯 Pose2/constraint 接口隔离 GTSAM，能脱离 ROS/karto 做单元测试 |
| ScanSolver Adapter | `gtsam_scan_solver.cpp`：`AddNode/AddConstraint/Compute` | 将 karto 节点、相对位姿和协方差适配为 GTSAM Prior/Between factors，通过 pluginlib 注入 slam_toolbox |
| 协方差防护 | `make_positive_definite` | 对称化协方差并钳制特征值，防止走廊等退化几何给出奇异矩阵导致求解器崩溃 |
| 地图/轨迹报告 | `tests/integration/test_slam_mapping_baseline.py`：`build_report` | 同时统计原始 ATE、闭环误差、`map->odom` 校正轨迹和已知地图面积 |
| 后端 A/B | `scripts/compare_slam_backends.py`：`compare` | 检查两次路线与漂移尺度一致，再比较校正 ATE、闭环误差、覆盖面积和时间 |
| 公开 bag 回放 | `bag_source.py`：`inspect_bag/iter_events`；`replay_node.py`：`replay` | 懒加载 ROS 1/2 消息，发布单调 `/clock`、隔离 TF 与 scan；不把 9 GB bag 读入内存 |
| 轨迹记录 | `trajectory_recorder.py`：`_on_tf/_record` | 显式组合 `map→odom→base`，逐样本 flush，避免只记录后端校正量或退出丢证据 |
| 真实后端 A/B | `compare_openloris_backends.py`：`compare` | 先验证样本窗/覆盖率可比，再描述 ATE/RPE 差异，不预设某个求解器必胜 |
| 地图复用 | `localization_navigation.launch.py` | 关闭 SLAM，加载保存的 YAML/PGM，启动官方 Nav2/AMCL 生命周期栈 |
| 定位规划验收 | `test_slam_localization_navigation.py` | 明确等待 `map->odom` 和 BT Navigator ACTIVE，再检查 plan、Action result、odom 与零速 |
| 动态目标跟踪 | `dynamic_obstacle_tracker.cpp`：`DynamicObstacleTracker::update` | 对标准 `PoseArray` 检测做最近邻关联、常速度估计、指数平滑、置信度累积和超时淘汰 |
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

首节点 Prior factor 消除整张图可任意平移/旋转的规范自由度；Huber 核 `ρ` 降低错误
回环的影响。优化结果不会重写轮速里程计，而是更新 `map->odom`，所以局部控制仍保持
连续，地图和全局位姿则被整体校正。

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
ATE 与闭环误差，避免只凭 RViz 截图判断。当前还没有带人工回环标注的真实数据，因此尚不能
给出 loop precision/recall；这项边界必须在面试中主动说明。

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

这里只采用可解释的常速度模型，不把它包装成复杂学习算法。优势是 CPU 开销小、参数可解释、
可独立单测；缺点是急转、急停和多人交叉时预测误差大。相比只把当前检测点写入 obstacle layer，
预测层能在行人尚未走到机器人直线路径前提前让路。相比 TEB/MPPI 内部的时空轨迹优化，当前
实现作用在全局二维代价地图，接入简单但时间维被压平；下一步可以把 track 送入支持时空障碍
的局部控制器，或改为 Kalman/IMM 预测并做消融实验。

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

## 6. 验收命令

```bash
bash scripts/acceptance_test.sh mapping-stage
bash scripts/acceptance_test.sh slam-benchmark
bash scripts/acceptance_test.sh slam-gtsam-benchmark
bash scripts/acceptance_test.sh slam-ab-benchmark
bash scripts/acceptance_test.sh slam-navigation
bash scripts/acceptance_test.sh dynamic-obstacle-stage
bash scripts/acceptance_test.sh dynamic-obstacle-navigation
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
OPENLORIS_BAG=/data/openloris/office1-1.bag bash scripts/acceptance_test.sh openloris-slam-ab
```

完整方法和 OpenLORIS 数据边界见
[REAL_WORLD_SLAM_EVALUATION.md](REAL_WORLD_SLAM_EVALUATION.md)。

## 8. 事实边界和下一步

- 已完成：仿真受控漂移、闭环建图、Ceres/GTSAM 后端、地图保存、AMCL、目标规划和预测动态避障。
- 未完成：真实传感器标定误差、轮滑/玻璃/长走廊等真实退化数据的系统评测。
- 已完成工具：OpenLORIS topic contract、ROS 1→ROS 2 SLAM 回放、map-frame 轨迹记录、
  Ceres/GTSAM A/B、ATE/RPE/回访统计和阈值门禁。
- 尚未完成实验：还没有提交 OpenLORIS 完整 bag 的 Ceres/GTSAM 实际回放报告；下载真值或
  对真值做 self-evaluation 不能代替这项证据。
- 下一步：固定 OpenLORIS office 序列实际跑完并保存 bag SHA256/commit/report；再比较动态障碍
  current-only 与 constant-velocity prediction，并引入 Kalman/IMM 做消融。
