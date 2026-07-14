# 真实/公开数据 SLAM 定量评估

## 1. 为什么单独建立评估链路

Gazebo 固定闭环能证明模块、TF 和后端求解器正确接线，但不能代表轮滑、时间同步、玻璃、动态人群
和传感器外参误差下的真实性能。本项目因此把 SLAM 评价收敛为一个独立深模块：调用者只需要提供
两条 TUM 格式轨迹，模块内部负责时间关联、插值、刚体对齐、ATE/RPE、尺度和回环统计。

```text
OpenLORIS ROS 1 bag (/odom + /scan + /tf_static)
  → rosbags 流式读取 → ROS 2 /clock + 隔离 TF + LaserScan
  → slam_toolbox（同前端，Ceres 或 GTSAM）→ map-frame estimate.tum
  → OptiTrack reference.tum → ATE/RPE/回访评估 → A/B JSON + Markdown
```

关键文件：

| 文件 | 职责 |
| --- | --- |
| `src/embodied_slam_tools/.../replay_node.py` | ROS 1/2 bag → ROS 2 clock/TF/LaserScan/Odometry |
| `src/embodied_slam_tools/.../trajectory_recorder.py` | 组合 `map→odom→base` 并输出 TUM |
| `src/embodied_slam/launch/openloris_mapping.launch.py` | recorder、slam_toolbox、replay 生命周期编排 |
| `scripts/setup_openloris_groundtruth.py` | 下载、SHA256 校验、按序列安全解压真值 |
| `scripts/setup_openloris_rosbag.py` | 断点续传、固定对象校验、只提取指定 bag、记录来源 |
| `scripts/evaluate_slam_trajectory.py` | 时间同步、SE(2) 对齐、指标与报告 |
| `scripts/analyze_openloris_revisits.py` | 从独立真值聚合回访事件，先验证序列是否适合回环实验 |
| `scripts/rank_openloris_revisit_sequences.py` | 在下载大型 bag 前批量比较方向敏感/360° LiDAR 回访与长序列门槛 |
| `scripts/evaluate_loop_constraints.py` | accepted 非局部图边的 precision、false-loop 与事件 recall |
| `scripts/extract_openloris_review_frames.py` | 稀疏提取 RGB 联络表，辅助人工标注走廊/动态遮挡区间 |
| `scripts/analyze_slam_degradation.py` | 共享同一对齐，按直行/转弯/静止及人工标注区间拆分误差 |
| `scripts/build_openloris_experiment_manifest.py` | 绑定 bag/配置/commit/指标/日志哈希 |
| `scripts/compare_openloris_backends.py` | 检查 A/B 输入可比性并描述指标差异 |
| `gtsam_graph_optimize.cpp` | 在无 ROS 回放的情况下重放同一位姿图并输出 TUM 轨迹 |
| `run_gtsam_robust_kernel_ablation.py` | 固定图 SHA256，比较 Gaussian/Huber/Cauchy 并生成报告 |
| `tests/repository/test_slam_trajectory_evaluation.py` | 数学与异常时间轴回归测试 |
| `tests/integration/test_rosbag_trajectory_adapter_runtime.py` | 可选 rosbags 真实读写测试 |

## 2. 数据集选择

首选 [OpenLORIS-Scene](https://lifelong-robotic-vision.github.io/dataset/scene.html)。office 场景使用
OptiTrack motion capture 真值，适合独立评价激光 SLAM，但当前可快速取得的 office 单序列只有
约 6 米/38 秒，不能代表长环路。market/corridor 等场景的真值由官方离线 LiDAR SLAM 生成，路径
更长但与在线 Hokuyo 输入并非完全独立；报告必须同时说明这项取舍，不能把它称为 mocap 真值。

项目可以自动下载真值和 office rosbag，但 `datasets/` 被 Git 忽略，不会把大型数据提交到仓库。
原始数据下载列表见
[OpenLORIS download index](https://github.com/lifelong-robotic-vision/OpenLORIS-Scene/blob/master/download.md)，
官方评价脚本见
[openloris-scene-tools](https://github.com/lifelong-robotic-vision/openloris-scene-tools)。使用数据前应再次
阅读官方许可和引用要求。

```bash
bash scripts/acceptance_test.sh openloris-groundtruth
# datasets/openloris/groundtruth/office1-1/groundtruth.txt
# datasets/openloris/groundtruth/office1-1/source.json
```

大型 bag 下载前先批量扫描真值 ZIP。排名同时计算相机式“同位置且同朝向”与 360° LiDAR
“同位置、允许反向通过”两种口径，并要求路径至少 100 米、时长至少 120 秒、回访两端至少相隔
60 秒。轨迹层第一名 `market1-3` 有 1 次同向长回访，但完整 bag 缺少 `/scan`，不能进入当前
slam_toolbox 2D 管线。经过 `/odom`、`/scan`、`/tf_static` 全量传感器契约后，正式推荐为
`corridor1-1`：272.49 秒、220.06 米、2 次位置回访；其 yaw 差约 129°～180°，因此报告明确使用
`position_only_360_lidar`，不把反向穿越伪装成同向视觉回环。

```bash
bash scripts/acceptance_test.sh openloris-sequence-ranking
```

下载器固定校验：

```text
SHA256 07564d7ed3d6739585002afa12bcf481cc0e9e358fc64efd5e658e2c994bdc3b
```

归档更新时不能静默跳过校验；需要人工核对官方来源后再更新常量和文档。

office rosbag 归档约 9.27 GB，当前固定来源对象：

```text
size   9267230720 bytes
sha256 f43ae38cd560e150d8a06bbac527235dc53a536a07bdea77db7e672c13875912
commit cbc03108723d08322b23d0338680bffa9404cce9
```

```bash
# 快速取得 office1-1：约 1.25 GB。
OPENLORIS_RANGE_ONLY=true bash scripts/acceptance_test.sh openloris-rosbag-setup

# 直接取得含回访事件的 office1-7：约 1.43 GB，不下载前六个成员。
OPENLORIS_SEQUENCE=office1-7 OPENLORIS_RANGE_ONLY=true \
  bash scripts/acceptance_test.sh openloris-rosbag-setup

# 完整来源审计：下载并校验整个 9.27 GB 归档。
bash scripts/acceptance_test.sh openloris-rosbag-setup
```

完整模式写入 `.part`；快速模式使用 8 个可独立续传的 `.parts`，遇到 CDN 提前 EOF 时只补缺失
尾段。HTTP 206 和 `Content-Range` 必须匹配请求才允许追加，合并大小和 SHA256 通过后才原子
发布；解包不调用
通用 `extract()`，只复制 basename 精确匹配的普通 `.bag`，拒绝绝对路径、`..` 和链接成员。

快速模式固定数据集 commit、HTTP Range、首成员名称与长度，并校验以下 range/bag SHA256；它没有
读取完整归档，因此 `source.json.archive_verification` 明确写成
`pinned_https_range_not_full_hash`。完整模式才写 `full_size_and_sha256`。实验 manifest 会继续
携带这个差别，防止把“快速可复现”误写成“完整归档已校验”。

```text
office1-1 range sha256 c637329c32caa00561419cdce8d04bc7e659cf5edcdc3a6b8f220791210f3c52
office1-1 bag   sha256 d18e335a34dc25b6f26df911886bdefbeeeacd41620c0ea525fe0314ea9f7a65
office1-7 range sha256 da2abbacc4a4c200890c8128186b677d0c3b0a7de6a66a2d7095c656ff0e4b6b
office1-7 bag   sha256 23f443c33353e4059b5d108ca099d452550954613b8af1b02e8159fb147af3ca
corridor1-1 range sha256 14aed071ea7198d47bd14ea992ce5e46dd17b128766ef77b953cc4a50d3744da
corridor1-1 bag   sha256 a373fb24539561ee6a8900c91603baeedf8b739881a7b04860ed5e363dc93a22
corridor1-1 bag   size   11227075960 bytes
```

`corridor1-1` 下载器按固定数据集 commit 和 tar 成员边界并行 Range 续传；先校验区间摘要，再
流式解出唯一 bag 并校验内容摘要，最后原子发布。正式入口随后只复制 `/odom`、`/scan`、
`/tf_static` 生成 provenance-bound 轻量 bag，避免每次参数实验重复扫描 RGB/Depth 数据。
`market1-3` 的完整对象摘要仍保留在下载器中，作为“真值排名不能替代传感器契约”的负例：

```bash
bash scripts/acceptance_test.sh openloris-long-loop-evidence
```

## 3. rosbag 回放 Adapter

OpenLORIS 使用 ROS 1 bag，而主项目运行 ROS 2 Jazzy。可选
[rosbags](https://gitlab.com/ternaris/rosbags) 能同时读取 ROS 1/2，避免为回放安装完整 ROS 1：

```bash
source scripts/activate.sh
pip install -r requirements-slam-eval.txt

bash scripts/acceptance_test.sh openloris-replay-stage
```

Adapter 的关键处理：

- `FrameMapper` 把数据集 frame 放进 `dataset_*` 独立树，避免与宿主机器人 TF 冲突。
- `ReplayTimeline` 保证 `/clock` 单调并按倍率节流，不让乱序写入破坏 TF buffer。
- `TimestampDeduplicator` 过滤旧 office bag 已知的重复 `/odom` 时间戳。
- 回放节点显式广播 `dataset_odom→dataset_base_link`；发布 Odometry 本身不会生成 TF。
- recorder 只在收到 `map→dataset_odom` 时组合完整 SE(2) 链，逐样本 flush，异常退出也保留证据。

小 fixture 只验证接口、时钟、TF、重复帧和双后端能启动，不代表真实场景精度。真实 bag 必须先过
`openloris-bag-preflight`，缺 `/scan`、`/odom` 或静态外参时直接失败，而不是猜测 TF。

## 4. 指标定义和工程取舍

### 时间关联

在每个估计时间戳上插值参考位姿。参考采样洞超过 `2 * max_time_diff` 时拒绝插值，防止在数据断流
区间生成看似平滑的低误差。报告同时给出估计匹配率和参考时间覆盖率。

### SE(2) 对齐

二维最小二乘只求一个全局旋转和平移，尺度固定为 1。这样能消除 `map` 坐标系原点差异，但保留
轮径、里程计尺度或 SLAM 尺度漂移。使用 Sim(2) 对齐会让尺度错误“消失”，不适合本项目的
轮式机器人评价目的。

### ATE 与 RPE

- ATE XY：对齐后每个位姿的绝对平移误差，报告 RMSE、均值、中位数、P95 和最大值；回答全局
  地图是否一致。
- RPE：默认比较相隔 1 秒的相对运动，报告平移与 yaw；回答局部里程计/scan matching 是否稳定。
- `path.length_ratio`：估计路径长度 / 参考路径长度，用于暴露尺度偏差。
- `worst_segment`：固定时间窗内最高 ATE RMSE，用于定位长走廊、急转或动态干扰退化段。

### 回环指标

先从真值定义回访事件：间隔超过阈值的位姿重新进入相同位置半径且朝向接近。方向敏感策略适合
保守的同向重访评价；`position_only_360_lidar` 允许 180°，只用于说明 360° 激光的地点重叠，
必须和传感器 profile 一起声明，不能与相机式同向指标直接比较。再检查估计轨迹中
对应两帧是否也在恢复容差内。它是“真值回访恢复率”，不是 scan descriptor 的候选
precision/recall；要评价前端候选质量，还需额外记录每个候选、匹配分数和人工真值标签。

本项目进一步在 GTSAM `ScanSolver::AddConstraint` Adapter 记录前端已经接受的全部图边，并用
Karto 原生 `end_closure` callback 的 scan id 选出真实闭环边。仅按 node id 间隔分类会把大量局部
图连接误当成 loop，已不用于正式指标。边是否正确通过“测得相对位姿 vs 真值相对位姿”的 SE(2)
残差判定（当前 0.75 m / 15°）；端点是否回到 1 m 内只用于真值事件 recall，不能替代约束正确性。
该 API 看不到 scan matcher 已拒绝的所有候选，因此报告仍限定为“accepted constraint 质量”，
不声称完成了全候选阈值曲线。

## 5. 使用方法

无外部数据的算法门禁：

```bash
bash scripts/acceptance_test.sh slam-evaluation-stage
```

它生成同一条闭环的纯里程计漂移和回环校正轨迹，要求校正后的 ATE、RPE 和终点漂移同时下降。
该结果只证明评估器和比较门禁，不代表真实模型精度。

公开序列先验收 contract，再选择单后端或 A/B：

```bash
OPENLORIS_BAG=/data/openloris/office1-1.bag \
  bash scripts/acceptance_test.sh openloris-bag-preflight

OPENLORIS_BAG=/data/openloris/office1-1.bag \
OPENLORIS_SEQUENCE=office1-1 \
OPENLORIS_REPLAY_RATE=1.0 \
  bash scripts/acceptance_test.sh openloris-slam-ab

# 推荐的回访证据入口：默认 office1-7。
bash scripts/acceptance_test.sh openloris-loop-evidence
```

若使用默认 `datasets/openloris`，完成 setup 后可省略 `OPENLORIS_BAG`。每个后端结束时还会生成
manifest，要求来源 `source.json`、bag contract、求解器配置、轨迹、评价、退化报告和干净退出日志
同时成立；生成时会重新计算当前 bag SHA256，而不只检查来源文件里的字符串。这样报告离开当前
机器后仍能回答“用的哪份数据、哪次提交、哪组参数”。

输出：

```text
logs/openloris/office1-1/bag_contract.json
logs/openloris/office1-1/{ceres,gtsam}_estimate.tum
logs/openloris/office1-1/{ceres,gtsam}_report.{json,md}
logs/openloris/office1-1/{ceres,gtsam}_degradation.{json,md}
logs/openloris/office1-1/{ceres,gtsam}_replay.log
logs/openloris/office1-1/{ceres,gtsam}_manifest.json
logs/openloris/office1-1/backend_comparison.json
logs/openloris/office1-7/revisit_catalog.json
logs/openloris/office1-7/gtsam_constraints.jsonl
logs/openloris/office1-7/gtsam_loop_constraints.json
logs/openloris/corridor1-1/gtsam_frontend.jsonl
logs/openloris/corridor1-1/gtsam_loop_constraints.json
logs/openloris/corridor1-1/gtsam_manifest.json
```

`compare_openloris_backends.py` 只在样本窗和时间覆盖可比时通过，不写死 GTSAM 或 Ceres 必须获胜。
精度阈值必须在实验前确定，并对两者使用同一 bag、前端参数、回放倍率和关联容差。

退化报告的 `straight/turning/stationary` 来自真值运动学，只回答“哪种运动状态误差更大”，
不能自动等同为“长走廊/玻璃/动态遮挡”。这些语义必须通过 `OPENLORIS_ANNOTATIONS` 提供人工
复核的时间区间；没有标注时报告会明确令 `dynamic_occlusion_evidence.available=false`。

原始 office bag 同时含高带宽 RGB/Depth。即使 `rosbags` 只反序列化 SLAM topic，也仍可能需要
扫描和解压包含图像的数据块，所以 38 秒 bag 的墙钟回放可达数分钟。后续性能优化应先生成只含
`/scan`、`/odom`、`/tf_static` 的派生轻量 bag，并在 manifest 中绑定原包与派生包哈希。

### 5.1 当前真实运行结果

2026-07-13 在 `office1-1`（26.999 秒，bag SHA256 如上）以 1× 回放得到：

| 指标 | Ceres | GTSAM |
| --- | ---: | ---: |
| 匹配姿态 / 时间覆盖 | 322 / 99.6487% | 322 / 99.6487% |
| ATE XY RMSE | 0.028794 m | 0.028899 m |
| 1 s RPE 平移 RMSE | 0.014806 m | 0.014753 m |
| 终点姿态位置误差 | 0.040168 m | 0.039983 m |
| 直行 ATE RMSE | 0.030012 m | 0.029998 m |
| 转弯 ATE RMSE | 0.021770 m | 0.022921 m |

ATE 差值只有 0.105 mm，按比较器预设 1% 容差判定为平局。该短序列的真值轨迹没有满足定义的
回访机会，`loop.recall=null`；没有人工动态遮挡标注，相关证据也为 unavailable。因此可以讲
“真实 bag 回放与精度评价已闭环”，不能讲“已在此序列证明回环或动态遮挡优化有效”。

### 5.2 `office1-7` 短时回访与 accepted-edge 实测

2026-07-14 使用固定 range/bag SHA256 对 `office1-7` 完成同前端 A/B：

| 指标 | Ceres | GTSAM |
| --- | ---: | ---: |
| 匹配位姿 / 时间覆盖 | 449 / 99.753% | 449 / 99.753% |
| ATE XY RMSE | 0.099962 m | 0.099885 m |
| 1 s RPE 平移 RMSE | 0.055384 m | 0.055483 m |
| 回访采样恢复 | 4 / 5 | 4 / 5 |
| 回访事件恢复 | 2 / 2 | 2 / 2 |
| glass partition 区间 ATE | 0.106098 m | 0.106386 m |
| dynamic occlusion 区间 ATE | 0.097820 m | 0.097419 m |

ATE 差值 0.077 mm，比较器仍判定平局。这张旧表使用最少相隔 10 秒的宽松回访定义；按正式长环路
采用的 60 秒门槛，`office1-7` 没有真值事件，因此“两次”不能再表述为独立长回环。GTSAM Adapter
实际记录到的 46 条 accepted graph edges 也全部为 ID 相邻边，非局部 accepted loop 数为 0。
当前只能讲“轨迹在短路径内保持几何一致”，不能讲“回环前端检测成功”。

### 5.3 前端 accepted-edge 阈值消融

运行：

```bash
bash scripts/acceptance_test.sh openloris-loop-sweep
```

runner 先从固定 SHA256 的 1.43 GB 原包流式复制 `/odom`、`/scan`、`/tf_static`，得到约 5 MB
SLAM-only ROS 1 bag；派生 `source.json` 绑定原包/派生包哈希、760 条 odom、1524 帧 scan、1 条
static TF 和 38.52 秒时间窗。六组实验固定派生 bag、GTSAM 后端和评估器，只改变声明过的前端参数：

| profile | 主要变化 | 匹配位姿 | accepted 非局部边 | event recall | ATE RMSE | wall clock |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| baseline | 当前配置 | 449 | 0 | 0 | 0.099840 m | 52 s |
| short_chain | chain 8→5 | 449 | 0 | 0 | 0.099576 m | 49 s |
| lower_response | coarse/fine 0.30/0.40→0.20/0.30 | 449 | 0 | 0 | 0.099806 m | 49 s |
| wider_search | neighbour 4→6 m，grid 8→10 m | 449 | 0 | 0 | 0.099800 m | 49 s |
| permissive_combined | 上述组合 | 449 | 0 | 0 | 0.099891 m | 49 s |
| diagnostic_extreme | chain=1，response=0.05，variance=100 | 449 | 0 | 0 | 0.100133 m | 52 s |

所有 profile 的约束日志都只有 46 条相邻边，最大 node ID separation 为 1。诊断性 extreme 也未
产生非局部约束，因此不能再把问题归因于 GTSAM 求解器或单纯的 response 阈值。新增的
`instrumented_async_slam_toolbox_node` 在不修改 Karto 决策的前提下，为每个真正入图的 scan 输出
候选拓扑 JSONL，并通过 `MapperLoopClosureListener` 记录 coarse/fine response、variance、reject
和 closure callback。baseline 共覆盖 47 个图节点：8 次 `insufficient_history`，39 次
`all_geometric_neighbors_near_linked`，没有候选送入 coarse matcher。失败边界因此被定位到
near-linked 排除/候选生成层。由于 Karto 候选函数是 private，候选链为同规则复算而不是私有函数
执行 trace；accepted edge 仍以 `ScanSolver` 日志为准。extreme 只用于定位边界，绝不自动替换
生产配置。

语义区间来自每 3 秒抽取的 D400 RGB 联络表人工复核：18–24 秒附近标为玻璃隔断，27–30 秒可见
移动人员穿越并近距离遮挡，因此标为动态遮挡。画面没有足够证据支持“长走廊”，配置文件显式记录
negative evidence，未为了凑指标虚构 corridor 标签；视觉标签也不等价于逐束 LiDAR 遮挡真值。

### 5.4 `corridor1-1` 长走廊与真实回访结果

2026-07-14 使用固定 range/bag SHA256、2× 回放和 GTSAM 前端/后端完成 284.15 秒传感器覆盖。真值
有效区间为 272.49 秒、路径 220.06 米，按 `position_only_360_lidar` 有 2 个至少相隔 60 秒的
位置回访事件：

| 指标 | 实测结果 |
| --- | ---: |
| 估计匹配 / 参考覆盖 | 3263 / 99.9743% |
| ATE XY RMSE / P95 | 1.675949 m / 4.539156 m |
| 1 s RPE 平移 RMSE | 0.337416 m |
| 路径长度比 | 1.016358 |
| 最终位置误差 | 4.445675 m |
| 轨迹几何回访事件恢复 | 2 / 2 |
| Karto 原生 closure callback | 8 |
| 真值覆盖内可评价 closure | 7（1 正确、6 错误） |
| accepted-edge precision | 14.2857% |
| accepted-edge 长回访事件恢复 | 0 / 2 |

这组结果不能包装成“回环优化成功”。它证明了更有面试价值的失败边界：最终轨迹仍能在两个事件
中回到相近位置，但 Karto 真正接受的闭环边没有恢复正式长回访；7 条可评价 closure 中只有 1 条
相对位姿残差通过 0.75 m / 15° 门槛。另 1 条 closure 的一端早于官方真值起点，被报告为
`outside_reference_coverage`，不强行判断真假。长走廊后段 ATE RMSE 约 2.10 m，
230 秒附近人工标注的行人区间约 0.97 m；两者只是时间区间统计，不能据此断言行人造成漂移。
相同 bag/参数的一次先行回放只产生 4 次 closure，说明异步前端仍有运行间波动；当前报告记录的是
随后通过完整正式入口生成的 8 次结果，后续应增加多次重复统计，而不是只挑最好的一次。

### 5.5 固定位姿图鲁棒核消融

直接为每个鲁棒核重新回放 bag 并不公平：异步前端每轮接受的 closure 会变化。本项目因此让
`GtsamScanSolver` 在不改变在线求解工作集的前提下，按 node ID 和 edge key 去重累计证据图；
`gtsam_graph_optimize` 再读取同一份快照离线求解。比较器要求 graph SHA256、节点数、约束数和
真值匹配位姿数完全一致，否则拒绝报告：

```bash
bash scripts/acceptance_test.sh openloris-long-loop-evidence
bash scripts/acceptance_test.sh openloris-robust-kernel-ablation
```

2026-07-14 的固定图 SHA256 为
`792ecb7d1871b506274423631fdf16da60b52e46aa18eaa1287cddbd4cadbc65`，包含 1834 个连续节点、
2751 条去重约束、0 条悬空边；四组都匹配 1828 个真值位姿：

| variant | 鲁棒范围 | ATE RMSE | ATE P95 | 1 s RPE RMSE | 终点误差 |
| --- | --- | ---: | ---: | ---: | ---: |
| Gaussian | 无 | 1.8235 m | 4.7794 m | 0.2103 m | 5.4486 m |
| Huber all | 全部 2751 条边 | 1.4081 m | 3.1571 m | 0.1784 m | 3.6858 m |
| Huber non-local | 858 条非局部边 | 1.4626 m | 3.3582 m | 0.1740 m | 3.9235 m |
| Cauchy non-local | 858 条非局部边 | **1.2236 m** | **2.3968 m** | **0.1595 m** | **2.4741 m** |

Cauchy 相比 Gaussian 的 ATE 下降 32.90%，说明重尾损失能降低已接受离群边对全局解的影响。
但 `non-local` 使用 node ID 间隔 ≥20 的后端启发式，其中不保证每条边都是 Karto 原生 closure；
正式回环 precision 仍由 closure callback + 真值相对位姿残差计算。鲁棒核不能召回漏检回环，
也不能把错误前端变成正确前端。可提交的小型结果见
[`docs/evidence/gtsam_robust_kernel_ablation.md`](evidence/gtsam_robust_kernel_ablation.md)。

## 6. 面试讲法和事实边界

可以讲：

- 仿真中注入可复现漂移，自己实现 GTSAM `karto::ScanSolver` Adapter，并与 Ceres 同前端 A/B。
- 真实数据评估层采用独立真值、时间同步、固定尺度 SE(2) 对齐、ATE/RPE 和退化时间窗。
- ROS 1 数据通过流式 Adapter 直接驱动 ROS 2 SLAM；核心评价数学仍不依赖 ROS，便于 CI 单测。

不能讲：

- 只下载了数据或 self-evaluation 就宣称真实场景精度已经通过。
- 把非 office 场景的离线 SLAM 真值说成独立 mocap 真值。
- 把回访恢复率说成回环前端 precision/recall。
- 在没有相同数据和阈值时，笼统宣称 GTSAM 优于 Ceres。

当前仓库已经具备真实 office bag 的双后端回放、长走廊回访序列、来源 manifest、人工退化区间、
SLAM-only 派生包、accepted-edge 参数消融和 Karto 候选级 instrumentation，但不随 Git 提交大型
bag/实验结果。鲁棒核固定图消融已经完成；下一步应针对 `corridor1-1` 的错误 closure 做前端感知
混淆抑制，再扩展到跨序列 lifelong/relocalization。动态障碍预测的 current-only、CV、Kalman、
IMM 同场景消融已另行完成。
