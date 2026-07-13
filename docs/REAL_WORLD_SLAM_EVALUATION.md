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
| `scripts/analyze_slam_degradation.py` | 共享同一对齐，按直行/转弯/静止及人工标注区间拆分误差 |
| `scripts/build_openloris_experiment_manifest.py` | 绑定 bag/配置/commit/指标/日志哈希 |
| `scripts/compare_openloris_backends.py` | 检查 A/B 输入可比性并描述指标差异 |
| `tests/repository/test_slam_trajectory_evaluation.py` | 数学与异常时间轴回归测试 |
| `tests/integration/test_rosbag_trajectory_adapter_runtime.py` | 可选 rosbags 真实读写测试 |

## 2. 数据集选择

首选 [OpenLORIS-Scene](https://lifelong-robotic-vision.github.io/dataset/scene.html) 的
`office1-*`：它来自真实轮式机器人，包含 2D Hokuyo LiDAR、轮式里程计和多次场景采集；官方说明
office 场景使用 OptiTrack motion capture 真值，因此适合独立评价激光 SLAM。其他场景的真值由
离线 LiDAR SLAM 生成，不应与独立 mocap 真值混为一谈。

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
# 快速取得 office1-1：只下载首个 tar 成员，约 1.25 GB。
OPENLORIS_RANGE_ONLY=true bash scripts/acceptance_test.sh openloris-rosbag-setup

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

先从真值定义回访事件：间隔超过阈值的位姿重新进入相同位置半径且朝向接近。再检查估计轨迹中
对应两帧是否也在恢复容差内。它是“真值回访恢复率”，不是 scan descriptor 的候选
precision/recall；要评价前端候选质量，还需额外记录每个候选、匹配分数和人工真值标签。

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
```

`compare_openloris_backends.py` 只在样本窗和时间覆盖可比时通过，不写死 GTSAM 或 Ceres 必须获胜。
精度阈值必须在实验前确定，并对两者使用同一 bag、前端参数、回放倍率和关联容差。

退化报告的 `straight/turning/stationary` 来自真值运动学，只回答“哪种运动状态误差更大”，
不能自动等同为“长走廊/玻璃/动态遮挡”。这些语义必须通过 `OPENLORIS_ANNOTATIONS` 提供人工
复核的时间区间；没有标注时报告会明确令 `dynamic_occlusion_evidence.available=false`。

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

当前仓库已经具备真实 office bag 的双后端回放与报告入口，但不随 Git 提交约 9 GB 的原始 bag。
在发布精度结论前，仍需由开发者固定具体序列实际跑完，并保存配置、commit、bag SHA256 和报告；
随后再围绕 `worst_segment` 对长走廊、急转和动态遮挡做参数消融。
