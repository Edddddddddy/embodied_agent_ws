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
| `scripts/evaluate_slam_trajectory.py` | 时间同步、SE(2) 对齐、指标与报告 |
| `scripts/compare_openloris_backends.py` | 检查 A/B 输入可比性并描述指标差异 |
| `tests/repository/test_slam_trajectory_evaluation.py` | 数学与异常时间轴回归测试 |
| `tests/integration/test_rosbag_trajectory_adapter_runtime.py` | 可选 rosbags 真实读写测试 |

## 2. 数据集选择

首选 [OpenLORIS-Scene](https://lifelong-robotic-vision.github.io/dataset/scene.html) 的
`office1-*`：它来自真实轮式机器人，包含 2D Hokuyo LiDAR、轮式里程计和多次场景采集；官方说明
office 场景使用 OptiTrack motion capture 真值，因此适合独立评价激光 SLAM。其他场景的真值由
离线 LiDAR SLAM 生成，不应与独立 mocap 真值混为一谈。

项目只自动下载官方 ground-truth archive，不把大型 bag 提交到 Git。原始数据下载列表见
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

输出：

```text
logs/openloris/office1-1/bag_contract.json
logs/openloris/office1-1/{ceres,gtsam}_estimate.tum
logs/openloris/office1-1/{ceres,gtsam}_report.{json,md}
logs/openloris/office1-1/{ceres,gtsam}_replay.log
logs/openloris/office1-1/backend_comparison.json
```

`compare_openloris_backends.py` 只在样本窗和时间覆盖可比时通过，不写死 GTSAM 或 Ceres 必须获胜。
精度阈值必须在实验前确定，并对两者使用同一 bag、前端参数、回放倍率和关联容差。

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
