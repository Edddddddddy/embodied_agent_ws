# 真实感语音 SLAM 与语义导航演示

## 1. 演示目标

这条主演示在一个 10 m × 8 m 的公寓/办公室混合场景中完成：

```text
真实麦克风
  → 在线/离线 ASR
  → 中文命令 NLU/LLM
  → typed RobotCommand / ExecuteRobotCommand
  → C++ ActionGuard 与 ActionScheduler
  → GazeboRobotExecutor（建图阶段的速度动作）
  → TurtleBot3 + LaserScan + Odometry
  → SLAM Toolbox 在线 OccupancyGrid
  → map_saver 保存 YAML/PGM
  → AMCL 定位
  → Nav2RobotExecutor + NavigateToPose
  → 去厨房/办公室/会议区或依次巡航
```

场景包含客厅、厨房、办公室、会议区、走廊、墙体、门洞、桌椅、沙发、柜体、植物和充电桩。
全部几何由项目内 primitive SDF 生成，不在演示时下载 Fuel 资源，因此离线可复现，也不会因模型
版本或网络波动导致现场启动失败。

## 2. 为什么用“一份清单生成三类资产”

唯一人工维护源是：

- `src/embodied_simulation/config/showcase_apartment.yaml`

`scripts/generate_showcase_scene.py` 从它确定性生成：

- `worlds/showcase_apartment.sdf.xacro`：Gazebo 视觉与碰撞几何；
- `maps/showcase_apartment.yaml/.pgm`：现场保底用静态地图；
- `maps/showcase_apartment_slam_frame.yaml`：自动门禁用的完整 SLAM 坐标系地图；
- `config/showcase_places.yaml`：静态地图/Gazebo 世界坐标下的语义地点；
- `config/showcase_mapping_places.yaml`：以建图起点为原点的语义地点。

这样避免了“修改墙体后忘记改地图”和“地图正确但目标点落在家具里”两类演示故障。仓库测试还会
按 TurtleBot3 膨胀半径验证所有地点处于同一可达自由空间。

两套地点表不是重复配置。项目静态地图的原点是世界左下角，而 SLAM Toolbox 的 `map` 坐标通常
以建图开始时的机器人为原点。生成器对后者执行：

```text
slam_place.x = world_place.x - spawn.x
slam_place.y = world_place.y - spawn.y
```

因此语音建图后重启机器人时发布 `(0, 0, 0)` 初始位姿，`去厨房` 不会整体偏移一个出生点。
脚本还把 Gazebo spawn 与 AMCL initial pose 拆成两组参数：重启时机器人仍在世界坐标
`(-4.15, -3.15)` 生成，而保存地图中的定位初值是 `(0, 0)`；不能把其中一组坐标同时传给两层。

## 3. 首次准备与轻量检查

```bash
cd ~/embodied_agent_ws
source scripts/activate.sh
python3 scripts/generate_showcase_scene.py --check
bash scripts/acceptance_test.sh slam-nav-showcase-stage
```

建议先确认麦克风：

```bash
bash scripts/acceptance_test.sh wsl-microphone-preflight
```

## 4. 三阶段人工演示

### 4.1 语音探索和在线建图

Terminal 1：

```bash
HEADLESS=false USE_RVIZ=true \
  bash scripts/voice_slam_nav_showcase.sh mapping offline
```

推荐沿房间外圈和门洞探索，并让雷达至少观察每个目标房间。可说：

```text
小智
前进两秒
左转九十度
前进三秒
右转九十度
停下
```

建图阶段故意选择 `GazeboRobotExecutor`：`move/turn/stop` 直接控制底盘，SLAM 只负责估计与建图；
此时使用 `Nav2RobotExecutor` 会形成“还没有完整地图，却要求 Nav2 先规划探索动作”的循环依赖。

### 4.2 保存地图

保持 Terminal 1 运行，在 Terminal 2 执行：

```bash
cd ~/embodied_agent_ws
bash scripts/voice_slam_nav_showcase.sh save
```

确认 `logs/showcase/voice_built_map.yaml` 和 `.pgm` 非空后，在 Terminal 1 按 `Ctrl+C`。

### 4.3 定位与语义导航

Terminal 1：

```bash
bash scripts/voice_slam_nav_showcase.sh navigation offline
```

推荐话术：

```text
小智，去厨房
去办公室
依次去入口、会议区、充电区
取消导航
```

这一阶段关闭 SLAM，加载刚保存的地图；AMCL 提供 `map→odom`，Nav2 完成全局规划、局部控制、
障碍层检查和 Action 反馈。`Nav2RobotExecutor` 会把语义地点解析为 `NavigateToPose`，巡航命令则
按顺序发送多个目标，失败或取消后必须停车。

## 5. 自动门禁和 PASS 标准

```bash
# 场景生成、坐标对齐、NLU 和脚本契约
bash scripts/acceptance_test.sh slam-nav-showcase-stage

# 真实场景 → SLAM /map → typed 移动 → map_saver
bash scripts/acceptance_test.sh slam-nav-showcase-mapping

# 文本模拟 ASR → Agent → typed Action → AMCL/Nav2 → odom 运动
bash scripts/acceptance_test.sh slam-nav-showcase
```

完整人工演示的 PASS 条件：

1. 建图时 `/map` 发布，语音动作使 `/odom` 改变；
2. 保存得到可加载的 YAML/PGM；
3. 导航重启后出现 `map→odom`；
4. 至少两个语义目标产生规划并成功到达；
5. 结束、失败和取消后 `/cmd_vel` 均归零。

自动导航门禁用文本 topic 代替真人发声，以保持 CI/本地回归可重复；人工三阶段验收才是麦克风证据，
两者会复用完全相同的 Agent、typed Action、ActionGuard 和 Nav2 executor 下游链路。
其中自动门禁的 `showcase_apartment_slam_frame.yaml` 是同源静态栅格经坐标变换得到的“充分探索”
替身，用来稳定验证重载坐标、AMCL 和控制链路；它不是本次运行由 SLAM Toolbox 在线生成的地图。

## 6. 现场保底与事实边界

如果现场时间不足，或语音探索未覆盖所有房间，可使用：

```bash
bash scripts/voice_slam_nav_showcase.sh navigation-static offline
```

它加载与 Gazebo 几何同源生成的确定性地图，适合稳定展示语音语义导航，但不能宣称该地图由本次
语音探索建立。当前 V1 的阶段切换和保存仍由第二终端命令触发；语音负责探索动作和导航目标。
后续可增加 typed `SlamSession` Action，把“保存地图/开始导航”也纳入可反馈、可取消的语音工作流。

本演示证明的是 Gazebo 仿真内的语音—SLAM—Nav2 闭环，不等于真实传感器标定、轮滑、玻璃反射、
动态人群和长期定位稳定性已经解决。真实环境漂移与回环质量仍应使用公开或自采 rosbag 的
ATE/RPE、回环 precision/recall 证据评估。
