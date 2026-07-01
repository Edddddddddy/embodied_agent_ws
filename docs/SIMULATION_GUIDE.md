# 语音控制 TurtleBot3 仿真指南

本模块使用 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 和 TurtleBot3 Burger。Jazzy 与 Harmonic 是当前环境的官方二进制组合；不需要为了参考项目降级到 Ubuntu 22.04/Humble。

## 1. 完整数据流

```text
麦克风或测试 PCM
  -> 在线/离线 ASR
  -> LLM + 确定性命令仲裁
  -> /agent/action_candidate
  -> C++ ActionGuard（schema、白名单、限幅）
  -> /robot/action_command
  -> C++ SimulationController
       |-> /scan 雷达安全与行为模式
       `-> /cmd_vel geometry_msgs/Twist
  -> ros_gz_bridge
  -> Gazebo DiffDrive
  -> /robot/action_ack + /odom、/tf、/scan
```

仿真控制器不订阅模型的原始输出，只接受 Guard 验证后的动作。实体硬件控制器和仿真控制器是 `/robot/action_command` 的两个可替换执行后端；仿真 launch 会关闭硬件后端，避免同一动作被重复执行。

## 2. 关键代码

| 文件 | 职责 |
|---|---|
| `src/embodied_simulation/include/embodied_simulation/simulation_controller.hpp` | 与 ROS 无关的行为控制接口、模式和配置 |
| `src/embodied_simulation/src/simulation_controller.cpp` | 手动定时运动、速度平滑、避障、沿墙 PID、雷达超时和急停 |
| `src/embodied_simulation/src/simulation_control_node.cpp` | 动作 JSON、LaserScan、仿真 ACK、模式 topic 与 Twist 的 ROS seam |
| `src/embodied_simulation/config/simulation_control.yaml` | 速度、加速度、安全距离、墙距和 PID 参数 |
| `src/embodied_simulation/config/turtlebot3_bridge.yaml` | ROS 与 Gazebo 的 clock/odom/tf/cmd_vel/scan 桥接 |
| `src/embodied_simulation/launch/voice_turtlebot3.launch.py` | Gazebo、TurtleBot3、bridge、Agent、Guard、控制器和可选 RViz 总启动 |
| `src/embodied_simulation/test/test_simulation_controller.cpp` | 纯 C++ 行为和安全单元测试 |

## 3. 控制模式

### manual

语音 `move/turn/stop` 会自动切回手动模式。运动带持续时间，过期后速度回到 0；速度和角速度在 C++ Guard 与仿真控制器两处限幅。

示例：

- “向前走一秒”
- “后退半秒”
- “左转九十度”
- “马上停下”

### obstacle_avoidance

前方距离大于 `obstacle_distance` 时低速前进；遇到障碍时比较左右可用空间并原地转向。语音“开启自动避障”生成：

```json
{"name":"set_mode","arguments":{"mode":"obstacle_avoidance"}}
```

### wall_following

右侧雷达距离与 `wall_target_distance` 的误差进入 PID：右侧太近向左修正，太远向右修正；前方遇到墙角时优先左转。语音“开始沿墙行走”切换该模式。

### 安全优先级

1. `/robot/emergency_stop`：立即清零、切回 manual；恢复运动必须重新下达明确命令。
2. 前方距离小于 `emergency_distance`：禁止正向线速度，允许原地避让旋转。
3. 超过 `scan_timeout` 未收到雷达：任何正向运动立即停车；后退和原地旋转仍可用于脱困。
4. 模式行为或手动命令。
5. 加速度限制和平滑输出。

状态发布到 `/robot/simulation_state`，包含 mode、front/right distance、linear/angular、sensor_stale、safety_stopped 和 reason。

## 4. 安装与构建

```bash
cd /home/ubuntu/embodied_agent_ws
bash scripts/bootstrap.sh
source scripts/activate.sh
```

若已有基础环境，可只安装仿真依赖：

```bash
sudo apt-get install ros-jazzy-ros-gz ros-jazzy-turtlebot3-gazebo ros-jazzy-rviz2
colcon build --symlink-install --packages-select embodied_simulation
```

## 5. 启动方式

### 无模型的功能演示

```bash
source scripts/activate.sh
ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  provider_mode:=mock microphone_enabled:=false speaker_enabled:=false
```

另一个终端发送文字，验证 Agent 到 Gazebo：

```bash
ros2 topic pub --once /agent/text_input std_msgs/msg/String \
  "{data: '小智，向前走一秒'}"
ros2 topic echo /odom
```

### 在线真实语音

```bash
ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  agent_type:=online provider_mode:=online \
  microphone_enabled:=true speaker_enabled:=true
```

需要 `.env` 中的 DashScope Key 和 WSLg 麦克风/扬声器。节点启动时会预热 LLM 并建立持久 TTS 连接。

### 离线真实语音

终端一：

```bash
bash scripts/start_llama_server.sh
```

终端二：

```bash
ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  agent_type:=offline provider_mode:=offline \
  microphone_enabled:=true speaker_enabled:=true
```

### RViz 与无界面运行

```bash
# Gazebo GUI + RViz
ros2 launch embodied_simulation voice_turtlebot3.launch.py rviz:=true

# CI/自动测试
ros2 launch embodied_simulation voice_turtlebot3.launch.py \
  gui:=false rviz:=false launch_agent:=false
```

## 6. 模式话题和急停

不经过 Agent 也可以测试控制器：

```bash
ros2 topic pub --once /robot/control_mode_request std_msgs/msg/String \
  "{data: obstacle_avoidance}"

ros2 topic pub --once /robot/emergency_stop std_msgs/msg/Empty "{}"
```

观察：

```bash
ros2 topic echo /scan
ros2 topic echo /cmd_vel
ros2 topic echo /odom
ros2 topic echo /robot/simulation_state
```

## 7. 自动验收

```bash
# 不启动 Gazebo：动作 Guard、合成 LaserScan、安全停车、模式、Twist
bash scripts/smoke_test_simulation.sh

# 真实 Gazebo：可信动作驱动 TurtleBot3 并产生可验证里程计位移
bash scripts/acceptance_test.sh gazebo

# 真实离线语音模型 + 真实 Gazebo 物理闭环
bash scripts/acceptance_test.sh gazebo-voice
```

真实麦克风的交互式验收：

```bash
bash scripts/accept_voice_simulation_microphone.sh offline
```

它不是只观察机器人是否移动，而是同时要求 ASR final、动作候选、Guard 后动作、simulation ACK、非零 Twist 和合理 Odometry 位移。

多次验收观测：可信动作测试移动 0.086–0.091 m；离线语音识别动作主体后移动 0.061–0.066 m，并收到 `backend=simulation` 的 move ACK。短唤醒词曾被识别为“脚AL/早之/早日”，但“向前走一秒”被正确解析。测试使用独立 `ROS_DOMAIN_ID`，并按进程组清理 Gazebo，避免多个 `/clock`、`/odom` 污染测量。

## 8. 当前边界与下一阶段

已经完成：语音/文字动作、模式切换、雷达紧急停车、基础避障、右侧沿墙 PID、速度平滑、Gazebo/RViz 集成和真实物理位移测试。`stop` 会退出自动模式并保持停车，不会在下一控制周期自行恢复。

下一阶段建议：

1. 增加键盘 WASD 节点，并用统一 mux 明确语音、键盘、Nav2 的控制优先级。
2. 在自定义世界中为避障和沿墙建立可重复轨迹指标，而不只检查局部速度。
3. 加入 TF2 目标坐标动作，如“前往客厅”，再接 Nav2 action，而不是让 LLM 直接输出连续速度。
4. 使用 rosbag 记录 `/scan`、`/cmd_vel`、`/odom`、状态和动作，离线回放回归。
5. 统计真实麦克风下唤醒率、误唤醒率、碰撞次数、轨迹完成率和语音到运动 P95。
