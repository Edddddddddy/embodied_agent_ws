# 02 Launch 与系统启动过程

## 源码导航

| 文件与关键行 | 符号 | 观察点 |
|---|---|---|
| `src/embodied_simulation/launch/voice_turtlebot3.launch.py:28` | `generate_launch_description` | 顶层 Gazebo/Agent 组合 |
| `src/embodied_simulation/launch/simulation_control.launch.py:13` | `generate_launch_description` | typed、composition、namespace |
| `src/embodied_online_agent/launch/online_agent.launch.py:9` | `generate_launch_description` | Agent/Audio/Guard/Lifecycle manager |
| `src/embodied_offline_agent/launch/offline_agent.launch.py:11` | `generate_launch_description` | 离线替代部署 |
| `src/embodied_simulation/src/simulation_control_main.cpp:8` | `main` | 2-thread executor |

## 1. Launch 解决什么问题

Launch 不只是“一次启动多个程序”。它把以下部署决策编码成可复现系统：

- 启动哪些节点；
- 每个节点读取哪些 YAML 和 override；
- 使用在线、离线还是 mock provider；
- 使用 legacy Topic 还是 typed Action；
- 使用 Gazebo 还是 mock executor；
- Lifecycle 是否自动 configure/activate；
- 控制节点独立进程还是 component；
- namespace、仿真时间和 Gazebo bridge 如何连接。

## 2. 顶层启动树

入口 `voice_turtlebot3.launch.py` 的结构为：

```text
voice_turtlebot3.launch.py
├─ gz_sim server
├─ gz_sim GUI                 [gui=true]
├─ turtlebot3 robot_state_publisher
├─ ros_gz_sim create          生成 burger 模型
├─ ros_gz_bridge parameter_bridge
├─ simulation_control.launch.py
│  ├─ simulation_control LifecycleNode 或 component
│  ├─ simulation_lifecycle_manager
│  └─ typed_action_bridge     [use_typed_actions=true]
├─ online_agent.launch.py     [agent_type=online]
│  ├─ online_agent
│  ├─ audio_frontend
│  ├─ action_guard LifecycleNode
│  └─ action_guard lifecycle manager
└─ offline_agent.launch.py    [agent_type=offline]
```

`online_condition` 和 `offline_condition` 保证正常情况下只启动一种 Agent。

## 3. 参数如何覆盖

ROS 2 参数的常见优先关系是默认值、YAML、launch 中后出现的参数字典。以 online launch 为例：

```python
parameters=[
    config,
    {
        "mode": mode,
        "microphone_enabled": ParameterValue(microphone_enabled, value_type=bool),
    },
]
```

节点先从 YAML 读取，再由 launch argument 覆盖 `mode` 和 microphone 开关。`ParameterValue(..., value_type=bool)` 很重要：LaunchConfiguration 本质是字符串替换，不显式转换可能把 `"false"` 当成字符串而非布尔值。

排查参数时使用：

```bash
ros2 param list /online_agent
ros2 param get /online_agent microphone_enabled
ros2 param dump /online_agent
```

不要只看 YAML 猜最终值。

## 4. Lifecycle 启动顺序

`action_guard` 和 `simulation_control` 创建时处于 unconfigured。`nav2_lifecycle_manager` 接收 `node_names` 后执行：

```text
configure -> inactive -> activate -> active
```

`autostart=false` 时，需要手动调用：

```bash
ros2 lifecycle get /action_guard
ros2 lifecycle set /action_guard configure
ros2 lifecycle set /action_guard activate
```

为什么这会影响功能：Guard inactive 时会忽略 action candidate；SimulationControl inactive 时控制 timer 不运行、lifecycle publisher 不发布速度。

`bond_timeout=0.0` 是因为节点继承原生 `rclcpp_lifecycle::LifecycleNode`，没有使用 Nav2 自带 bond 行为。它让 manager 不以缺少 bond 判定节点失联；这也意味着这里不能把 bond 当作运行时健康证明。

## 5. typed 与 legacy 路径

`simulation_control.launch.py` 根据 `use_typed_actions` 设置：

```python
"legacy_command_enabled": use_typed_actions != true
```

并在 typed 模式启动 `typed_action_bridge`。因此两条路径是：

```text
typed=true:
Guard -> robot/action_command_typed -> bridge -> robot/execute_command Action

typed=false:
Guard -> robot/action_command JSON -> SimulationControl on_action()
```

typed 路径有 feedback、cancel、result、timeout 和 BT；legacy 路径主要用于迁移兼容，直接调用 executor，不具备等价的 Action 生命周期。面试时要把“当前默认路径”和“兼容路径”分清。

## 6. 独立进程与 Component

`use_composition=false` 启动 `simulation_control_node` 可执行文件。入口 `simulation_control_main.cpp` 创建 2 线程 `MultiThreadedExecutor`。

`use_composition=true` 启动 `component_container_mt`，动态加载同一个 `SimulationControlNode` 类。复用通过：

- `SimulationControlNode` 只实现一次；
- `make_simulation_control_node()` 给独立 main 创建同一类型；
- `RCLCPP_COMPONENTS_REGISTER_NODE` 注册 component。

| 独立进程 | Component |
|---|---|
| 故障隔离更强，调试直观 | 减少进程和序列化开销 |
| 跨进程消息通常走 DDS | 同进程可利用 intra-process 优化 |
| 启动和内存开销更高 | 一个进程崩溃可能影响多个节点 |
| 可单独观察 CPU/内存 | executor 和 callback group 设计更关键 |

当前代码虽支持 component，但没有在 NodeOptions 中显式开启 intra-process communication，所以不能直接宣称“已经零拷贝”。

## 7. Gazebo bridge

Gazebo Transport 与 ROS 2 DDS 是不同通信系统。`ros_gz_bridge` 根据 `turtlebot3_bridge.yaml` 转换：

- Gazebo LaserScan -> ROS `/scan`；
- Gazebo Odometry -> ROS `/odom`；
- ROS `/cmd_vel` -> Gazebo 速度命令；
- 其他时钟和 TF 相关消息。

如果 `/cmd_vel` 在 ROS 中有数据但机器人不动，问题可能位于 bridge、Gazebo topic 名、模型插件或仿真暂停，而不是 Action server。

## 8. `use_sim_time`

仿真节点使用 `/clock` 作为 ROS time。控制节点的 `now_seconds()` 取 `get_clock()->now()`，因此 Action duration 与仿真时钟一致。控制 timer 是 wall timer：它按真实时间触发，但状态计算使用 ROS time。

这带来一个细节：仿真暂停时 timer 仍可能触发，但 `now` 不前进，动作进度不增长。恢复仿真后继续。测试超时和持续时间时必须知道使用的是 wall time 还是 ROS time。

## 9. 启动后的 ROS graph 验证

```bash
ros2 node list
ros2 topic list -t
ros2 action list -t
ros2 lifecycle nodes
ros2 component list
```

typed Gazebo 链至少应看到：

```text
/agent/action_candidate
/robot/action_command_typed
/robot/execute_command
/robot/action_feedback
/robot/action_result
/cmd_vel
/scan
/odom
```

还应确认 Action server：

```bash
ros2 action info /robot/execute_command
```

## 10. 常见启动故障

| 表现 | 优先检查 |
|---|---|
| action candidate 有数据，Guard 无输出 | Guard lifecycle 是否 active |
| typed command 有数据，result 显示 server unavailable | SimulationControl 是否 active、Action 名称/namespace |
| 控制节点 configure 失败 | executor plugin 名、BT XML、参数合法性 |
| `/scan` 不存在 | bridge YAML、Gazebo server、模型是否生成 |
| 节点时间不前进 | `/clock`、`use_sim_time`、仿真是否暂停 |
| namespace 模式找不到 Action | 客户端和服务端是否都使用相对名并处于同一 namespace |

## 11. 面试回答模板

**问题：为什么使用 Lifecycle 和 composition？**

控制节点在配置、激活和停止时有明确资源边界。configure 阶段加载 plugin 和 BT、创建接口但不输出速度；activate 后才启动控制 timer；deactivate 会先取消目标、执行 stop 并发布零速度，再停 publisher。这能保证未准备好的节点不会控制底盘。composition 是部署选择，同一个 `SimulationControlNode` 既能独立运行，也能装进多线程 component container，避免维护两套控制逻辑。独立进程更利于故障隔离，component 更节省进程和通信开销，项目通过 launch 参数切换。

## 12. 自测

1. `provider_mode=mock` 和 `executor_plugin=MockRobotExecutor` 分别替换哪一层？
2. `use_typed_actions=false` 后哪些 Action 能力消失？
3. 为什么 `ParameterValue` 要声明 bool 类型？
4. 仿真暂停时 wall timer 和 ROS time 分别怎样变化？
5. component 模式为什么不自动等于零拷贝？
