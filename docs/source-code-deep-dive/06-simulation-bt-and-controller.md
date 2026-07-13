# 06. 仿真 Action Server、行为树与控制器

## 先给结论

`SimulationControlNode` 是执行域入口：它接收统一 `ExecuteRobotCommand` goal，立即动作直接确认，长动作交给 executor 和 `ActiveActionRuntime`，每个控制周期更新反馈、BT、安全状态和 `/cmd_vel`。行为树负责可组合的执行策略，控制器负责连续动力学与传感器安全，两者职责不同。

## configure 阶段装配什么

核心文件：[simulation_control_node.cpp](../../src/embodied_simulation/src/simulation_control_node.cpp)

`on_configure()` 的顺序：

1. 读取速度、加速度、雷达距离、PID、控制频率和 action timeout。
2. `validate_node_configuration()` 检查参数组合。
3. 用 pluginlib 加载 `RobotExecutor`。
4. 创建 `SimulationRosIo` 的 managed publishers。
5. 读取 BT XML，注册并创建行为树实现。
6. 创建 `ActiveActionRuntime`。
7. 创建 `ExecuteRobotCommand` Action server。
8. 创建 mode、emergency stop、LaserScan subscriptions。
9. 创建控制 timer 和独立 mutually-exclusive diagnostics timer，初始均 cancel。

只有 `on_activate()` 才激活 publisher 和 timer，因此 inactive 节点不会产生速度。

## Action Server 三个入口

### `handle_goal()`

先检查 Lifecycle active，再检查动作是否合法。inactive 或不支持的 goal 直接 REJECT，避免进入 executor 后才发现节点不能工作。

### `handle_cancel()`

只接受与 `active_goal_` 相同的 handle。不能取消一个已经不是当前目标的旧 goal。

### `handle_accepted()`

若意外已有 active goal，先取消旧 BT 并以 `preempted_by_new_goal` 收敛。正常情况下上游 scheduler 已保证单 goal，这里是执行端纵深防御。

动作分两类：

- 立即动作：STOP、CANCEL_NAVIGATION、SET_MODE、WAVE、SET_LED。执行后立刻 succeed/abort。
- 长动作：MOVE、TURN、NAVIGATE_TO、FOLLOW_WAYPOINTS。启动 executor 和 action runtime，后续由控制 tick 推进。

## BehaviorTree 结构

XML 位于 [command_tree.xml](../../src/embodied_simulation/config/command_tree.xml)：

```xml
<ReactiveSequence>
  <ValidateCommand/>
  <CheckSafety/>
  <ExecuteCommand/>
  <ConfirmResult/>
</ReactiveSequence>
```

### 为什么是 ReactiveSequence

每次 tick 从前面的条件重新检查。动作运行期间若 LaserScan 突然显示近障，`CheckSafety` 会在下一周期失败，而不是只在 goal 开始时检查一次。

### Blackboard 存什么

`CommandBehaviorTree` 把以下值放入 blackboard：

- `command`
- `safety_blocked`
- `execution_state`
- `outcome`
- `stage`
- `detail`

节点不直接持有 ROS publisher。`tick()` 返回 `CommandTreeResult`，再由 `SimulationRosIo` 转成 `BehaviorTreeStatus`。因此 BT 逻辑可脱离 ROS 单测。

### 四个节点

1. `ValidateCommandNode`：执行端再次验证 command。
2. `CheckSafetyNode`：运行时安全阻断。
3. `ExecuteCommandNode`：把 `ActionExecutionState` 映射到 RUNNING/SUCCESS/FAILURE。
4. `ConfirmResultNode`：只有真实 succeeded 才确认成功。

BT 当前结构不算复杂，但它提供明确扩展点。以后加入电量检查、门状态、恢复行为时，可以改 XML 和节点组合，不必把更多 if/else 塞进 Action callback。

## `ActionExecution` 的终态优先级

核心文件：[action_execution.cpp](../../src/embodied_simulation/src/action_execution.cpp)

进度是：

```text
progress = clamp(elapsed / duration, 0, 1)
```

终态判断顺序：

1. cancel requested -> CANCELED
2. safety blocked -> BLOCKED
3. hard timeout 且 timeout 小于计划 duration -> TIMED_OUT
4. elapsed 达到 duration -> SUCCEEDED
5. 否则 RUNNING

优先级固定在纯值对象中，避免 BT 和 Action server 各写一套判断。

## `ActiveActionRuntime` 统一本地与外部动作

核心文件：[active_action_runtime.cpp](../../src/embodied_simulation/src/active_action_runtime.cpp)

本地 MOVE/TURN 以 duration 作为完成依据。Nav2 则设置 `uses_external_result=true`：

- 本地 `ActionExecution` 仍提供 progress 下限和 hard timeout。
- 只要未本地超时，外部 Nav2 update 覆盖 running/succeeded/canceled/blocked。
- 外部 detail 优先透传 planner/controller 失败原因。
- 最终状态再进入 BT。

这样本地动作和 Nav2 使用同一个外层 Action result 契约，但完成证据各自正确。

## RobotExecutor 插件接口

核心文件：[robot_executor.hpp](../../src/embodied_simulation/include/embodied_simulation/robot_executor.hpp)

关键契约：

- `execute()` 和 `step()` 不得长时间阻塞 ROS executor。
- `stop()` 必须幂等并立即归零/取消。
- `publishes_cmd_vel()` 表示速度由本节点还是外部 Nav2 控制器发布。
- `external_action_update/detail()` 让外部 Action server 回传真实终态。

### Gazebo executor

[gazebo_robot_executor.cpp](../../src/embodied_simulation/src/gazebo_robot_executor.cpp) 用 `SimulationController`：

- MOVE 支持 linear + angular，因此 ARC 也走 MOVE。
- TURN 设 linear 为 0。
- 非 Nav2 profile 的 navigate/follow 用可观测替代运动，只证明语义链路和执行可见，不证明真实路径规划。
- WAVE/LED 因仿真无附件，只 ACK 并停止底盘。

### Mock executor

用于不拉 Gazebo 的控制逻辑和 Action 生命周期验收。它证明 plugin seam 可替换，但不能作为物理运动证据。

### Nav2 executor

只处理导航类命令，且 `publishes_cmd_vel=false`，避免 SimulationControl 的零速度与 Nav2 controller 抢 `/cmd_vel`。

## 20 Hz 控制器怎么工作

核心文件：[simulation_controller.cpp](../../src/embodied_simulation/src/simulation_controller.cpp)

### 手动动作

`set_manual_command()`：

- 将 mode 切到 manual。
- 先按 controller 自身更保守的速度上限 clamp，默认 0.22 m/s 和 1.0 rad/s。
- 记录 `manual_until_s = now + duration`。
- 清 PID 并允许从当前速度平滑接近目标。

ActionGuard 上限比 controller 上限宽，表示 Guard 是系统硬边界，具体底盘仍可有更保守能力边界。

### 加速度斜坡

非强制停止时：

```text
current_linear = approach(current_linear, target_linear, linear_acceleration * dt)
current_angular = approach(current_angular, target_angular, angular_acceleration * dt)
```

避免速度从 0 瞬间跳到目标，减小仿真/真实底盘冲击。强制停止、急停、scan fail-safe 时直接赋目标零速度，不等待斜坡。

## LaserScan 安全

`update_scan()` 把有效 range 按角度分成：

- 正前方 ±20 度。
- 左侧 90 度 ±25 度。
- 右侧 -90 度 ±25 度。

忽略 NaN、inf 和超出 range min/max 的值，各扇区取最小距离。

### scan stale

超过 `scan_timeout` 未收到雷达：

- autonomous 模式停止。
- 任何正向 linear 速度被置零并标记 `safety_stopped`。

这是 fail-safe 思路：不知道前方是否安全时，不继续向前。手动后退不因前方 scan stale 被一刀切停止。

### 近障停车

前方距离小于 `emergency_distance` 且 linear_x > 0 时，正向速度归零，reason=`front_emergency`。转向或后退可保留，用于脱困。

## 避障与沿墙

### obstacle avoidance

前方小于 `obstacle_distance` 时，根据左右空间选择转向；否则以 autonomous speed 前进。这是轻量反应式控制，不是全局路径规划。

### wall following

前方有障碍先转弯；右侧无墙时向右搜索；检测到右墙时以：

```text
error = target_right_distance - measured_right_distance
angular = Kp*error + Ki*integral + Kd*derivative
```

控制右侧距离。积分项 clamp 防 windup，模式切换和拐角时 reset PID。

## 控制 tick 的完整顺序

`control_tick()` 每周期：

1. `executor.step(now)` 得到速度、安全和状态快照。
2. `update_active_action()` 处理 cancel、阻断、Nav2 result、BT 和终态。
3. 若 executor 自己不发布速度，则按 action 是否刚终止发布当前或零速度。
4. 发布 `SimulationState`。

Action 终态前 `finish_active_action()` 必先 `executor.stop()`，再映射 succeed/canceled/timeout/blocked，最后 reset runtime。这保证“宣布完成”与“底盘已停止”的顺序。

## 自测问答

### 问：有 Action server 为什么还要 BT？

答：Action server 解决跨进程 goal/feedback/result/cancel；BT 解决执行域内部可组合决策，如校验、安全检查、执行和结果确认。两者处在不同抽象层。

### 问：pluginlib 的实际价值是什么？

答：同一个 Action server、BT 和状态输出可以换成 Mock、Gazebo 或 Nav2 后端。上游 Agent/Guard/Scheduler 不需要知道后端类型，测试也能分层运行。

### 问：怎么证明动作真的执行了，而不是只发了消息？

答：至少要联合观察 Action feedback/result、BT status、SimulationState、`/cmd_vel`，Gazebo 中还可观察 odom/模型位姿。只有 candidate 或 ACK 不能证明运动完成。

### 问：雷达失联为什么只阻止正向速度？

答：前方未知时继续前进危险；后退和原地转向可能是安全脱困动作。当前策略是方向相关的 fail-safe，不是所有自由度全部锁死。
