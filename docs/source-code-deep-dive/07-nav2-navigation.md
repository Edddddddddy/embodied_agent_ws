# 07. 语义导航到 Nav2 的完整实现

## 先给结论

“去门口”先在 NLU 层转换成稳定地点 key `door`，再通过 `RobotCommand.NAVIGATE_TO` 到 Nav2 executor。executor 从 YAML 解析地图坐标，构造 `PoseStamped` 并调用 Nav2 `NavigateToPose`。真正完成与否以 Nav2 result 为准，不以候选消息、goal accepted 或固定 duration 为准。

## 三层导航能力要区分

| 层 | executor/环境 | 能证明什么 | 不能证明什么 |
| --- | --- | --- | --- |
| navigation demo | Mock/Gazebo | 语义命令、Guard、Action、可观测替代运动 | 真实 Nav2 规划与到点 |
| nav2 bridge | Nav2 executor + fake server | 正确构造 `NavigateToPose/FollowWaypoints` goal、取消和 result 映射 | 真实地图定位与控制 |
| Nav2 TurtleBot3 | 官方 Nav2 + Gazebo/map | 完整规划、控制、到点/失败闭环 | 真实硬件环境鲁棒性 |

面试时必须说明使用的是哪一层证据。

## 语义地点解析

核心文件：[navigation_phrases.py](../../src/embodied_agent_core/embodied_agent_core/navigation_phrases.py)

自然语言别名映射到有限 key，例如：

```text
门口 -> door
书桌 -> desk
起点/回到原位 -> home
充电站 -> charging_station
```

这样 LLM/NLU 不直接生成任意地图坐标。地点 key 是稳定领域接口，地图坐标属于部署配置。

巡航表达通过 `extract_waypoints()` 提取有序 key；“开始巡航”使用默认 waypoints；“取消导航”被识别为 priority 控制命令。

## Guard 的导航约束

`ActionValidator` 对 target/waypoints 再做白名单：

- `NAVIGATE_TO` 只允许单 target，无 motion/accessory/waypoint payload。
- `FOLLOW_WAYPOINTS` 要求 1 到 8 个合法地点，loops 归一到 1 到 3。
- `CANCEL_NAVIGATION` 不允许任何 payload，且可以 priority。

因此即使模型构造未知地点，也不会到 Nav2 层触发异常或任意坐标导航。

## 地点 YAML 到 PoseStamped

配置：[places.yaml](../../src/embodied_simulation/config/places.yaml)

```yaml
frame_id: map
places:
  door: {x: 1.2, y: 0.0, yaw: 0.0}
  desk: {x: 1.2, y: 1.0, yaw: 1.57}
```

[nav2_places.cpp](../../src/embodied_simulation/src/nav2_places.cpp) 读取 frame 和每个 `x/y/yaw`。`to_pose_stamped()` 构造位置，并把二维 yaw 转四元数：

```text
orientation.z = sin(yaw / 2)
orientation.w = cos(yaw / 2)
```

找不到 key 时抛 `out_of_range`。正常链路中 Guard 已过滤未知 key，但这里仍保留执行端防御。

## Nav2 executor 的线程模型

核心文件：[nav2_robot_executor.cpp](../../src/embodied_simulation/src/nav2_robot_executor.cpp)

plugin 内创建独立 `rclcpp::Node("nav2_robot_executor")`、两个 Action Client 和一个 `SingleThreadedExecutor`，由 `spin_thread_` 驱动回调。外层 SimulationControl 自己也在 ROS executor 中，因此 Nav2 client 不能等待由同一阻塞线程才能处理的 callback。

析构顺序是：`stop()`、cancel executor、join spin thread，避免后台线程访问已释放对象。

## `NavigateToPose` 发送过程

`send_navigate_goal(target)`：

1. 最多等 action server 500 ms。
2. 设置 external state 为 RUNNING/sending。
3. 从 places 构造 pose 并写当前 stamp。
4. 递增 `navigate_generation_`，清旧 handle。
5. 配置 goal response/result callback。
6. `async_send_goal()`，最多同步等 future 1 秒确认 accepted。
7. timeout/rejected 时设置 BLOCKED 并返回 false。

### 为什么既有异步 callback，又等 future

`execute()` 需要快速回答“goal 是否成功提交”，让外层决定是否启动长动作 runtime；后续真正完成仍由异步 result callback 更新。等待只有 1 秒上限，不等待导航过程。

## `FollowWaypoints`

流程类似，但把每个 key 转成 pose 数组，并设置 `number_of_loops`。result detail 还记录：

- Nav2 result code。
- error code/message。
- `missed_waypoints.size()`。

这比只返回 bool 更适合现场定位规划器、控制器或局部目标失败。

## generation 如何消灭“幽灵导航”

典型竞态：

1. 发送 goal，Nav2 尚未返回 goal handle。
2. 用户立刻说“取消导航”。
3. `stop()` 当下没有 handle 可 cancel。
4. 旧 goal 稍后被 Nav2 接受。

如果只清本地状态，机器人仍会在后台导航。当前实现中 `stop()` 递增 generation。迟到的 goal response callback 发现自己的 generation 已旧：若 handle 有效，立刻 `async_cancel_goal(handle)`，然后退出。它不仅忽略回调，还主动取消远端旧 goal。

result callback 也检查 generation，旧结果不能覆盖下一条导航的 active/detail。

## 外层 Action 如何等待真实 Nav2 result

Nav2 executor：

- `publishes_cmd_vel() == false`，外层不抢速度控制。
- `external_action_update()` 返回当前 Nav2 state。
- `external_action_detail()` 返回 server/goal/planner/controller 细节。

SimulationControl 启动 runtime 时，将外部动作 duration 设为 `action_timeout_s + 1`，并标记 `uses_external_result=true`。每个 tick：

1. 本地计时器检查 cancel、safety 和 hard timeout。
2. 未超时时读取 Nav2 external result。
3. BT 映射终态。
4. 外层 `ExecuteRobotCommand` 返回同样的 succeeded/canceled/blocked/timed_out。

不能在 3 秒后自动说“到达门口”，因为 goal accepted 只表示 Nav2 接单，路径规划和运动可能仍在进行或已经失败。

## Cancel 的两条路径

### 用户说“取消导航”

控制面将其识别为 priority CANCEL_NAVIGATION，scheduler 先取消当前外层 goal。SimulationControl 的 cancel 路径使 `ActiveActionRuntime` 返回 CANCELED，并调用 executor `stop()`，后者取消 Nav2 goal。随后 scheduler 再派发 CANCEL_NAVIGATION 立即 goal，形成确认性停止。

### Lifecycle deactivate

SimulationControl 若有 active goal，先 cancel BT，再 `finish_active_action(CANCELED, "deactivated")`，其中调用 executor stop。之后发布零速度并停 timer/publisher。

## `unreachable_zone` 的作用

YAML 故意放置地图边界外地点，用于验证 Nav2 aborted/失败反馈。它是测试目标，不是业务地点。Guard 白名单中保留它，是为了让失败进入 Nav2 执行路径，而不是在上游被未知地点拦截。

## 故障推演

### server unavailable

`wait_for_action_server(500ms)` 失败，detail 包含 action 名和 target；外层 goal abort，Python 收到失败 result。

### goal rejected

goal response handle 为空，external state=BLOCKED；`execute()` 返回 false，SimulationControl 立即 abort。

### planner/controller aborted

result code 映射为 BLOCKED，detail 保留 Nav2 error code/message，外层 Action abort。

### Nav2 永不返回

外层 `ActiveActionRuntime` hard timeout 先收敛为 TIMED_OUT，并调用 executor stop 取消 Nav2 goal。

## 自测问答

### 问：“去门口”到底怎么导航？

答：NLU 把“门口”解析为 `door`，发布 NAVIGATE_TO typed candidate；Guard 校验地点；scheduler 发送 ExecuteRobotCommand goal；Nav2 executor 从 places.yaml 得到 map 坐标和 yaw，构造 PoseStamped，发送 NavigateToPose goal；最终以 Nav2 result 映射外层 Action 终态。

### 问：为什么不让 LLM 直接输出 x/y？

答：自由坐标难校验、与地图版本耦合、容易越界。稳定地点 key 可白名单，坐标放部署 YAML，地图变化时无需改 prompt 或 NLU。

### 问：Nav2 已经会避障，为什么外层还要 Action timeout？

答：Nav2 可能失联、回调丢失或内部永久不收敛。外层 hard timeout 是系统级资源回收边界，不替代 Nav2 planner/controller 的正常终态。

### 问：Gazebo 的导航替代运动能否证明 Nav2 已实现？

答：不能。它只证明语义命令到执行层可观测。必须展示 Nav2 executor 的真实 goal/result 或完整 TurtleBot3 Nav2 验收，才能证明导航 bridge/闭环。
