# 05. 动作安全、Outbox 与 Action 调度

## 先给结论

模型不能直接发布 `/cmd_vel`，因为语言输出不是可信控制输入，也无法天然表达反馈、取消和终态。项目把动作链拆成候选、校验、调度、Action 执行四层，并用 `command_id` 贯穿结果关联。

## 领域动作到强类型候选

Python 内部使用 [types.py](../../src/embodied_agent_core/embodied_agent_core/types.py) 的 `ActionCommand(name, arguments, request_id, priority)`。发布前 [ros_action_transport.py](../../src/embodied_agent_core/embodied_agent_core/ros_action_transport.py) 将它映射成 [RobotCommand.msg](../../src/embodied_agent_interfaces/msg/RobotCommand.msg)。

强类型消息的收益：

- 编译/生成期固定字段和类型。
- Python/C++ 不再各自解析控制 JSON。
- action type 使用常量，不依赖自由字符串。
- rosbag、topic echo 和测试容易观察。

但强类型不能阻止 `linear_x=999`、NaN、MOVE 携带 LED 字段或非法地点，所以仍需业务校验。

## `SequentialActionPublisher` 只负责批次

核心文件：[action_sequence.py](../../src/embodied_agent_core/embodied_agent_core/action_sequence.py)

它为每条动作补唯一 `agent-action-N`，一次把整个 batch 发布给 C++ scheduler，然后按 ID 等终态。它不决定“下一条何时发送”，否则会与 C++ FIFO 形成双调度。

等待失败或 timeout 时，它额外发布 priority STOP。`cancel()` 递增 `_cancel_generation` 并唤醒 condition，防止“走正方形”在用户喊停后继续等待并推进。

## ActionGuard 校验规则

核心文件：[action_validator.cpp](../../src/embodied_agent_cpp/src/action_validator.cpp)

### 通用规则

- command ID 为空时使用 Guard 生成的 fallback ID。
- source 为空时补 `agent`。
- priority 只允许 STOP/CANCEL_NAVIGATION。
- 未知 action type 拒绝。
- 动作不相关字段必须为空/零。

### 运动规则

| 字段 | Guard 范围 |
| --- | ---: |
| `linear_x` | `[-0.5, 0.5]` m/s |
| `angular_z` | `[-1.5, 1.5]` rad/s |
| `duration_s` | `[0, 10]` s |

所有浮点先检查 `isfinite`。ARC 验证后规范化为 MOVE，因为底层执行器用同一组 `linear_x/angular_z/duration` 表达圆弧。

### 非运动和导航规则

- wave 次数 clamp 到 1 到 5，但 0 直接拒绝。
- LED 只允许 off/red/green/blue/yellow/white。
- mode 只允许 manual/obstacle_avoidance/wall_following。
- navigate target 必须在地点白名单。
- waypoints 非空、最多 8 个、每个在白名单，loops 限 1 到 3。

为什么有些值 clamp、有些拒绝？连续数值的小幅越界可以安全收敛到物理上限；动作类别、颜色、地点和字段组合代表离散语义，擅自猜测会改变意图，所以拒绝。

## Guarded outbox

核心文件：[guarded_command_outbox.cpp](../../src/embodied_agent_cpp/src/guarded_command_outbox.cpp)

ActionGuard 是 LifecycleNode。active 后收到合法候选时，不直接假设 scheduler 已完成 discovery，而是：

1. 以 steady clock 时间入有界 outbox。
2. 检查 publisher subscription count。
3. 下游 ready 时 drain 并更新时间戳后发布。
4. 等待超过 TTL 时发布 `action_downstream_unavailable:<id>`。
5. disconnected 时 health 从 READY 变 DEGRADED。

steady clock 不受系统时间校准影响，适合 TTL/watchdog；ROS time 仍用于消息 header。

## `ActionScheduler` 状态机

核心文件：[action_scheduler.cpp](../../src/embodied_agent_cpp/src/action_scheduler.cpp)

内部状态只有：

```text
active_: optional RobotCommand
pending_: deque RobotCommand
cancel_requested_: bool
```

### 普通命令

- 没有 active：立刻生成 Dispatch event。
- 有 active：进入 pending 尾部。
- pending 满：为该命令生成 REJECTED result。
- command ID 与 active/pending 重复：只记 diagnostics，不发布同 ID 失败，防止原请求误消费这个终态。

### priority 命令

只有 priority STOP/CANCEL：

1. 为所有 pending 生成 CANCELED result 并清队列。
2. 无 active 时直接 dispatch。
3. 有 active 时放 pending 头部并生成 CancelActive event。
4. 等 active 返回 canceled/其他终态后 dispatch priority 命令。

这保证任一时刻只有一个 goal，避免 STOP goal 与旧运动 goal 同时在 Action server 竞争。

### 终态

`complete(command_id, ...)` 只接受与 `active_` 相同的 ID。watchdog 已经推进后到达的旧 result 被忽略，不能错误推进新队列。

普通动作失败且 `clear_queue_on_failure=true` 时清 pending；但若失败来自 priority cancel，则仍要继续派发队头的 STOP/CANCEL。

## `TypedActionBridgeNode` 如何适配 ROS Action

核心文件：[typed_action_bridge_node.cpp](../../src/embodied_agent_cpp/src/typed_action_bridge_node.cpp)

纯 `ActionScheduler` 只产生事件，不依赖 ROS。Node Adapter 把事件映射为副作用：

| Scheduler event | ROS 副作用 |
| --- | --- |
| Dispatch | `async_send_goal()` |
| CancelActive | `async_cancel_goal()` |
| Result | 发布 `/robot/action_result` |
| RejectedInput | 记录 diagnostics |

`SendGoalOptions` 的三个回调：

- goal response：保存 goal handle；如果 watchdog 已使该 ID 过期，立刻取消迟到 handle。
- feedback：映射成 `RobotCommandFeedback` topic。
- result：清 goal state，再交给 scheduler complete。

## 取消 watchdog 与迟到回调

取消请求发出后，如果 Action server 永远不返回终态，scheduler 会永久卡在 canceling。因此每 100 ms 检查：

```text
elapsed >= cancel_timeout_s
  -> 清旧 goal state
  -> complete(old_id, TIMED_OUT, cancel_result_timeout)
  -> scheduler 派发 priority command
```

随后旧 result callback 即使到达，也因 active ID 不匹配被忽略。这是异步系统中常见的“代次/关联 ID 防迟到写”模式。

## 为什么模型绝不能发 `/cmd_vel`

如果模型直接发速度，会失去：

- 动作白名单和字段互斥校验。
- 统一速度/时长限幅。
- 组合动作 FIFO。
- command ID 和结果关联。
- 标准取消、反馈和超时。
- executor 替换和 Nav2 语义动作。
- 运行时雷达安全与 Lifecycle 停机闭环。

正确边界是模型只产出“候选意图”，控制层决定它是否可以执行以及如何执行。

## 端到端失败传播示例

假设命令“去不存在的会议室”：

1. NLU 无法解析稳定地点，可能转 LLM。
2. LLM 若输出未知 target，typed conversion 形成 candidate。
3. ActionValidator 拒绝 `unsupported navigation target`。
4. `/robot/action_rejected` 有原因，但不会产生 Action goal。

假设 target 合法但 Nav2 server 不可用：

1. Guard 接受。
2. Scheduler dispatch。
3. Simulation Action server 接受语义 goal。
4. Nav2 executor 等 server 500 ms 失败，给出 `server_unavailable` detail。
5. 外层 Action abort，bridge 发布 BLOCKED/失败 result。
6. Python 批次按 ID 被唤醒并停止后续动作。

## 自测问答

### 问：ActionGuard 与 BT 的校验是否重复？

答：是纵深防御，不是同一层无意义复制。Guard 是不可信 Agent 到执行域的入口，负责字段和上限；BT/Action server 位于执行端，防御旁路 client、动态安全状态和 executor 结果。任何能独立接收输入的边界都应校验自身前提。

### 问：为什么 scheduler 是纯 C++ 类而不是全写在 Node callback？

答：FIFO、重复 ID、priority cancel、失败清队列是状态机逻辑。纯类输入命令/终态、输出事件，可以用毫秒级单测覆盖所有竞态语义，Node 只适配 ROS 副作用。

### 问：为什么 priority STOP 还要等旧 goal 终态？

答：取消请求已经立即发出；等待终态是为了维持“单 active goal”不变量。若同时派发 STOP goal，server 的 preemption 与两个 goal 回调可能交错。watchdog 保证等待不会无限持续。

### 问：强类型消息是否已经足够安全？

答：不够。类型系统能防字段类型和 schema 错误，不能防 NaN、超速、非法字段组合、未知地点或 priority 滥用。业务验证仍由 ActionGuard 完成。
