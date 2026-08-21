# 08 BehaviorTree 执行状态机

## 源码导航

| 文件与关键行 | 符号 | 观察点 |
|---|---|---|
| `src/embodied_simulation/config/command_tree.xml:1` | `CommandExecution` | ReactiveSequence 结构 |
| `src/embodied_simulation/src/command_behavior_tree.cpp:25` | `valid_command` | BT 内第二次验证 |
| `src/embodied_simulation/src/command_behavior_tree.cpp:48` | `ValidateCommandNode` | rejected 路径 |
| `src/embodied_simulation/src/command_behavior_tree.cpp:69` | `CheckSafetyNode` | 动态安全条件 |
| `src/embodied_simulation/src/command_behavior_tree.cpp:88` | `ExecuteCommandNode` | StatefulActionNode |
| `src/embodied_simulation/src/command_behavior_tree.cpp:126` | `ConfirmResultNode` | 成功确认语义 |
| `src/embodied_simulation/src/command_behavior_tree.cpp:150` | `CommandBehaviorTree::Impl` | factory、blackboard、PImpl |

## 1. BT 在本项目中的真实作用

行为树没有负责路径规划，也没有直接计算速度。它编排一个可信命令的阶段：

```xml
<ReactiveSequence>
  <ValidateCommand/>
  <CheckSafety/>
  <ExecuteCommand/>
  <ConfirmResult/>
</ReactiveSequence>
```

速度和雷达算法在 `SimulationController`，时间/取消状态在 `ActionExecution`，BT 读取它们的快照并形成统一阶段和 outcome。

## 2. BT 基础状态

每个节点 tick 返回：

- `SUCCESS`：本节点完成，Sequence 继续下一个；
- `FAILURE`：本节点失败，Sequence 结束；
- `RUNNING`：异步工作未完成，下个控制周期继续 tick。

项目还在 blackboard 保存领域状态 `CommandTreeOutcome`，因为 BT 的 FAILURE 本身无法区分 rejected、blocked、canceled 和 timed out。

## 3. Blackboard

每个 goal `start()` 时创建新 blackboard，写入：

```text
command
safety_blocked=false
execution_state=running
outcome=running
stage=validate
detail=""
```

每次 `tick()` 更新动态值：safety、execution state、detail。命令保持不变。

blackboard 是树节点之间的共享上下文，不是 ROS Parameter，也不跨 goal 持久化。新 goal 创建新 blackboard，避免旧状态污染。

## 4. 四个自定义节点

### 4.1 ValidateCommand

ConditionNode 检查 STOP、SET_MODE、MOVE、TURN 及有限数和 duration 范围。虽然 Guard 已验证，BT 再检查是 defense in depth，并保护直接 Action client 绕过 Guard 的情况。

注意 BT 当前只接受 stop/set_mode/move/turn；wave 和 LED 虽能通过 Guard、用于硬件链，但仿真 Action executor 不支持。

### 4.2 CheckSafety

读取 `safety_blocked`。为 true 时写入 outcome blocked、stage safety，并返回 FAILURE。

### 4.3 ExecuteCommand

继承 `StatefulActionNode`：

- `onStart()` 和 `onRunning()` 都读取外部 `ActionExecutionState`；
- running -> BT RUNNING；
- succeeded -> SUCCESS；
- canceled/timed out/blocked -> 写领域 outcome 并 FAILURE；
- `onHalted()` 当前无额外操作，真正 stop 由 Action server 统一执行。

### 4.4 ConfirmResult

只有 execution state 是 succeeded 才返回 SUCCESS，并设置 stage confirm/outcome succeeded。它把“执行值对象说完成”转成树的最终确认阶段。

当前 Confirm 没有读取 odom 或下位机 ACK，所以它确认的是执行状态机完成，不是独立传感器验证目标位移。这是重要边界。

## 5. 为什么使用 ReactiveSequence

普通 Sequence 在子节点 RUNNING 后下一 tick 通常从 running child 继续；ReactiveSequence 每 tick 从第一个 child 重新评估。

运动开始时：

```text
Validate SUCCESS -> Safety SUCCESS -> Execute RUNNING
```

下一 tick 新障碍出现：

```text
Validate SUCCESS -> Safety FAILURE(blocked)
                              -> running Execute 被 halt
```

这使安全条件在动作期间持续有效。如果用只检查一次的 Sequence，障碍出现后可能还要等 Execute 自己返回，反应更慢或完全漏检。

## 6. PImpl 为什么出现

`CommandBehaviorTree` 头文件只暴露：

```cpp
start(command)
tick(safety, execution_state, detail)
cancel(detail)
```

BehaviorTree.CPP factory、blackboard 和 tree 类型藏在 `Impl` 中。好处：

- 使用者不用包含大量 BT 头文件；
- 降低编译依赖；
- 外部接口稳定；
- 实现细节可替换。

代价是多一层动态分配和间接调用，但相对 20 Hz 控制周期可以忽略。

## 7. tick 与 Action server 的配合

`SimulationControlNode::update_active_action()` 先调用：

```cpp
ActionExecution::update(now, canceling, output.safety_stopped)
```

再把结果注入 BT。若 outcome running，发布 `PHASE_EXECUTING`；否则映射到 `finish_active_action()`。

BT 不拥有 ROS goal handle，也不直接 publish。Action server 不知道 XML 节点细节。两者通过小型状态值连接，便于分别单测。

## 8. status 去重

BT 每 20 Hz tick，但 `/robot/bt_status` 不必重复发布相同状态。`publish_bt_status()` 计算：

```text
command_id:stage:outcome:detail
```

与上一次相同则跳过。这样保留状态变化，又避免 20 Hz 日志和 Topic 噪声。

## 9. 与硬编码状态机对比

| BT | 硬编码 if/switch |
|---|---|
| XML 可调整结构 | 编译期固定，阅读直接 |
| 适合组合、复用、Reactive 条件 | 简单流程代码更少 |
| 可视化工具和节点生态 | 无额外依赖 |
| blackboard 类型错误多在运行时暴露 | C++ 状态类型更显式 |
| 复杂树可维护性更好 | 四节点简单流程可能显得偏重 |

本项目引入 BT 的主要价值是展示持续安全重检和未来可扩展编排，不应宣称简单 move 必须依赖 BT 才能实现。

## 10. 测试推演

`test_command_behavior_tree.cpp` 验证：

- valid command：running -> confirm success；
- unknown command：validate rejected；
- 运动中出现障碍：safety blocked；
- cancel 与 timeout 保持不同 outcome；
- blocked 后新 goal 创建新树，可恢复成功。

手工推演一条 1 秒 move：

```text
t=0.00: validate/safety success, execute running, progress 0
t=0.50: ReactiveSequence 重查安全, execute running, progress .5
t=1.00: execution succeeded, execute success, confirm success
        -> server stop -> Action result SUCCEEDED
```

## 11. 面试回答模板

**问题：为什么使用 BehaviorTree？**

这里 BT 不计算速度，而是编排可信动作的验证、安全、执行和确认。每个控制周期，ActionExecution 先根据时间、取消和雷达阻塞得到状态，再注入 blackboard。树使用 ReactiveSequence，所以 Execute 处于 RUNNING 时仍会从 Validate 和 CheckSafety 重新 tick，新障碍能立即 halt 当前执行并返回 BLOCKED。领域终态单独用 enum 保存，因为 BT 的 FAILURE 无法区分拒绝、取消、超时和阻塞。外部只看到 start/tick/cancel，factory 和 blackboard 通过 PImpl 隐藏。对于四节点流程硬编码状态机也可实现，但 BT 为后续恢复、条件和子树扩展提供了可组合结构。

## 12. 自测

1. BT 的 FAILURE 为什么不能直接当作 Action rejected？
2. ReactiveSequence 与普通 Sequence 的关键差异是什么？
3. `onHalted()` 为空，机器人为什么仍会停？
4. ConfirmResult 当前确认了什么，又没有确认什么？
5. blocked 后为什么新 goal 不会继承旧 blackboard？
