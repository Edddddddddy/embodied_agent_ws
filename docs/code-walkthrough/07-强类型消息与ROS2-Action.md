# 07 强类型消息与 ROS 2 Action

## 源码导航

| 文件与关键行 | 符号 | 观察点 |
|---|---|---|
| `src/embodied_agent_interfaces/msg/RobotCommand.msg:1` | `RobotCommand` | typed command 字段与枚举 |
| `src/embodied_agent_interfaces/action/ExecuteRobotCommand.action:1` | `ExecuteRobotCommand` | goal/result/feedback |
| `src/embodied_agent_cpp/src/typed_action_bridge_node.cpp:56` | `on_command` | Action client callbacks |
| `src/embodied_simulation/src/simulation_control_node.cpp:306` | `handle_goal/handle_cancel` | 接受与取消策略 |
| `src/embodied_simulation/src/simulation_control_node.cpp:329` | `handle_accepted` | immediate、抢占、启动执行 |
| `src/embodied_simulation/src/action_execution.cpp:16` | `ActionExecution::update` | 终态优先级与 progress |
| `src/embodied_simulation/src/simulation_control_node.cpp:466` | `update_active_action` | feedback/result/stop 映射 |

## 1. 从字符串协议到任务协议

legacy 命令是 JSON String：灵活，但字段直到运行时才发现错误。`RobotCommand.msg` 将动作统一为固定字段和枚举：

```text
command_id/source/action_type
linear_x/angular_z/duration_s
count/color/mode
```

未使用字段保持默认值。`action_type` 决定哪些字段有意义。

强类型的收益：

- 编译期生成 Python/C++ 类型；
- ros2 interface 可发现；
- rosbag 和工具能理解字段；
- 不需要每个消费者重复 JSON key 解析；
- Action goal 可以直接嵌套命令。

代价是接口变更需要重新生成、重新编译，兼容演进要更谨慎。

## 2. RobotCommandAdapter

Adapter 先调用 `ActionValidator`，再把合法 JSON 映射为 typed message。它同时保留规范化后的 legacy JSON，确保两条迁移路径使用同一 clamp 结果。

例如输入：

```json
{"name":"move","arguments":{"linear_x":9,"duration_s":20}}
```

转换后：

```text
action_type = MOVE
linear_x = 0.5
duration_s = 10.0
command_id = guard-N
source = agent
```

## 3. Action 接口三段

`ExecuteRobotCommand.action` 由 `---` 分成：

### Goal

```text
RobotCommand command
```

### Result

```text
success
status = SUCCEEDED / REJECTED / CANCELED / TIMED_OUT / BLOCKED
message
```

### Feedback

```text
phase = ACCEPTED / EXECUTING / STOPPING
progress [0,1]
detail
```

Result 的 status 比单一 bool 更有诊断价值。blocked 表示环境安全条件中止，timed_out 表示硬上限，canceled 表示用户或抢占请求。

## 4. typed bridge 客户端

`TypedActionBridgeNode::on_command()`：

1. 最多等 Action server 1 秒；
2. 不可用则发布 rejected result；
3. 构造 goal；
4. 设置 goal response、feedback、result callbacks；
5. `async_send_goal()`。

bridge 不阻塞等待动作完成，因此仍能接收后续 typed command。服务端定义新 goal 如何抢占旧 goal。

反馈和结果被转成 JSON String 观察 Topic，是为了 CLI/测试方便；Action 本身仍是权威任务接口。

## 5. 服务端 goal 接受

`handle_goal()` 只做快速检查：

- Lifecycle 必须 active；
- 没有 BT 时，命令必须通过 `supported_action_goal()`；
- 返回 `ACCEPT_AND_EXECUTE` 或 `REJECT`。

Action server 的 goal callback 应快速返回，不能在其中运行运动循环。真正启动发生在 `handle_accepted()`。

一个代码细节：启用 BT 时，`handle_goal()` 对 unsupported goal 也可能先接受，再由 BT Validate 阶段 abort。这让客户端看到“goal accepted 后 result rejected”，而不是 goal response 直接拒绝。文档和测试应区分两种拒绝时机。

## 6. immediate 与 duration action

stop 和 set_mode 是 immediate：执行后立刻 `succeed/abort`。

move/turn：

1. 若已有 active goal，先 cancel BT 并以 `preempted_by_new_goal` 结束旧目标；
2. 启动 BT；
3. 调用 executor `execute()` 设置目标速度和截止时间；
4. 创建 `ActionExecution(duration, timeout, start)`；
5. 发布 accepted feedback 和 ACK；
6. 之后由 20 Hz `control_tick()` 推进。

## 7. ActionExecution 终态优先级

`ActionExecution::update()` 计算：

```text
elapsed = now - start
progress = clamp(elapsed / duration, 0, 1)
```

判定顺序：

1. cancel；
2. blocked；
3. hard timeout，且 timeout 小于 requested duration；
4. duration 完成；
5. running。

所以同一 tick 同时发生 cancel 和 obstacle，返回 canceled；这是代码定义的可测试语义。

timeout 条件写成 `elapsed >= timeout && timeout < duration`，当默认 12 秒 timeout 大于最大 10 秒动作时不会触发。它主要防御未来更长任务或配置改变。

## 8. 取消与抢占

`handle_cancel()` 只接受当前 `active_goal_` 的取消请求。实际终止不在 cancel callback 里完成，而在下一次 control tick：

```text
active_goal_->is_canceling()
  -> ActionExecution kCanceled
  -> BT canceled 或 direct state
  -> finish_active_action
  -> executor_->stop()
  -> goal_handle->canceled(result)
```

新 goal 抢占旧 goal 时，旧 goal 未处于 `is_canceling()`，因此 `finish_active_action(kCanceled)` 对旧目标调用 `abort()`，status 字段仍是 CANCELED。客户端应同时看 ROS Action result code 和自定义 status。

## 9. finish 的安全不变量

所有 duration action 终态进入 `finish_active_action()`，第一步是 `executor_->stop()`。随后设置 result、结束 goal、清理 active pointer 和 execution state。

`control_tick()` 若发现本 tick 刚结束动作，用 `action_stopped` 强制本次发布零速度，避免 controller step 在 stop 前计算出的旧速度再发一次。

## 10. 旧 JSON Topic 与 Action 对比

| 能力 | legacy Topic | typed Action |
|---|---|---|
| schema | 运行时 JSON | IDL + 服务端检查 |
| goal 接受 | ACK 模拟 | 内建 goal response |
| 进度 | 无统一语义 | feedback |
| 取消 | 另发 stop | cancel protocol |
| 抢占 | 隐式覆盖 | 明确定义旧目标终态 |
| 结果 | ACK，不完整 | result + status |
| BT | legacy 路径绕过 | 默认路径接入 |

## 11. 面试回答模板

**问题：为什么从 Topic 升级为 Action？**

JSON Topic 适合早期验证，但一个持续数秒的运动不是瞬时事件。Action 把 goal 接受、执行 feedback、用户取消和最终 result 变成标准协议。Guard 先把不可信 JSON 校验并转成带 command ID 的 `RobotCommand`，bridge 再发送 `ExecuteRobotCommand`。服务端用 20 Hz timer 推进 `ActionExecution`，不会 sleep 阻塞 callback，所以运动期间仍可处理雷达、取消和新目标。新目标会明确抢占旧目标，障碍返回 BLOCKED，硬超时返回 TIMED_OUT，所有终态统一 stop。项目保留 legacy Topic 作为迁移兼容，但默认 typed Action 才是完整可靠链。

## 12. 自测

1. `RobotCommand` 为什么仍保留所有动作字段而不是 union？
2. goal response reject 与 result rejected 有何区别？
3. cancel 为什么在下一 control tick 才真正结束？
4. cancel 与 blocked 同时发生，哪一个优先？
5. 为什么 finish 后本 tick 还要强制发布零速度？
