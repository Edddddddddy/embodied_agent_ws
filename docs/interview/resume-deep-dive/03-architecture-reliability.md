# 软件架构与可靠性追问

## Q1. 系统为什么分成认知、可信执行和机器人执行三层？

口述：

语音识别和大模型输出带有概率性，只适合产生候选；可信执行层负责类型检查、限幅、排队、取消和终态；机器人执行层才拥有速度、地图和导航结果。这样替换在线或离线模型不会修改底盘安全逻辑，替换 Gazebo 或 Nav2 后端也不会修改会话逻辑。系统共同遵守“上游提出请求，下游返回事实”。

源码落点：

- 应用编排：[agent_application_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py)
- 动作校验：[action_validator.cpp](../../../src/embodied_agent_cpp/src/action_validator.cpp)
- 动作调度：[action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)
- 执行接口：[robot_executor.hpp](../../../src/embodied_simulation/include/embodied_simulation/robot_executor.hpp)

运行：ASR 文本进入会话层，产生 `RobotCommand` 候选；C++ 规范化后转成 Action；执行插件取得物理或导航终态；结果按业务 ID 回到会话层。

## Q2. 状态机和行为树有什么区别？项目为什么两者都用？

口述：

状态机适合阶段有限、转换规则明确的流程，例如建图、存图、启动定位、导航和停止；每个阶段只允许特定命令。行为树适合把条件检查、动作执行、失败传播和后续扩展组织成可组合节点。项目用状态机约束高层会话阶段，用行为树组织单条机器人命令的校验、安全、执行和结果确认，两者解决的层级不同。

源码：

- 会话状态机：[showcase_session.py](../../../src/embodied_slam_tools/embodied_slam_tools/showcase_session.py)
- 命令行为树：[command_behavior_tree.cpp](../../../src/embodied_simulation/src/command_behavior_tree.cpp)
- XML：[command_tree.xml](../../../src/embodied_simulation/config/command_tree.xml)

取舍：简单动作完全可以写 `if/else`；项目保留行为树是为了让流程节点可观测，并能继续加入重试、恢复或不同任务 XML。固定阶段若强行写成行为树，合法转换和持久状态反而不够直观。

## Q3. 项目中的行为树每个控制周期具体做什么？

口述：

树是一个 ReactiveSequence，依次检查命令是否合法、当前安全条件是否允许、执行状态是否仍在运行，最后确认结果。执行节点是有状态动作：运行中返回 `RUNNING`，成功继续到结果确认，取消、超时或阻塞则返回失败。每个控制周期只 tick 一次，所以当前阶段和失败原因可以被状态消息观察。

源码：[command_behavior_tree.cpp](../../../src/embodied_simulation/src/command_behavior_tree.cpp)

```xml
<ReactiveSequence>
  <ValidateCommand/>
  <CheckSafety/>
  <ExecuteCommand/>
  <ConfirmResult/>
</ReactiveSequence>
```

运行：

1. Action goal 到达后把命令写入 blackboard。
2. 执行插件开始动作。
3. 20 Hz 控制循环更新 `safety_blocked` 和 `execution_state`。
4. 行为树 tick 后返回运行、成功或失败。
5. 执行节点据此发布 feedback 或 Action result。

追问：为什么校验还要在行为树中再做？Action server 可能存在旁路输入，执行层必须守住自己的命令契约。

## Q4. pluginlib 在项目中怎样做到执行后端解耦？

口述：

控制节点只依赖 `RobotExecutor` 接口，不直接依赖 Gazebo 或 Nav2 类。Launch 通过 `executor_plugin` 参数选择插件，pluginlib 在运行时创建具体实现。Gazebo 后端输出速度，Nav2 后端发送导航 Action，mock 后端用于快速测试，但它们对上层提供相同的执行、停止、传感器更新和状态查询接口。

源码：

- 接口：[robot_executor.hpp](../../../src/embodied_simulation/include/embodied_simulation/robot_executor.hpp)
- 插件声明：[robot_executor_plugins.xml](../../../src/embodied_simulation/robot_executor_plugins.xml)
- Nav2 实现：[nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)

接口关键约束：

```cpp
virtual bool execute(const RobotCommand &, double now_s) = 0;
virtual void stop() = 0;        // 必须幂等，并尽快停止副作用
virtual ControllerOutput step(double now_s) = 0; // 不长时间阻塞 executor
virtual bool publishes_cmd_vel() const;
```

取舍：插件接口要保持小而稳定。把 Nav2 goal handle 或 Gazebo 类型暴露到接口中，会让上层重新耦合具体后端。

## Q5. 动作调度器维护了哪些不变量？

口述：

调度器始终只允许一个活动命令，普通命令按 FIFO 进入有界队列；重复 ID 和队列满会被拒绝。只有当前活动 ID 的终态才能推进队列，旧结果被忽略。任务失败可按配置清空后续命令，防止组合动作在前一步失败后继续执行。

源码：[action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)

```cpp
std::optional<RobotCommand> active_; // 唯一活动任务
std::deque<RobotCommand> pending_;   // 普通命令尾插，优先停止头插

if (!active_) {
  dispatch(command);
} else {
  pending_.push_back(command);
}

if (!active_ || active_->command_id != result_id) {
  return {}; // 迟到或未知结果不能修改当前状态
}
```

设计原因：把纯调度状态从 ROS 回调中抽出来后，取消、失败和迟到结果可以用毫秒级 gtest 穷举，不必每次启动完整 ROS graph。

## Q6. 急停为什么不只是发布一条 STOP？

口述：

若只把 STOP 放到普通队尾，它要等前面的长任务结束，已经失去急停意义。项目在 Python 层取消当前动作批次并清语句队列；C++ 调度层清 pending、请求取消 active，再把 STOP 放到队首；执行端收到取消或停用后立即归零。最后还要观察零速度或硬件确认，不能把“已经发送停止”当成“已经停止”。

源码：

- 上层抢占：[agent_application_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_application_runtime.py)
- 调度抢占：[action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)
- 控制归零：[simulation_controller.cpp](../../../src/embodied_simulation/src/simulation_controller.cpp)

三个必须一致的状态：尚未处理的语句、尚未执行的机器人命令、当前活动 Action。

## Q7. 系统里为什么有多层超时？

口述：

不同超时保护不同资源。音频端点超时用于结束一句话，语句 TTL 防止旧命令排队太久，Action client 等待超时防止无结果，执行端任务超时限制动作本身，取消 watchdog 防止 cancel 永远不返回，传感器超时防止在盲区继续运动。不能用一个统一数值，因为短动作和 Nav2 长任务的正常时长差异很大。

源码：

- 动作批次超时：[action_sequence.py](../../../src/embodied_agent_core/embodied_agent_core/action_sequence.py)
- 取消 watchdog：[typed_action_bridge_node.cpp](../../../src/embodied_agent_cpp/src/typed_action_bridge_node.cpp)
- 控制与传感器超时：[simulation_controller.cpp](../../../src/embodied_simulation/src/simulation_controller.cpp)

设计例子：move/turn 结果通常数秒内返回，Nav2 可能经历规划、恢复和多航点执行，因此使用独立的长任务超时。否则上层会误判导航失败并发送 STOP。

## Q8. 健康检查和 readiness 有什么区别？

口述：

健康消息描述单个组件当前是否可用，readiness 根据某个部署场景要求的组件集合计算系统是否可以接单。聚合节点保存每个组件最近状态和接收时间；只要缺少必需组件、组件报错或状态过期，就发布未就绪。这样 Launch 启动完成并不等于语音、校验、桥接和执行链都已经可用。

源码：[system_readiness_node.cpp](../../../src/embodied_agent_middleware/src/system_readiness_node.cpp)

运行：

1. 各组件发布 `ComponentHealth` 状态快照。
2. 聚合节点按组件名更新 registry，并记录单调时间。
3. timer 检查必需集合和 stale timeout。
4. 发布 `SystemReadiness`，自动任务据此等待阶段切换。

QoS 使用可靠且保留最近状态，晚启动的监控节点不必等待下一次偶发状态变化。

## Q9. 怎样处理异步回调的迟到结果？

口述：

项目使用两种标识。`command_id` 解决跨 Topic、Action 和会话的业务关联；generation 解决同一对象内部旧异步工作的失效。取消导航、停用 Lifecycle 或恢复说话时递增 generation，旧 timer 或旧 Action 回调到达后只退出，不再修改新状态。对于已经被远端接受的旧导航 goal，还要主动取消，不能只丢弃本地回调。

源码：

- ASR 端点代次：[asr_endpoint_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/asr_endpoint_runtime.py)
- Nav2 goal 代次：[nav2_robot_executor.cpp](../../../src/embodied_simulation/src/nav2_robot_executor.cpp)
- 业务 ID 调度：[action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)

区分：ID 回答“这是哪个任务”，generation 回答“这次异步工作还属于当前生命周期吗”。

## Q10. 任务一致性具体指什么？

口述：

一致性不是数据库事务，而是同一命令在排队、执行、取消和结果返回过程中不能串到另一条命令。项目要求 ID 唯一、一个 active、终态只完成同 ID 任务、迟到结果不推进队列、急停同时清理上下两层队列。多步骤动作中，某一步失败会停止后续动作，并用批次报告记录完成到第几步。

源码：

- Python 批次结果表：[action_sequence.py](../../../src/embodied_agent_core/embodied_agent_core/action_sequence.py)
- C++ 单活动槽：[action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)

典型错误：旧导航被取消后，watchdog 已推进新任务；此时旧 result 到达。如果只按“收到一个结果”推进，系统会误把新任务结束。项目必须比较活动 ID。

## Q11. 单条任务异常为什么不会杀死连续语音 worker？

口述：

连续模式只有一个 worker 串行取命令。每条任务在独立的 `try/except/finally` 中执行，异常会发布失败事件并交给统一错误回调，但 `finally` 一定复位 busy、调用 `task_done()`，随后 worker 继续消费下一条。Lifecycle 停用使用单独的受控取消异常，不把正常关停记录成 provider 故障。

源码：[agent_execution_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_execution_runtime.py)

```python
try:
    execute_item(item)
except AgentExecutionCancelled:
    publish_finished(success=False, reason="lifecycle deactivated")
except Exception as exc:
    publish_finished(success=False, reason=str(exc))
    on_error(exc)
finally:
    finish_turn()          # busy 必须复位
    command_queue.task_done()
```

取舍：异常隔离保证服务持续，但不能静默吞错，所以同时保留 typed 事件和日志。

## Q12. Lifecycle 停用时为什么要先清副作用，再销毁接口？

口述：

如果先销毁 Action client 或 publisher，再尝试取消任务，就无法通知下游，也无法给等待方返回终态。项目停用时先停止新输入，清调度器，处理取消和被清命令的结果，再停 timer 和 publisher，最后释放 client、线程及模型资源。恢复激活时重新创建本代状态，旧回调通过 generation 失效。

源码：

- C++ 桥接生命周期：[typed_action_bridge_node.cpp](../../../src/embodied_agent_cpp/src/typed_action_bridge_node.cpp)
- Python Agent 生命周期：[agent_lifecycle_runtime.py](../../../src/embodied_agent_core/embodied_agent_core/agent_lifecycle_runtime.py)

回答关键词：停止接单、取消在途、给出终态、验证静默、释放资源。析构函数只是最后保险，不应承担完整业务关停协议。

## Q13. 如何验证这些可靠性设计不是文档设想？

口述：

纯状态和策略使用 gtest、pytest 验证，例如重复 ID、队列满、取消超时、迟到结果、状态机非法转换和漏点结果。ROS smoke test 验证节点、Lifecycle、Topic 和 Action 接线；Gazebo E2E 再验证实际运动、地图、定位和最终零速度。每层测试回答不同问题，mock 单测不能代替完整运行。

源码与入口：

- C++ 调度测试：[test_action_scheduler.cpp](../../../src/embodied_agent_cpp/test/test_action_scheduler.cpp)
- Python 连续执行测试：[test_agent_execution_runtime.py](../../../src/embodied_agent_core/test/test_agent_execution_runtime.py)
- Action 结果策略测试：[test_nav2_result_policy.cpp](../../../src/embodied_simulation/test/test_nav2_result_policy.cpp)
- 总体验收说明：[TESTING.md](../../TESTING.md)
