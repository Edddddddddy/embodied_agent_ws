# 架构深挖讲稿

## 1. 系统分层怎么讲

推荐回答：

```text
我把系统拆成声学输入、模型交互、动作候选、安全仲裁、动作执行、验收观测六层。
```

具体解释：

- 声学输入：C++ PortAudio、AEC、VAD、0.4 秒静音断句，负责把连续音频切成可识别 utterance。
- 模型交互：在线 Qwen 或离线 ZipFormer + llama.cpp + Sherpa-TTS，负责识别、理解和语音反馈。
- 动作候选：Python 解析 LLM 输出里的 `<speech>` 与 `<action>`，但候选动作还不可信。
- 安全仲裁：C++ ActionGuard 做白名单、schema、限幅、拒绝、急停和 watchdog。
- 动作执行：ROS 2 Action + BehaviorTree.CPP + pluginlib executor 驱动 Gazebo 或 mock。
- 验收观测：Action result、BT status、ACK、`/cmd_vel`、`/odom`、diagnostics、release gate。

关键句：

```text
模型层只负责“建议动作”，控制层负责“决定能不能执行”，执行层负责“可取消、可反馈、可证明地执行”。
```

## 2. 数据流怎么画

```text
Mic PCM
  -> C++ AudioFrontend(AEC/VAD/silence)
  -> ASR online/offline
  -> Wake/retry/hotwords
  -> LLM streaming
  -> speech/action parser
  -> ActionGuard(C++ schema + clamp + reject)
  -> RobotCommand / ExecuteRobotCommand Action
  -> BehaviorTree Validate/Safety/Execute/Confirm
  -> RobotExecutor plugin
  -> Gazebo /cmd_vel -> /odom
```

面试里不用展开所有 topic，除非被追问。优先讲“候选动作”和“可信动作”的边界。

## 3. 为什么 Python 和 C++ 分工

推荐回答：

```text
Python 更适合模型 SDK、文本协议、流式解析和对话编排；C++ 更适合实时音频、动作校验、硬件通信和仿真控制。两边通过 ROS 2 interface 连接，避免模型 adapter 直接碰电机控制。
```

项目证据：

- `embodied_online_agent` 和 `embodied_offline_agent` 处理模型 provider、流式协议、记忆、重试、fallback。
- `embodied_agent_cpp` 处理音频、ActionGuard、UART/SPI、CRC、watchdog。
- `embodied_simulation` 处理 Action server、BT、Gazebo executor、diagnostics。

边界说明：

```text
这不是说 Python 不能做控制，而是这个项目里实时和安全边界更适合收敛到 C++。
```

## 4. 为什么 LLM 不能直接发布 `/cmd_vel`

推荐回答：

```text
因为 LLM 输出是不可信文本，可能格式错误、越界、重复、幻觉，甚至把解释性文字混进动作。`/cmd_vel` 是机器人底盘速度控制入口，一旦直接暴露给模型，就绕过了速度限制、急停、watchdog 和执行结果确认。
```

本项目做法：

- LLM 只输出候选 `<action>`。
- ActionGuard 只接受白名单动作。
- Guard 检查 schema、速度、角速度、持续时间、枚举和缺失字段。
- 真正的 `/cmd_vel` 只由 executor 发布。
- 每个 terminal path 都要求停车。

一句总结：

```text
模型可以参与意图理解，但不能拥有最终执行权。
```

## 5. 为什么用 ROS 2 Action

推荐回答：

```text
`move / turn / stop` 虽然动作简单，但它们不是瞬时请求，而是有持续时间、反馈、取消、超时和最终结果的执行任务，所以比普通 topic 或 service 更适合用 Action。
```

对比：

- topic：适合连续数据流，如音频、雷达、状态、速度输出；但缺少 goal/result 语义。
- service：适合短请求，如 lifecycle transition；但不适合长时间运动和取消。
- action：适合运动任务，可以表达 goal、feedback、result、cancel、timeout、preempt。

项目证据：

- `/robot/execute_command` 承载可取消动作。
- typed bridge 将旧 JSON/typed command 转成 Action goal。
- 测试覆盖成功、取消、阻塞、超时和抢占。

## 6. 为什么同时需要 Lifecycle 和 Behavior Tree

推荐回答：

```text
Lifecycle 管节点资源和启动顺序，回答“这个控制器现在能不能工作”；Behavior Tree 管单次任务的业务流程，回答“这个动作如何验证、安全检查、执行和确认”。它们解决的是不同层面的状态问题。
```

Lifecycle 负责：

- configure 时创建 publisher、subscription、Action server、timer。
- activate 后才接收和转发动作。
- deactivate 时终止活动 goal 并发布零速。
- cleanup 时释放接口和计时器。

Behavior Tree 负责：

- `ValidateCommand`
- `CheckSafety`
- `ExecuteCommand`
- `ConfirmResult`
- RUNNING 时反复检查安全条件。
- 取消、阻塞、超时有明确终态。

面试补充：

```text
我采用的是 Nav2 中常见的工程模式，但没有引入复杂导航栈，而是把 lifecycle、action、BT 和 diagnostics 这些成熟模式用到语音动作执行链上。
```

## 7. 为什么用 pluginlib

推荐回答：

```text
pluginlib 的价值不是“用了设计模式”，而是把执行后端变成可替换、可测试的 `RobotExecutor`。核心 Action/BT/Guard 不需要知道后端是 Gazebo、mock，还是以后接真实机器人。
```

项目证据：

- `GazeboRobotExecutor` 封装仿真控制。
- `MockRobotExecutor` 不依赖仿真或传感器，适合 CI 和快速测试。
- launch 参数 `executor_plugin` 选择后端。
- plugin 测试验证两个 adapter 可发现且行为一致。

一句总结：

```text
后端变化不应该反向污染动作安全和任务编排。
```

## 8. 怎么证明“真的执行了”

推荐回答：

```text
我不把 topic 发布成功当成动作成功。一个动作通过，需要从命令、执行、仿真物理结果三个层面同时有证据。
```

验收证据：

- Guard 产出可信 action command。
- ROS 2 Action 返回 terminal result。
- BT status 到 `confirm/succeeded`。
- executor 返回 ACK 和 backend。
- `/cmd_vel` 出现非零速度。
- Gazebo `/odom` 出现合理位移。

项目记录：

- typed Action/BT 驱动 Gazebo 位移 0.330 m。
- 在线语音 typed 闭环位移 0.163 m。
- 离线语音到 typed Action 到 Gazebo 位移 0.162 m。

边界：

```text
这些是当前 WSL 环境的验收样本，能证明工程闭环可复现，但不能替代实体机器人硬件验收。
```

## 9. 在线和离线链路取舍

在线链路：

- 优点：ASR/LLM/TTS 效果更好，热启动延迟当前样本较低。
- 代价：依赖云 API、网络、密钥和服务稳定性。

离线链路：

- 优点：无网络依赖，适合端侧原型和隐私场景。
- 代价：模型能力弱、CPU 资源敏感、TTS/LLM 链路更复杂。

项目中的共同点：

```text
在线和离线只在 provider 层不同，最终都进入同一套 Guard、Action、BT、executor 和验收链路。
```

## 10. 最重要的工程取舍

- 把主能力收敛在 `move / turn / stop`，先证明闭环，而不是一开始扩展复杂导航。
- 把 LLM 输出和机器人控制隔离，用 Guard 把不可信文本变成可信动作。
- 把动作执行从 topic 升级为 Action，得到取消、反馈、超时和结果。
- 用 Lifecycle 保证启动顺序和未激活门控。
- 用 BT 表达动作执行过程，不把所有分支写死在一个回调里。
- 用 pluginlib 支持后端替换，降低接入新机器人或测试替身的成本。
- 用分层 release gate 证明系统，而不是只展示单次 demo。
