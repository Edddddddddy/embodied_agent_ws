# 面试材料索引

这组文档面向面试使用，不是项目 README 的重复版。阅读顺序建议按“先会讲，再能被追问，再落到简历”来走。

## 速查顺序

1. 先读 `01-project-pitch.md`
   - 目标：准备 30 秒、1 分钟、3 分钟三种开场。
   - 面试官问“介绍一下你的项目”时，优先用 1 分钟版本。

2. 再读 `02-architecture-deep-dive.md`
   - 目标：能解释数据流、分层、ROS 2 Action、Lifecycle、BehaviorTree.CPP、pluginlib 的选择原因。
   - 面试官开始问“为什么这么设计”时，用这里的回答骨架。

3. 练 `03-interview-qna-bank.md`
   - 目标：覆盖高频追问，尤其是安全、测试、模型边界和 ROS 2 通信机制。
   - 回答时先给结论，再给项目证据，最后补边界。

4. 最后改 `04-resume-bullets.md`
   - 目标：把项目压缩成简历 bullet，并避免夸大。
   - 所有数字都来自 `../TESTING_AND_ACCEPTANCE.md` 当前记录。

5. 需要追到代码实现时读 `../code-walkthrough/README.md`
   - 目标：从节点、回调、线程、状态机一直追到 `/cmd_vel`、硬件协议和 Nav2。
   - 每章都包含源码入口、失败分支、测试证据、面试回答和闭卷自测。

## 主项目定位

项目名称可写为：

```text
Embodied Voice Agent for ROS 2
```

中文面试表述：

```text
基于 ROS 2 的端侧语音机器人控制系统
```

一句话定位：

```text
这是一个面向 TurtleBot3 和端侧机器人的语音控制工程原型，把麦克风输入、在线/离线 ASR、LLM 动作解析、C++ 安全仲裁、ROS 2 Action/BehaviorTree 执行、Gazebo 仿真或硬件输出串成了可测试闭环。
```

## 面试叙事主线

推荐始终围绕这一条主线讲：

```text
麦克风 / ASR / LLM 动作解析
  -> C++ ActionGuard 做 schema 校验、限幅、拒绝和急停
  -> ROS 2 Action 承载可取消、可反馈、可超时的动作执行
  -> BehaviorTree.CPP 做 Validate / Safety / Execute / Confirm 编排
  -> pluginlib 切换 Gazebo 或 mock executor，后续可接 UART/SPI
  -> 用 ACK、/cmd_vel、/odom、BT status 和 release gate 证明闭环真实发生
```

## 应主动说明的事实边界

- 当前是可复现工程原型，不是生产级机器人系统。
- 主链路聚焦 `move / turn / stop`，不包装成复杂自主导航或完整具身智能。
- LoRA 数据和配置已准备，但尚未训练；不能宣称训练后准确率。
- 原始 0.6B 模型种子集动作为 2/8，fallback 后为 7/8；fallback 成绩是工程链路成绩，不是模型准确率。
- 延迟数据来自 2026-07-02 当前 WSL 环境的单次或少量样本，不是 SLA。
- UART/SPI interface 有实现和 mock/PTY 测试，但实体 MCU、电机、急停硬件仍需目标机器人验收。

## 面试中不要这样说

不要说：

```text
我实现了完整工业级机器人语音控制系统。
```

建议说：

```text
我实现了一个可复现的 ROS 2 语音机器人控制工程原型，重点验证语音到安全动作执行的闭环。
```

不要说：

```text
模型指令准确率达到 7/8。
```

建议说：

```text
原始模型在小种子集上只有 2/8，加入有限命令 fallback 后系统链路达到 7/8，所以我在文档里把模型能力和工程兜底明确区分。
```

不要说：

```text
离线系统稳定低于 3 秒。
```

建议说：

```text
当前机器少量样本下离线整轮约 2.313 秒，但正式结论还需要固定硬件和 100 轮 P50/P95。
```

## 临场回答策略

- 如果问“项目亮点”：讲同一条安全动作 seam 同时连接在线、离线、Gazebo、硬件输出。
- 如果问“你负责什么”：讲从链路设计、C++ Guard、ROS 2 Action/BT、测试验收到文档边界。
- 如果问“难点”：讲 LLM 不可信输出如何变成可验证动作，以及如何证明机器人真的执行了。
- 如果问“还有什么不足”：讲 LoRA、P95、实体硬件、独立 KWS，这会显得更可信。
