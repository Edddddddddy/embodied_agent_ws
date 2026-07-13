# 10. 故障定位与测试证据

## 先给结论

调试这类系统不能从“机器人没动”直接猜模型问题。应沿数据面逐层找最后一个正确证据，并把“消息已发布、goal 已接受、动作已成功、物理已发生”四种证据分开。

## 四级执行证据

| 级别 | 证据 | 能证明 | 不能证明 |
| --- | --- | --- | --- |
| L1 意图 | ASR final、NLU parse、action candidate | 系统理解出什么 | 命令是否合法/已执行 |
| L2 接受 | Guard output/rejection、Action goal accepted、ACK | 某层接受了请求 | 动作最终成功 |
| L3 终态 | Action result、BT terminal、Nav2 result | 软件执行事务已成功/失败收敛 | 真实硬件物理结果，除非反馈来自硬件 |
| L4 物理 | `/cmd_vel` + odom/Gazebo pose，或 MCU/编码器 ACK | 机器人确实运动/到点 | 模型本身准确率 |

面试说“证明真的执行了”时，至少要讲到 L3；Gazebo 演示最好补 L4。

## 主链观测点

```text
/audio/frontend_metrics
/audio/clean_pcm
/audio/vad_event
/audio/speech_started, /audio/speech_ended
/agent/asr_partial, /agent/asr_final
/agent/wake_event, /agent/recognition_feedback
/agent/nlu_parse, /agent/command_queue, /agent/command_execution
/agent/action_candidate
/robot/action_rejected
/robot/action_command_typed
/robot/action_feedback, /robot/action_result
/robot/bt_status, /robot/simulation_state
/cmd_vel, /odom
/diagnostics
/system/component_health, /system/readiness
```

## “机器人不动”的二分定位

### 1. 没有 ASR final

检查顺序：

1. frontend metrics 的 RMS/peak。
2. clean PCM topic 频率。
3. VAD start/end 是否成对。
4. Agent 是否 active、microphone enabled。
5. endpoint recognition feedback 是否有 duplicate/blocked。
6. online provider WebSocket 或离线 ASR worker 日志。

最后正确证据若是 PCM，没有 endpoint，问题在 VAD/阈值；有 endpoint 无 final，问题在 commit/provider。

### 2. 有 final，没有 candidate

看 wake、recognition、NLU 和 command events：

- `wake_word_not_detected/session_timeout`：会话门控。
- `filler/duplicate_command`：主动过滤。
- `retry`：明确缺槽位。
- queue rejected/expired：容量或 TTL。
- `action_source=blocked`：否定/疑问/危险语义阻断。
- LLM protocol error：标签或 JSON 不完整。

### 3. 有 candidate，没有 guarded command

看 `/robot/action_rejected` 和 ActionGuard health：

- unsupported type/place/color/mode。
- unrelated payload。
- non-finite 或非法 priority。
- downstream buffer full/unavailable。
- Guard inactive。

### 4. 有 guarded command，没有 Action feedback

看 typed bridge diagnostics/readiness：

- action server unavailable。
- scheduler queue full/duplicate ID。
- goal rejected。
- simulation node inactive。

### 5. 有 feedback，最后失败

按 result detail 分：

- canceled：用户优先命令或 Lifecycle。
- timed_out：本地 hard timeout 或 cancel watchdog。
- blocked/front_emergency/scan_timeout：运行时安全。
- nav2 server unavailable/goal rejected/aborted：导航执行层。
- executor_rejected：plugin 不支持或初始化异常。

### 6. Action succeeded，但看不到运动

确认 executor：

- Mock executor 成功不产生真实运动。
- Gazebo navigate 是替代运动，不是路径规划。
- WAVE/LED 在无附件仿真只 ACK 并停止底盘。
- Nav2 executor 自己不发布 `/cmd_vel`，应观察 Nav2 controller 和 odom。

## 并发类故障的识别

### 旧结果唤醒新命令

检查 command ID 是否贯穿 candidate、guarded command、Action feedback/result。`ActionScheduler.complete()` 和 `SequentialActionPublisher._wait_for_result()` 都按 ID 匹配，不应依赖“第几个结果”。

### 取消后仍继续导航

检查两层 generation：

- Python sequence cancel generation 是否唤醒批次等待。
- Nav2 navigate/follow generation 是否使迟到 goal handle 被主动取消。

### deactivate 后仍有 callback

检查：

- `AsrEndpointRuntime` generation/timer 是否取消。
- Agent execution threads 是否 join 成功。
- Nav2 spin thread 是否在 plugin 析构时 cancel/join。
- managed publisher 是否在 STOP 后才 deactivate。

## 测试金字塔

### 纯逻辑单测

最便宜、最稳定，适合边界和状态机：

- NLU 数字/槽位/多命令/危险语义。
- Session filler/duplicate/timeout/sleep/priority stop。
- Queue capacity/TTL。
- Endpoint generation/duplicate。
- Streaming protocol 残缺 tag/非法 JSON。
- ActionValidator 白名单和 clamp。
- ActionScheduler FIFO/cancel/duplicate ID/failure clear。
- `ActionExecution` cancel/block/timeout precedence。
- Controller scan stale/near obstacle/PID/acceleration。
- Hardware frame/CRC/watchdog。

### ROS/C++ 组件测试

验证消息类型、QoS、Lifecycle、Action callback、pluginlib 装载和 pseudo-terminal 系统调用。它能证明跨进程契约，不需要云 API 或 GUI。

### 分层 smoke

常见入口：

```bash
cd /home/ubuntu/embodied_agent_ws
source scripts/activate.sh

bash scripts/acceptance_test.sh mock
bash scripts/acceptance_test.sh cpp-action-client
bash scripts/acceptance_test.sh navigation-demo
bash scripts/acceptance_test.sh nav2-bridge
bash scripts/acceptance_test.sh speaker-runtime
bash scripts/acceptance_test.sh offline-voice-e2e-report
```

以当前 `acceptance_test.sh --help` 和 [TESTING_AND_ACCEPTANCE.md](../TESTING_AND_ACCEPTANCE.md) 为准，脚本模式可能随项目演进。

### 重型/人工验收

- 真实麦克风连续 3 到 5 分钟。
- 云 API 在线 ASR/LLM/TTS。
- Gazebo GUI 和 odom。
- 完整 Nav2 TurtleBot3 map/localization/planner/controller。
- 多人声纹 FAR/FRR。
- 真实 UART/SPI 下位机和物理反馈。

mock 通过不能替代这些证据。

## 为什么测试要围绕“不变量”

比“某函数返回 1”更重要的不变量：

- 同一时刻最多一个 active Action goal。
- priority 只属于 STOP/CANCEL。
- started 必有 finished，busy 最终复位。
- 一旦发布 speech_started，最终必有 speech_ended。
- Lifecycle inactive 不接受新输入，停用前已发布 STOP。
- 迟到 result 不推进新队列。
- unknown speaker 不写用户画像。
- Nav2 完成以外部 result 为准。
- scan stale 时不继续正向运动。

这些不变量直接对应面试里的工程判断。

## 性能指标怎样解释

不要把不同测量范围混在一起：

- LLM first token：请求开始到第一个 token。
- decode tokens/s：首 token 后剩余 token 的生成速度。
- ASR/TTS realtime factor：处理时长与音频时长比。
- sentence TTS synth time：整句返回时间。
- first text to first audio：伪流式管线从第一短句到第一 PCM 块。
- endpoint to first audio：包含 ASR commit、LLM 首 token、切句和 TTS 的 E2E。

当前机器少量样本是阶段报告，不是生产 SLA。

## 事实边界检查清单

写简历或回答前逐项问：

- 这是代码存在，还是运行证据存在？
- 是 mock、fake server、Gazebo，还是实机？
- 是模型原始分数，还是 fallback 后工程出口分数？
- 是单次/少量样本，还是统计分布？
- ACK 表示接受、发送还是实际完成？
- 参数开关存在，后端是否真的 active？
- 哪个日志/report 可以复现？

## 自测问答

### 问：怎样最快定位“没动”？

答：沿 topic/action 链找最后一个正确证据，不要从最上游重跑整个系统。先分清没有 candidate、Guard 拒绝、Action 未派发、执行失败还是只有 mock 无物理运动。

### 问：单元测试能证明 Nav2 导航成功吗？

答：只能证明地点转换、goal 构造、result 映射和状态机。真实到点还需要完整 Nav2 环境、定位、地图、planner/controller 和 odom 证据。

### 问：为什么 command ID 是关键测试点？

答：异步取消和 timeout 后旧回调会迟到。只有 ID/代次关联才能防旧结果完成新请求，这是执行一致性的核心。
