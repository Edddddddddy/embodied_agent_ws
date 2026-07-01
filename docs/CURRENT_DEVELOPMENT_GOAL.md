# 当前开发目标：语音到基础仿真动作闭环

## 目标

当前只聚焦一条可重复验证的主链路：用户说出简单运动指令，系统经过 ASR、LLM、动作解析和安全校验后，让 Gazebo 中的 TurtleBot3 执行 `move/turn/stop`。

```text
语音 PCM
  -> ASR final
  -> LLM <speech>/<action>
  -> /agent/action_candidate
  -> C++ ActionGuard
  -> /robot/action_command
  -> SimulationController
  -> /robot/action_ack + /cmd_vel
  -> Gazebo TurtleBot3
  -> /odom 位移或转角证据
```

## 当前完成定义

一次基础动作只有同时满足以下条件才算全链路通过：

1. `/agent/asr_final` 收到非空识别文本。
2. LLM 或确定性仲裁生成合法 `move/turn/stop`。
3. C++ Guard 在 `/robot/action_command` 发布经过 schema 检查和限幅的命令。
4. 仿真执行器在 `/robot/action_ack` 返回 `backend=simulation` 和对应 action。
5. `/cmd_vel` 出现合理速度，雷达危险或失联时正向速度必须为 0。
6. Gazebo `/odom` 出现物理合理的变化，不接受跨世界或多进程造成的里程计跳变。
7. `stop` 后切回 manual 并保持停车，不允许下一周期自行恢复。

自动测试 `bash scripts/acceptance_test.sh gazebo-voice` 已覆盖 1–6；控制器 GTest 覆盖第 7 条和安全边界。真实麦克风需要按本文末尾清单人工验收。

## 当前支持的语音动作

- “向前走一秒” -> `move(linear_x=0.2, duration_s=1.0)`
- “后退半秒” -> `move(linear_x=-0.2, duration_s=0.5)`
- “左转九十度” -> `turn(angular_z=0.6, duration_s=2.6)`
- “右转一秒” -> `turn(angular_z=-0.6, duration_s=1.0)`
- “马上停下” -> `stop()`

## 暂不作为当前目标

- Nav2 全局规划、地图目标点和复杂任务编排。
- 机械臂、抓取、多机器人和复杂动作序列。
- 键盘、语音、导航之间的通用控制权仲裁。
- 自动避障、沿墙行为的复杂场景轨迹优化。

已有的基础避障和沿墙代码保留，但后续迭代优先保证基础语音动作链的正确性、可观测性和稳定性。

## 一条命令启动

离线：

```bash
cd /home/ubuntu/embodied_agent_ws
bash scripts/run_voice_simulation.sh offline
```

在线：

```bash
bash scripts/run_voice_simulation.sh online
```

脚本默认打开 Gazebo GUI、麦克风和扬声器。在线模式读取 `.env`；离线模式会复用或自动启动本地 llama-server。

## 真实麦克风验收

推荐直接运行交互式验收器：

```bash
# 默认关闭唤醒门控和扬声器，单独验证核心语音动作链
bash scripts/accept_voice_simulation_microphone.sh offline

# 在线模型
bash scripts/accept_voice_simulation_microphone.sh online

# 核心链通过后，再把唤醒词纳入验收
WAKE_WORD_ENABLED=true \
  bash scripts/accept_voice_simulation_microphone.sh offline
```

脚本会检查 WSL 麦克风、启动模型与 Gazebo，并逐层显示 `[1/6]` ASR、`[2/6]` 动作候选、`[3/6]` Guard 后命令、`[4/6]` simulation ACK、`[5/6]` Twist、`[6/6]` Odometry。默认提示说“向前走一秒”；开启唤醒门控后提示说“小智，向前走一秒”。失败时会列出未完成环节并附启动日志。

也可以手工验收：

1. 确认 Gazebo 中只有一个 TurtleBot3，终端没有重复 `/clock` 警告。
2. 说“小智，向前走一秒”。
3. 依次观察：

```bash
ros2 topic echo /agent/asr_final
ros2 topic echo /agent/action_candidate
ros2 topic echo /robot/action_command
ros2 topic echo /robot/action_ack
ros2 topic echo /cmd_vel
ros2 topic echo /odom
```

4. ACK 应包含 `{"action":"move","backend":"simulation","status":"accepted"}`。
5. TurtleBot3 应短距离前进并自动停止；前方小于安全距离时不得前进。
6. 再分别测试左转、右转、后退和停止。

自动合成语音验收不能替代真实麦克风的噪声、回声、唤醒词和设备延迟测试。

## 麦克风验收常见问题

### ASR 把“小智”识别为近音字

离线 ZipFormer 已启用 sherpa-onnx 的 `modified_beam_search` 和 hotword
contextual biasing，热词表位于
`src/embodied_offline_agent/config/hotwords_zh.txt`。唤醒词与运动命令会在
声学解码阶段获得额外分数，这比不断罗列识别结果更稳健。别名仍作为最后一道兼容层：

```yaml
wake_word_aliases: ["小志", "小治", "晓智", "晓志"]
```

门控仍未通过时不会卡死：节点发布 `/agent/recognition_feedback`，状态切到
`retry_listening`，验收终端显示“请再说一次”及次数，然后继续接收下一段语音。
`recognition_max_retries` 只控制计数周期，不会停止监听。

生产级进一步方案是 sherpa-onnx open-vocabulary Keyword Spotting：在 ASR 前用
独立声学 KWS 判断唤醒词，并调节 boost/trigger threshold。当前版本优先采用无需新增
模型、部署成本最低的 hotword biasing；需要评估误唤醒率/漏唤醒率时再切换 KWS。

不建议使用不受约束的模糊匹配，否则普通中文短语可能误唤醒。若现场出现新的稳定同音结果，把它加入在线和离线 YAML，再补 WakeWordGate 单测。

### 在线正确识别“向前走一秒”但没有动作

在线 launch 现在显式接收并覆盖 `wake_word_enabled`。默认核心链验收为 false，所以无需说“小智”；开启后必须说唤醒词或配置别名。可单独检查：

```bash
bash scripts/smoke_test_online_wake_config.sh
```

### 完成一条命令后仍持续出现 ASR final

Agent 在 LLM/TTS turn 忙碌期间会丢弃新的音频帧、静音 commit、partial 和重叠 final；本轮结束后才恢复监听。这避免扬声器回声或尾音形成“嗯”等第二条指令。若希望连续对话，应等待状态重新变为 `listening`。
