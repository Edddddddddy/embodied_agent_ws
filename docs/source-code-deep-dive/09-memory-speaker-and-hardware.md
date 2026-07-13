# 09. 声纹、用户记忆与硬件 Adapter

## 先给结论

用户个性化和底层硬件都采用 Adapter 模式，但安全边界不同。声纹只决定是否可以读取/写入某个用户画像，不能绕过动作安全；硬件 Adapter 只消费 Guard 后的强类型命令，再编码为带版本、序号和 CRC 的帧，并用本地 watchdog 自动 STOP。

## 声纹 sidecar 的输入与输出

核心文件：[speaker_identity_node.py](../../src/embodied_voice_frontend/embodied_voice_frontend/speaker_identity_node.py)

节点持续收集 `/audio/clean_pcm` 到有界 audio buffer，在 `/audio/speech_ended` 时冻结当前 utterance：

1. 计算 RMS。
2. 清 buffer，避免下一句混入。
3. 如有 enrollment session，保存样本。
4. RMS 低于阈值则发布 unknown/low_audio_rms。
5. Sherpa 模式计算 embedding 并分类；mock 模式发布固定身份。

输出使用 typed `SpeakerIdentity`，包含 top-1、second best、margin、threshold、model 和 reason，不只给一个名字。

## Sherpa speaker embedding

启动时：

1. 创建 `SpeakerEmbeddingExtractor`。
2. 读取 speaker file 中 `speaker_id wav_path`。
3. 每段 WAV 转 mono float waveform。
4. 计算 embedding。
5. 按 speaker 聚合多段 embedding，一次加入 manager。

识别时对每个已注册 speaker 计算相似度，再用 `classify_speaker_scores()` 同时判断：

- top-1 是否超过 threshold。
- top-1 与 second-best 的 margin 是否超过 `min_margin`。

margin 防止两个人分数接近时强行认成 top-1。消息中的 confidence 是模型实际相似度，不用配置阈值冒充。

## 声纹录入

用户说“记住我，我是某某”后，`MemoryCommandService` 可发布 `SpeakerEnrollRequest`。sidecar 创建 enrollment session，要求 1 到 5 段样本：

- 每个 speech end 检查 RMS。
- 保存单声道 int16 WAV。
- 追加到 `speakers.txt`。
- 发布 STARTED/COLLECTING/COMPLETED status。
- Sherpa 模式完成后重新加载 backend。

文本姓名 fallback 只是演示身份起点，真实声纹仍需采集样本和独立 FAR/FRR 评估。

## Agent 为什么冻结用户快照

核心文件：[user_context_runtime.py](../../src/embodied_agent_core/embodied_agent_core/user_context_runtime.py)

声纹 callback 和 LLM/Action worker 是异步的。若一个长 turn 期间 speaker identity 从 A 更新为 B，直接读取全局身份会出现：用 A 的偏好生成动作，却把结果写入 B 的历史。

因此命令入队或开始 direct turn 时调用 `snapshot()`，得到不可变：

```text
identity + MappingProxy preferences + prompt summary
```

同一个 turn 的 prompt、动作偏好和 interaction write 都使用这份快照。

## 可用身份和写入门槛

ROS 消息转领域对象时，[speaker_transport.py](../../src/embodied_agent_core/embodied_agent_core/speaker_transport.py) 只有在：

```text
message.enrolled == true
and confidence >= speaker_identity_min_confidence
```

时才保留 speaker ID，否则映射为 unknown。

`SpeakerIdentity.usable` 还要求 ID 非空且非 unknown。`UserMemoryStore._writable_speaker_id()` 对不可用身份抛 `LowConfidenceSpeakerError`。因此 unknown 用户可以得到普通回答，但不能写入共享画像。

## 两类记忆为什么分开

### ConversationMemory

[memory.py](../../src/embodied_agent_core/embodied_agent_core/memory.py) 保存最近若干 user/assistant message，用于当前对话上下文。它有界、线程安全，并用临时文件加 `os.replace()` 原子保存。

### UserMemoryStore

[user_memory.py](../../src/embodied_agent_core/embodied_agent_core/user_memory.py) 按 speaker ID 保存：

- display name
- preferences
- command counts
- recent interactions
- corrections
- updated time

它有 retention TTL 和 max recent 限制，适合长期画像，不把所有聊天原文无限塞入 prompt。

## 记忆命令

`MemoryCommandService.handle()` 统一处理：

- 我是谁
- 查询偏好
- 记住姓名/开始声纹录入
- 设置移动速度、默认移动时长、默认转角等偏好
- 删除单项偏好
- 清空当前用户记忆

online/offline 节点只负责发布响应、TTS 和 enrollment request，不复制分支逻辑。

## 偏好如何影响动作

核心文件：[user_preferences.py](../../src/embodied_agent_core/embodied_agent_core/user_preferences.py)

偏好不是只拼进 prompt。`apply_user_preferences()` 在候选发布前对确定性动作做纯函数变换：

- slow/fast 缩放 linear/angular speed。
- default move duration 只替换默认值，不覆盖用户显式时长。
- turn 缩放速度时同步调整 duration，尽量保持默认角度。
- arc 同时缩放 linear/angular 并反向调整 duration，尽量保持轨迹形状。
- STOP/CANCEL 永不受个性化影响。

偏好层只是软策略，之后仍经过 ActionGuard 和 controller 的硬限幅。

## 硬件 Adapter 拓扑

核心文件：[hardware_controller_node.cpp](../../src/embodied_agent_cpp/src/hardware_controller_node.cpp)

节点订阅 Guard 后的 `/robot/action_command_typed`，根据参数创建 mock/UART/SPI transport。它与 simulation bridge 是同一 guarded topic 的不同消费者，因此一个部署若同时启用两者，命令会同时送到仿真和硬件。真实部署必须用 launch profile 明确选择所需消费者，不能把默认 mock 组合误认为单后端路由器。

当前硬件节点支持 STOP、MOVE、TURN、WAVE、SET_LED。SET_MODE 和导航不编码到下位机协议。

## 二进制帧协议

核心文件：[hardware_protocol.cpp](../../src/embodied_agent_cpp/src/hardware_protocol.cpp)

帧布局：

```text
0xAA 0x55 | version | opcode | payload_len | sequence_le16 | payload | crc16_le
```

动作编码：

- move：`linear_x * 1000` 转 int16，`duration_s * 1000` 转 uint16。
- turn：`angular_z * 1000` + duration。
- wave：1 byte count。
- set_led：RGB 3 bytes。
- stop：空 payload。

CRC 使用 CRC-16/CCITT 多项式 `0x1021`、初值 `0xffff`。sequence 每帧递增，便于下位机去重/追踪。当前上位机只发帧，没有实现下位机 ACK 解析闭环，所以 `RobotActionAck.STATUS_ACCEPTED` 表示 transport send 接受，不等于物理动作完成。

## UART transport

[hardware_transport.cpp](../../src/embodied_agent_cpp/src/hardware_transport.cpp) 使用 RAII `FileDescriptor`：

- `O_RDWR | O_NOCTTY | O_CLOEXEC | O_NONBLOCK` 打开。
- termios raw mode、配置白名单 baud、8N1、关闭硬件流控。
- `poll(POLLOUT, 100ms)` 等可写。
- 循环处理 partial write、EAGAIN 和 EINTR，直到完整帧发送。

测试用 pseudo-terminal 验证收到的 bytes 与原帧完全相同，比只 mock `write()` 更接近真实 UART 系统调用。

## SPI transport

打开 `/dev/spidev*` 后配置 MODE 0、8 bits、speed，再用 `SPI_IOC_MESSAGE(1)` 做一次全双工 transfer。当前只关心发送，不解析 received bytes。

## MotionWatchdog

MOVE/TURN 发送成功后，watchdog 以 steady clock 记录 deadline。20 ms timer 检查到期就发送 STOP。`expired()` 只返回 true 一次并清 deadline，避免重复 stop storm。

如果 STOP transport send 失败，节点重新 arm 100 ms 后重试。这个 watchdog 防止上层忘记结束有时长运动，但真实下位机仍应有自己的独立失联 watchdog，不能只依赖 Linux 进程。

## 事实边界与改进点

- 已有 UART/SPI 系统调用、帧 CRC、sequence、上位机 duration watchdog 和 mock/pseudo-terminal 测试。
- 尚未证明真实 MCU 接收、执行和回传 ACK。
- 上位机 ACK 当前表示“帧已交给 transport”，不是编码器/里程计证明。
- 更完整协议应增加下位机 ACK/NACK、sequence 对账、重试上限、设备心跳和 MCU watchdog。
- 真实部署应避免 simulation bridge 与 hardware controller 无意并行消费同一命令。

## 自测问答

### 问：声纹更新为什么不能直接改当前 turn 的用户？

答：声纹和 LLM/动作是异步回调，直接读全局身份会产生跨用户读写。命令开始时冻结 snapshot，整个 turn 使用同一 identity/preferences。

### 问：用户偏好会不会突破安全速度？

答：不会依赖偏好层自觉。偏好是纯函数软变换，后面 C++ ActionGuard 和具体 controller 仍做硬限幅；STOP/CANCEL 完全不参与偏好变换。

### 问：硬件 ACK 能证明机器人执行成功吗？

答：当前不能。它只证明上位机 transport send 成功。没有下位机回包和物理反馈时，必须把硬件闭环描述为 mock/接口预留。

### 问：为什么上位机和下位机都应有 watchdog？

答：上位机 watchdog 防应用逻辑忘记停止；下位机 watchdog 防 Linux 进程、链路或整机失联。任何单侧 watchdog 都覆盖不了另一侧故障。
