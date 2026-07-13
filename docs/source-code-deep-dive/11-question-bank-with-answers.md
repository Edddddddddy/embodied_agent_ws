# 11. 源码深挖问答库

本章集中回答高频追问。建议先遮住答案口述，再回到对应章节和源码验证。

## 架构与 ROS 2

### 1. 为什么项目要拆这么多节点？

答：声卡、云 API、离线模型、动作安全和 Nav2 的故障域、实时性和依赖不同。拆节点可以独立替换与验收，但会增加接口复杂度，所以项目用 typed msg/action、统一 QoS、bringup contract 和 readiness 管理。核心不是“节点越多越好”，而是让进程边界对应真实职责和故障隔离。

### 2. Topic、Service、Action 的选型原则是什么？

答：持续流和广播用 Topic，短请求响应用 Service，长任务且需要反馈、终态和取消用 Action。项目中 PCM 是 Topic，句级 SummerTTS 是 Service，机器人运动和 Nav2 是 Action。

### 3. 为什么命令 topic 是 reliable/volatile？

答：已匹配节点之间需要可靠传输，但重启后不能重放陈旧运动命令，所以不用 transient-local。启动 discovery 前的窗口由有界 TTL outbox 补偿。

### 4. 为什么音频用 best effort？

答：旧 PCM 迟到会累积语音延迟，少量丢帧通常比排队重传更可接受。小队列 best effort 让消费者跟不上时丢旧帧。

### 5. 为什么状态 topic 用 transient-local？

答：状态是可覆盖快照，新监控加入后需要立即看到最新 active/readiness；它不是一次性命令，不会因重放造成动作。

### 6. Lifecycle 解决了什么？

答：把资源创建、开放输入、停止运动和释放资源变成标准状态迁移。尤其 deactivate 固化“关输入、取消、清队列、STOP、等 worker、停 publisher”的安全顺序。

### 7. ACTIVE 和 READY 有何区别？

答：ACTIVE 是 Lifecycle 状态；READY 表示 provider、topic match、Action server 等依赖实际可用且心跳未过期。active 节点也可能因下游断连而 degraded。

## 音频与语音

### 8. 项目怎么做降噪？

答：主链当前实现的是 NLMS 回声消除，利用 `/audio/tts_pcm` 作为扬声器参考预测并减去回声。C++ 通用 noise suppression 和 AGC 尚未实现，参数开启会告警。WebRTC/Silero 是 VAD，不等于降噪；openWakeWord 可选 Speex NS 只作用于该 KWS provider。

### 9. NLMS 相比普通 LMS 有什么区别？

答：NLMS 用参考信号能量归一化权重更新，使有效步长不那么依赖输入幅度。源码按 `step * error / (epsilon + history energy)` 更新每个 tap。

### 10. Energy、WebRTC、Silero VAD 怎么选？

答：Energy 最轻但易被噪声触发；WebRTC 轻量成熟但只有 bool 且限制帧格式；Silero 给概率、通常更稳，但需要 ONNX/JIT 模型和更重运行时。三者共享 endpoint 事件契约，下游不用改。

### 11. 为什么 endpoint 要有开始确认和结束滞回？

答：开始确认过滤碰麦等短噪声；结束用较低阈值和持续静音防止临界概率抖动断句。一旦发布 start，状态机保证配对 end。

### 12. `asr_commit_delay_ms` 有什么用？

答：speech end 后短暂等待 provider/声卡尾部缓冲，减少尾字被截断。timer 受 generation 管理，Lifecycle 停用后不会提交旧句子。

### 13. 为什么 KWS 单独做 sidecar？

答：sherpa/openWakeWord/LiveKit 的依赖和输入不同，但 Agent 只需统一 WakeEvent。sidecar 让模型可替换，也能独立发布 score 做阈值校准。

## 会话、NLU 与模型

### 14. ASR final 为什么不能直接进 LLM？

答：要先处理未唤醒、语气词、重复 final、partial 尾部恢复、命令归一化和急停。固定域命令走本地 NLU还能降低延迟和模型不确定性。

### 15. 连续语音如何避免命令乱序？

答：自然语言先进入有界 FIFO，单个 worker 串行消费；动作批次一次交给 C++ scheduler，scheduler 独占单 active goal 和 pending FIFO；每个终态按 command ID 关联。

### 16. 急停为什么不走普通 FIFO？

答：它需要同时清自然语言队列、取消 Python 批次等待、清 C++ pending 并取消 active goal。只插队无法停止已经在执行的 Action。

### 17. 为什么本地 NLU 用字符 n-gram？

答：中文短命令分词收益有限，字符 n-gram 对轻微措辞差异有容忍度，模型很轻、可解释、可无重依赖运行。槽位仍由确定性 parser 提取。

### 18. “前进 20 米”怎么处理单命令 10 秒上限？

答：NLU 根据距离和速度计算总时长；若用户显式慢速导致超过 10 秒，则拆成多个顺序 move，不静默提速或截断距离。每段仍经过 Guard 和 scheduler。

### 19. 为什么 deterministic parser 能覆盖模型动作？

答：固定命令域中规则结果更可预测、更容易验收。LLM 主要补充未覆盖表达；明确命令不必把最终动作控制权交给随机输出。

### 20. 如何防止损坏的流式输出变成动作？

答：speech 可增量消费，action 必须等完整闭标签后 JSON 解码成功。流结束残缺、非法结构都会产生 protocol error，模型动作不被暴露。

### 21. 什么是伪流式 TTS？

答：LLM 文本先切成短句，每句由离线模型完整合成，再把 PCM 分块发布。TTS 合成和播放可与下一句生成并行，但单句模型不是原生流式。

## 动作安全与执行

### 22. 为什么 LLM 不能直接发 `/cmd_vel`？

答：会绕过白名单、限幅、FIFO、command ID、Action feedback/result、取消、超时、雷达安全和 executor 替换。模型只应输出候选意图。

### 23. typed message 已经有类型，为什么还要 Guard？

答：类型只能保证字段结构，不能防超速、NaN、非法字段组合、未知地点或 priority 滥用。Guard 是业务安全边界。

### 24. Guard 为什么既 clamp 又 reject？

答：连续数值小幅越界可安全收敛到上限；离散语义或互斥字段错误不能猜，否则会改变用户意图，所以拒绝。

### 25. Reliable QoS 为什么还会丢启动期消息？

答：reliable 只作用于已 discovery 匹配的 endpoints。volatile publisher 在订阅者出现前发出的样本不会补发，所以 Guard 用有界 TTL outbox 等 scheduler 匹配。

### 26. C++ scheduler 如何保证单执行？

答：只有一个 optional active 和一个 pending deque。没有 active 才 dispatch；priority 命令先清 pending、取消 active，等旧终态后再派发。

### 27. 取消 result 永远不回来怎么办？

答：bridge 的 cancel watchdog 超时后把旧 ID完成为 TIMED_OUT，scheduler 得以推进 priority 命令。迟到旧 result 因 ID 不匹配被忽略。

### 28. 怎么证明动作真的执行完成？

答：看 ExecuteRobotCommand result 和 BT terminal；Gazebo 再看 `/cmd_vel`、odom 或模型位姿；Nav2 看其原生 result；真实硬件要看下位机/编码器反馈。candidate 或 accepted ACK 只证明前置层接收。

## 仿真与导航

### 29. Action server 和 BT 各负责什么？

答：Action server 提供跨进程 goal/feedback/result/cancel；BT 编排执行端内部的 validate、safety、execute、confirm，并可在每个 tick 重新检查条件。

### 30. pluginlib 的价值是什么？

答：Action server、BT、状态和超时逻辑不变，只替换 Mock/Gazebo/Nav2 executor。这样测试和部署共用上游契约。

### 31. 雷达安全怎么实现？

答：LaserScan 被划分前/左/右扇区取最小有效距离；scan stale 时禁止正向运动，前方小于 emergency distance 时立即将正向速度置零。控制器还做速度和加速度限制。

### 32. “去门口”怎么变成 Nav2 goal？

答：NLU 把中文地点变成 `door`，Guard 白名单校验，Nav2 executor 从 places.yaml 取 map 坐标和 yaw，构造 PoseStamped，发送 NavigateToPose。

### 33. 为什么导航不能按 duration 判成功？

答：goal accepted 后可能还在规划、运动或失败。Nav2 executor 将原生 result 映射到外层 ActiveActionRuntime，本地计时只提供 progress 下限和 hard timeout。

### 34. 取消发生在 goal handle 返回前怎么办？

答：stop 递增 generation。迟到 response callback 发现代次已旧，会主动取消刚收到的 handle，防止后台幽灵导航。

## 记忆、硬件与边界

### 35. 为什么用户身份要做 snapshot？

答：声纹 callback 可能在长 turn 中更新。冻结 identity/preferences 可防止读取 A 的偏好却写入 B 的画像。

### 36. unknown speaker 会写记忆吗？

答：不会。消息先经过 confidence/enrolled 门槛，不可用身份映射 unknown；Store 对 unknown 写入抛 `LowConfidenceSpeakerError`。

### 37. 用户偏好会绕过安全限制吗？

答：不会。偏好只对领域动作做软变换，STOP/CANCEL 不变，后面仍经过 ActionGuard 和 controller 硬限幅。

### 38. UART 帧如何保证完整性？

答：有固定 magic、version、opcode、payload length、sequence 和 CRC16；UART send 处理 partial write/EINTR/EAGAIN，测试用 pseudo-terminal 校验完整 bytes。

### 39. 当前硬件 ACK 能证明什么？

答：只能证明帧成功交给 mock/UART/SPI transport。尚无下位机 ACK 解析和物理反馈，不能宣称真实硬件闭环。

### 40. 项目当前最需要如实说明的边界？

答：C++ 通用 NS/AGC 未实现；LoRA 未训练；Gazebo 替代导航不等于 Nav2；硬件 transport 不等于实机闭环；少量本机性能样本不是生产 SLA；fallback 出口分数不是模型准确率。
