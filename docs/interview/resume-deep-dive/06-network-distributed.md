# 网络通信与分布式系统追问

本章先说明事实边界：当前机器人项目直接使用 HTTP 流、SSE 解析和 WebSocket；没有在该仓库中接入 gRPC、Redis、Kafka、MySQL 或 MQTT。后半部分属于技能理解和场景设计，不写成项目实做。

## Q1. TCP、HTTP、WebSocket 和 gRPC 应该怎样区分？

口述：

TCP 提供可靠字节流，但消息边界、序列化和业务协议要自己定义。HTTP 适合请求响应，流式响应可通过 chunk 或 SSE 持续返回；WebSocket 建立后支持全双工长连接，适合持续音频和服务端事件；gRPC 基于 HTTP/2 和 Protobuf，适合强类型的跨服务调用及多种流模式。项目中 LLM 使用 HTTP 流，在线 ASR、TTS 使用 WebSocket。

| 机制 | 适合场景 | 主要代价 |
| --- | --- | --- |
| TCP Socket | 自定义低层协议、设备或服务连接 | 要自行处理帧、重连和版本 |
| HTTP | 一次请求响应、兼容 Web API | 高频双向交互开销较高 |
| WebSocket | 持续双向音频和事件 | 要管理连接、心跳和并发操作 |
| gRPC | 强类型跨语言服务、流式 RPC | 浏览器和外部系统兼容要额外考虑 |

## Q2. 在线 ASR 的 WebSocket 链路怎样运行？

口述：

节点启动时创建实时 ASR 会话并关闭云端自动端点检测，由本地 VAD 控制切句。每个 PCM 帧转为 Base64 后追加到连接；服务端的 transcription text 事件作为 partial，completed 事件作为 final。收到本地 `speech_ended` 后调用 commit，连接仍保留给下一句话，避免每轮重新握手。

源码：[qwen_asr.py](../../../src/embodied_online_agent/embodied_online_agent/providers/qwen_asr.py)

```python
conversation.update_session(
    enable_turn_detection=False,           # 端点由本地 VAD 统一管理
    enable_input_audio_transcription=True,
)

conversation.append_audio(base64.b64encode(pcm16).decode("ascii"))
conversation.commit()                      # 一句话结束，不关闭整个连接
```

取舍：长连接降低每轮建立 TLS 和会话的延迟，但要处理断连、关闭和旧回调。Base64 便于 SDK 事件协议传输，但比二进制帧多占空间。

## Q3. 在线 TTS 为什么需要两个锁和两个 Event？

口述：

连接锁保护“是否已经建立连接”和关闭过程，避免两个线程同时 connect；操作锁保证同一 WebSocket 上一次只合成一段回复，防止文本和音频回调交叉。`session_ready` 等待服务端确认配置完成，`response_done` 等待本次音频流结束。回调线程无论遇到关闭还是异常都要 set 事件，否则调用方会永久等待。

源码：[qwen_tts.py](../../../src/embodied_online_agent/embodied_online_agent/providers/qwen_tts.py)

运行：

1. `connect()` 建立连接并等待 `session.updated`，上限 5 秒。
2. `synthesize()` 在操作锁内追加文本并 commit。
3. `response.audio.delta` 解码后立即调用播放回调。
4. `response.done` 或错误唤醒等待方，上限 30 秒。
5. `finally` 清除本次 `_on_audio`，避免下一轮误用旧回调。

## Q4. LLM 的 HTTP 流是怎样处理的？

口述：

在线和离线 LLM 都使用 OpenAI 兼容接口。client 发送一次聊天请求并打开 `stream=True`，服务端通过流式事件返回增量 token，provider 只 yield 非空内容。在线连接指向云端兼容地址，离线连接指向本机 llama-server。协议适配一致，所以应用层不依赖具体模型部署方式。

源码：

- 在线：[openai_compatible_llm.py](../../../src/embodied_online_agent/embodied_online_agent/providers/openai_compatible_llm.py)
- 离线：[llama_cpp.py](../../../src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py)

取舍：SSE/HTTP 流对现有 SDK 兼容好；如果需要客户端同时持续上传和下载，WebSocket 或双向 gRPC stream 会更自然。

## Q5. 为什么要 warmup？

口述：

第一次请求通常包含 DNS、TCP、TLS、模型加载或 kernel 初始化等冷启动成本。在线 provider 用一个最小请求提前建立连接，离线 llama.cpp 还可以预热模型 kernel 和可复用的系统提示词前缀。warmup 失败应影响 readiness，而不是等用户第一句话时才暴露。

源码：

- [openai_compatible_llm.py](../../../src/embodied_online_agent/embodied_online_agent/providers/openai_compatible_llm.py)
- [llama_cpp.py](../../../src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py)

边界：预热降低首轮抖动，但会增加启动时间和一次调用成本，需要在部署配置中明确。

## Q6. 网络超时和重试怎样设计？

口述：

连接、会话就绪、首 token、总响应和业务任务应使用不同超时。重试前先判断操作是否有外部可见副作用：请求尚未输出 token 时可有限重试；已经输出文本、音频或动作后不能透明重放。错误应分类成连接、超时、HTTP 状态和业务拒绝，方便上层选择离线降级或提示用户。

源码：[llama_cpp.py](../../../src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py)

项目规则：

- client 自带重试关闭，由 provider 明确控制次数。
- 已输出 token 后立即向上抛错，不拼接第二次回复。
- 使用 `time.perf_counter()` 记录首 token 和总耗时。
- 错误信息保留 base URL、模型和状态码，但不打印 API key。

## Q7. Reactor 和 epoll 是什么？项目是否直接实现了？

口述：

Reactor 用一个事件循环等待多个 fd 就绪，再分发读写回调；Linux 常用 epoll 做大规模就绪通知。它适合大量网络连接，避免一个连接一个阻塞线程。当前项目的在线 provider 使用 SDK，底层事件循环由 SDK 管理，没有自行实现 epoll Reactor；项目直接使用 `poll()` 的地方是 UART 可写等待，不应把它描述成完整网络框架。

追问回答：

```text
accept/read/write 都应非阻塞
epoll_wait 返回就绪事件
连接对象保存读写缓冲和协议解析状态
写不完时继续监听 EPOLLOUT
错误、半关闭和超时由连接状态机收口
```

边界：这是个人网络编程能力，代码证据应来自相应服务端项目，而不是本 ROS 2 仓库。

## Q8. ROS 2 通信和 gRPC 有什么不同？

口述：

ROS 2 面向机器人图内的发现、Topic、Service、Action、QoS 和 TF，适合传感器流和设备任务；gRPC 面向明确 client/server 的跨服务接口，使用 Protobuf 和 HTTP/2，适合云端或业务微服务。机器人内部导航任务优先用 ROS Action；机器人向云端账户、模型或调度服务请求数据时可以用 gRPC。两者可以通过网关连接，不需要互相替代。

项目例子：`NavigateToPose` 需要反馈和取消，直接使用 ROS Action；在线模型使用外部 Web API，由 provider 隔离网络协议。

## Q9. WebSocket 和 MQTT 如何选？

口述：

WebSocket 是一条全双工连接，应用自己定义消息语义，适合浏览器或实时音频；MQTT 由 broker 管理发布订阅、主题、会话和不同交付等级，适合设备遥测、状态上报和弱网重连。语音 PCM 和实时 TTS 需要单会话低延迟双向流，WebSocket 更直接；多机器人遥测、设备影子或云端命令分发可考虑 MQTT。

边界：当前仓库没有 MQTT broker、client 或主题设计，面试时只能作为方案比较。

## Q10. Redis、Kafka 和 MySQL 在机器人平台中分别适合什么？

口述：

MySQL 适合需要事务和长期查询的设备、用户、任务配置；Redis 适合短期状态、缓存、幂等键、限流和分布式锁；Kafka 适合高吞吐、可回放的事件流和异步分析。机器人本地的实时控制不应依赖这些外部中间件在线可用，云端可以消费任务事件，但急停和安全限幅必须留在端侧。

一个合理的平台分工：

| 组件 | 可保存内容 |
| --- | --- |
| MySQL | 机器人、地图版本、任务定义、巡检结果索引 |
| Redis | 在线状态、任务租约、去重键、短期会话 |
| Kafka | 遥测、审计事件、离线分析流水 |
| 对象存储 | 地图、bag、图片和大体积日志 |

边界：这只是系统设计回答，本项目没有这些代码。

## Q11. 网络断开时在线语音系统怎样降级？

口述：

先区分当前操作是否已经产生副作用。ASR 断连且尚未得到 final，可以提示重说或切换离线 ASR；LLM 尚未输出 token 时可切换离线模型；已经发布动作后不能重跑整轮，而应继续根据 `command_id` 等待机器人结果。安全命令、基础 NLU 和底盘控制必须能在无网状态下运行。

设计步骤：

1. provider 健康状态进入 readiness。
2. 会话开始时选择 online 或 offline provider，不在半轮中无标识切换。
3. 切换时递增 generation，关闭旧回调。
4. 保留同一动作 ID 的唯一所有者。
5. 将降级原因写入事件和指标。

项目已有在线、离线 provider 和 generation 机制，但自动无缝切换策略仍应按具体产品要求设计。

## Q12. 分布式系统中的幂等性在本项目怎样体现？

口述：

消息可能重复，结果也可能迟到。项目用 `command_id` 拒绝活动或等待队列中的重复命令；结果只有与当前活动 ID 相同才生效；流式 LLM 已输出 token 后不透明重试；地图和验收产物还要求属于本次 session。核心做法是给副作用请求稳定标识，并把“是否已经执行”交给拥有状态的下游判断。

源码：

- [action_scheduler.cpp](../../../src/embodied_agent_cpp/src/action_scheduler.cpp)
- [action_sequence.py](../../../src/embodied_agent_core/embodied_agent_core/action_sequence.py)
- [llama_cpp.py](../../../src/embodied_offline_agent/embodied_offline_agent/providers/llama_cpp.py)

## Q13. Provider 健康检查为什么分部署前和运行后？

口述：

部署前 preflight 检查 Python 模块、模型、知识文件、API key 和可选 endpoint，只读且不启动推理；
它回答“这台机器具不具备启动条件”。Lifecycle configure/warmup 和 ROS readiness 再回答“已加载的
provider 是否真的可服务”。麦克风、VAD 波形和长时间掉线还要由运行时监控判断，不能用一个
`process alive` 或文件存在代替。

源码：

- [voice_runtime_deployment.py](../../../src/embodied_agent_core/embodied_agent_core/voice_runtime_deployment.py)
- [voice_runtime_preflight.py](../../../scripts/voice_runtime_preflight.py)
- [system_readiness_node.cpp](../../../src/embodied_agent_middleware/src/system_readiness_node.cpp)

默认 preflight 不访问网络；只有显式 `--probe-endpoints` 才发送只读 health GET，且不会发起推理。
