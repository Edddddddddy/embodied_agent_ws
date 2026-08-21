# 高频问答库

## Q1：这个项目一句话怎么介绍？

这是一个基于 ROS 2 的端侧语音机器人控制工程原型。它把麦克风输入、在线/离线 ASR、LLM 动作解析、C++ 安全 Guard、ROS 2 Action/BehaviorTree 执行、Gazebo 仿真或硬件后端串成可测试闭环。重点不是训练一个大模型，而是解决“大模型输出如何安全、可取消、可观测地控制机器人”。

## Q2：项目最核心的亮点是什么？

核心亮点是同一条安全动作 seam 同时连接在线语音、全离线 CPU 模型、Gazebo 物理仿真和硬件传输接口。在线/离线 provider 不同，但最终都进入 C++ ActionGuard、ROS 2 Action、BT 和 executor。这样模型能力、动作安全、执行后端和验收证据是分层的，不会把云模型 demo 和机器人控制混在一起。

## Q3：为什么模型不能直接控制 `/cmd_vel`？

LLM 输出是不可信文本，可能字段缺失、格式错误、参数越界、动作幻觉或重复输出。`/cmd_vel` 是底盘速度入口，直接暴露给模型会绕过限速、急停、watchdog 和执行确认。项目里模型只生成候选动作，C++ ActionGuard 做白名单、schema、限幅和拒绝，最终只有 executor 能发布 `/cmd_vel`。

## Q4：为什么用 Action，不用 topic 或 service？

运动任务有持续时间，需要反馈、取消、超时和最终结果。topic 适合连续数据流，比如音频、雷达和速度；service 适合短请求，比如 lifecycle transition；Action 更适合 `move / turn / stop` 这种需要 goal、feedback、result、cancel 的动作执行。项目里 Action 测试覆盖成功、取消、阻塞、超时和抢占。

## Q5：Lifecycle 和 BehaviorTree.CPP 的区别是什么？

Lifecycle 管节点生命周期，解决“节点什么时候可以工作”：configure 创建资源，activate 后才执行，deactivate 要停车，cleanup 要释放资源。Behavior Tree 管单次动作流程，解决“一个动作如何验证、安全检查、执行和确认”。项目里两者互补：Lifecycle 防止未激活节点执行动作，BT 让动作执行过程可观察、可取消、可插入安全检查。

## Q6：为什么要用 pluginlib？

pluginlib 让执行后端变成可替换的 `RobotExecutor`，核心 Guard、Action 和 BT 不需要知道后端是 Gazebo 还是 mock。这样 CI 可以跑 mock executor，演示可以跑 Gazebo，后续真实机器人可以接新的 executor。它的价值在于把后端差异限制在小 interface 里，而不是把硬件细节散落到控制链。

## Q7：怎么证明机器人真的执行了动作？

我不把“topic 发出去了”当成成功。验收会同时看 Action terminal result、BT `confirm/succeeded`、executor ACK、非零 `/cmd_vel` 和 Gazebo `/odom` 位移。当前记录里 typed Action/BT 驱动 Gazebo 位移 0.330 m，在线语音 typed 闭环位移 0.163 m，离线语音到 Gazebo 位移 0.162 m。这些能证明仿真闭环真实发生，但不能替代实体机器人硬件验收。

## Q8：在线链路和离线链路分别做什么？

在线链路使用 Qwen 实时 ASR、流式 LLM 和实时 TTS，效果和延迟在当前样本下较好，但依赖网络、密钥和云服务。离线链路使用 sherpa-onnx ZipFormer、Qwen3-0.6B Q8/llama.cpp 和 Sherpa-TTS，不依赖网络，更适合端侧原型，但模型能力和本机性能压力更明显。两条链路最终共享同一套 Guard、Action、BT、executor 和验收。

## Q9：项目里的 C++ 主要做了什么？

C++ 负责实时和安全相关的部分：PortAudio 音频前端、AEC/VAD、ActionGuard、动作 schema 校验、限幅、UART/SPI 协议、CRC、watchdog、Lifecycle Action server、BT 执行、Gazebo 控制和 diagnostics。这个分工的原因是模型 SDK 和文本编排适合 Python，但机器人动作安全和硬件接口应该收敛在类型更强、延迟更可控的 C++ 层。

## Q10：项目里的 Python 主要做了什么？

Python 负责在线/离线模型 provider、流式 ASR/LLM/TTS 接入、`<speech>/<action>` 协议解析、对话记忆、识别重试、热词和 fallback。Python 层输出的是候选动作，不直接发布底盘速度。这样模型链路变化不会绕过 C++ 安全仲裁。

## Q11：fallback 是什么，为什么不能当成模型准确率？

fallback 是针对有限机器人命令的工程兜底，用来把常见同义、同音或 ASR 噪声文本映射到白名单动作。当前记录里原始 0.6B 模型在 8 条种子命令上只有 2/8，加入 fallback 后系统链路是 7/8。这说明工程链路可用性提高了，但不能说明模型本身准确率变成 7/8，所以文档里明确区分模型成绩和系统成绩。

## Q12：LoRA 做了吗？

没有完成训练。项目里有 LoRA 种子数据、dataset 注册和配置，但尚未执行训练，也没有独立测试集结果。所以简历和面试里不能写“LoRA 后达到 85%”之类的结论，只能说“准备了训练材料，后续计划用于提升离线模型指令遵循”。

## Q13：当前性能数据能怎么说？

可以说当前 WSL 环境少量样本下，在线热启动 LLM 首 token 350-384 ms，在线 TTS 首音频 222-242 ms，ZipFormer RTF 0.0416，llama.cpp CPU decode 34.10 token/s，离线整轮 2.313 s。必须补一句：这些不是 SLA，只是 2026-07-02 当前机器的单次或少量样本；正式结论需要固定硬件、冻结输入集、保存日志，并报告 100 轮 P50/P95。

## Q14：为什么主链路只做 `move / turn / stop`？

这是一个有意收敛。项目目标是验证语音到可信动作执行的闭环，而不是一开始扩展复杂导航或多 Agent。`move / turn / stop` 足够覆盖 ASR、LLM、Guard、Action、BT、executor、Gazebo 位移和测试验收。先把小动作链路做扎实，再扩展导航、视觉或任务规划，风险更可控。

## Q15：Behavior Tree 在项目里具体做了什么？

BT XML 描述 `Validate -> Safety -> Execute -> Confirm`。Validate 检查命令合法性，Safety 检查雷达和控制状态，Execute 异步执行动作，Confirm 确认结果。ReactiveSequence 让安全条件在 RUNNING 期间反复检查，所以新障碍或取消可以中断执行。状态会发布到 `/robot/bt_status`，用于验收和排障。

## Q16：MultiThreadedExecutor 怎么避免数据竞争？

项目里主控制 callback 和 diagnostics timer 不直接共享可变后端对象。跨线程只共享 atomic 标志和 mutex 保护的快照，例如控制输出、后端状态和诊断字段。diagnostics 只读取快照，不直接调用 executor 的可变控制逻辑。这个设计让状态上报不阻塞动作执行，也降低数据竞争风险。

## Q17：如果 ASR 识别错了，为什么还能执行正确动作？

项目里有热词偏置、唤醒别名、失败反馈和有限命令 fallback。比如同音或噪声导致文本轻微偏差时，fallback 可以把它恢复到白名单动作。但 fallback 之后仍然要经过 C++ Guard，不会绕过安全限制。风险是 fallback 不能无限扩展成字符串匹配系统，正式产品应该接独立声学 KWS，并统计误唤醒和漏唤醒。

## Q18：测试体系怎么设计？

测试是分层的：先测纯 C++ 和 Python 逻辑，再测 ROS mock，再测 lifecycle、typed Action、BT、pluginlib、namespace、component container，最后才跑在线 API、离线模型和 Gazebo。`acceptance_test.sh mock` 是最低门槛，`all` 覆盖 mock、online、offline、Gazebo 和语音到 Gazebo，但不包含真人麦克风。这样能避免每次都依赖云 API 或重型仿真。

## Q19：你遇到的主要难点是什么？

第一个难点是把不可信 LLM 输出变成可信机器人动作，所以需要 C++ Guard、强类型 Action 和明确拒绝原因。第二个难点是不能只验证消息链路，要证明动作真的穿过 BT 并产生 Gazebo 物理位移。第三个难点是在线、离线、mock、Gazebo 后端要复用同一安全执行链，不能各写一套逻辑。

## Q20：这个项目还缺什么？

主要缺四类：第一，LoRA 还没训练，离线模型指令遵循还弱；第二，性能数据还缺固定硬件上的 100 轮 P50/P95；第三，真实麦克风、扬声器、AEC、KWS 和实体 UART/SPI 电机链路还需要目标硬件验收；第四，项目首屏演示和 benchmark 原始日志还可以继续加强。这个项目现在是工程原型，不是生产系统。

## Q21：如果继续做，你优先做什么？

我会优先做三个方向。第一，把语音扰动、同音词、噪声和拒识样本做成可复现 benchmark，记录 ASR、LLM、Guard、BT、Gazebo 每层错误。第二，接独立 KWS 和真实机器人硬件，补齐误唤醒、漏唤醒、AEC 和 UART/SPI 验收。第三，提取更清晰的 TurnCoordinator，让在线和离线 provider 复用同一轮对话编排，减少重复代码。

## Q22：这个项目和普通 ROS 2 demo 的区别是什么？

普通 demo 可能只展示自然语言转一个命令 topic。这个项目更强调工程闭环：模型输出不可信，必须经过 Guard；动作执行用 Action 和 BT 表达反馈、取消和确认；后端可通过 pluginlib 切换；验收需要 ACK、BT status、`/cmd_vel` 和 `/odom` 多重证据。它更像一个可继续接真实机器人和扩展 benchmark 的工程原型。

## Q23：怎么把这个项目写得不夸大？

可以强调“可复现工程原型”“语音到安全动作执行闭环”“在线/离线/Gazebo release gates”“C++ Guard 和 ROS 2 Action/BT”。不要写“工业级”“完整具身智能”“训练出高准确率模型”“稳定低延迟 SLA”“实体机器人已完成全链路”，除非有真实硬件和统计报告支撑。
