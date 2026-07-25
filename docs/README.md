# 文档阅读地图

这个项目同时涉及语音 Agent、ROS 2/C++ 控制、Gazebo、SLAM 和 Nav2。内容不少，但不需要从头
到尾读完所有文件。先确定自己要“跑项目、看架构、学原理，还是核验证据”，再沿下面的路径阅读。

## 第一次接触项目

1. 先读仓库根目录的 [README](../README.md)，理解项目解决什么问题、最短部署方式和公开验收入口。
2. 再读 [系统架构](ARCHITECTURE.md)，把语音输入、动作安全、仿真执行、建图和导航串成一条数据流。
3. 需要实际运行时转到 [测试与验收](TESTING.md)，按证据层级选择命令，并理解什么结果才算 PASS。

根 README 是产品入口，不承担算法教材或历史档案的职责；遇到细节应继续进入对应专题，而不是在
README 中寻找所有答案。

## 想读懂代码

[系统架构](ARCHITECTURE.md) 维护稳定的模块职责、上下游接口、主数据流和信任边界。建议先看模块
所有权，再顺着主数据流定位代码。易变的测试数量、单次实验指标和开发计划不属于这份文档。

随后按兴趣选择一份学习笔记：

- [语音 Agent](learning/VOICE_AGENT.md)：音频前端、VAD/ASR、连续会话、NLU、在线与离线 provider。
- [语音运行时部署](deployment/VOICE_RUNTIME.md)：在线/离线 profile、全栈 preflight、RAG 与降级边界。
- [ROS 2/C++ 控制](learning/ROS2_CPP_CONTROL.md)：typed Action、ActionGuard、调度、Lifecycle、BT 和 pluginlib。
- [SLAM/Nav2](learning/SLAM_NAV2.md)：frontier 探索、地图保存、AMCL、规划、动态障碍、回环与后端优化。

三份笔记使用同一讲解顺序：场景和问题 → 调用链与代码锚点 → 设计原理 → 为什么这样做 → 替代方案
→ 常见故障 → 对应测试。它们负责帮助读者理解，不负责声明某次运行已经成功。

## 想运行或验收

[测试与验收](TESTING.md) 是命令、PASS 判定、产物位置和故障分层的唯一说明。先选与目标匹配的证据
层级：单元测试、mock、Gazebo、unknown-world、真人语音和公开数据不能互相替代。

[证据索引](evidence/README.md) 保存已经产生的可复核事实，并继续按能力分为：

- [语音证据](evidence/voice/README.md)
- [离线模型证据](evidence/offline/README.md)
- [SLAM 证据](evidence/slam/README.md)
- [导航证据](evidence/navigation/README.md)

证据文档回答“哪次运行证明了什么”；测试手册回答“怎样重新产生并判定证据”。历史 PASS 不自动
证明新提交、新地图或新接口仍然通过。

## 想准备演示或面试

[15 分钟汇报稿](PRESENTATION_15MIN.md) 给出演示节奏和代码讲解顺序。它从听众视角组织内容，不作为
架构或测试事实源。简历表述可参考 [ROS 2/C++ 项目经历](interview/04-ros2-cpp-resume.md)，最终描述
仍应以自己实际跑过的证据为边界。语音模块的论文、部署取舍与代码追问集中在
[语音部署与 RAG 深挖](interview/resume-deep-dive/11-voice-deployment-rag.md)。

## 想继续开发

当前工程记录位于 [development](development/)；其中的 Goal、决策、工程日志和踩坑记录用于恢复
开发上下文，不是稳定用户文档。正在进行的多模态演示工作从
[开发档案入口](development/multimodal_showcase/README.md) 开始阅读。

[历史变更](history/CHANGELOG.md) 只解释版本如何演进。旧命令、旧指标和旧架构可能已经失效，不能
把历史记录当成当前实现说明。

## 文档维护约定

- 根 README 只保留项目定位、最短部署和公开入口。
- ARCHITECTURE 只维护稳定接口、模块所有权和数据流。
- TESTING 只维护命令、PASS 条件、产物与排障路径。
- learning 负责代码锚点、原理、设计理由、替代方案和故障分析。
- PRESENTATION 负责讲稿；evidence 负责事实；development 负责当前工程过程；history 负责过去。
- 同一事实只设一个所有者，其他文档使用链接，不复制会漂移的命令、阈值和统计值。
