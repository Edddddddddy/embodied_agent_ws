# 多模态演示开发档案

本目录记录“SLAM 建图、Nav2 导航、自然语言/语音控制、键盘接管”统一演示的工程过程。它面向开发评审与复盘，不替代项目 README 或用户验收手册。

## 当前结论

- `main` 当前稳定版本为 `v0.5.0`（`4125d4b`），不包含持久会话改动。
- 控制平面 PR #89 已合入 `dev`（`2d91ffe`）。
- `feature/demo-persistent-session` 已实现同一 Gazebo、机器人和 Agent 会话内的
  `mapping -> navigation` 切换；本地单元/ROS 包测试与 fresh
  `showcase-gazebo-e2e` 重型门禁均已通过。
- fresh session `20260724T053935Z-1431080-d6efcab6` 用时 `1515 s`：
  总覆盖率 `0.998`、最弱区域 `0.985`、AMCL P95 `0.125 m`、3 个导航目标、
  动态重规划、运行时连续性和 STOP 后最终零速全部通过。
- 该证据来自 dirty feature worktree，证明当前实现闭环，但仍需整理提交、PR 到
  `dev` 和 CI；它不是已经发布到 `main` 的版本。

本轮 fresh 事实源保存在本地运行产物目录：

```text
logs/acceptance/showcase_gazebo_e2e/20260724T053935Z-1431080-d6efcab6/
```

其中包含最终报告、会话清单与运行日志。`logs/` 不进入 Git；正式发布证据需要在
clean commit 上重跑，并把精简结果同步到受版本控制的证据文档。

2026-07-21 的 session `20260721T072342Z-2344751-5452a492` 是
`voice-unknown-world-e2e` 工作树上的 strict 基线：覆盖率 `0.998`、最弱区域
`0.988`、AMCL P95 `0.120 m`、3 个导航目标和最终零速均通过。它使用 mock
provider，产生于本轮 persistent 实现之前，不含 `runtime_continuity`，因此既不能
证明本分支的跨阶段进程连续性，也不能证明真实麦克风识别效果。

## 文件导航

- [PLAN.md](PLAN.md)：交付顺序、分支策略、演示流程和门禁。
- [DECISIONS.md](DECISIONS.md)：已经接受的架构决策。
- [ENGINEERING_LOG.md](ENGINEERING_LOG.md)：按时间记录事实、失败与测试证据。
- [PITFALLS.md](PITFALLS.md)：可复现的故障模式及防回归措施。
- [REFERENCES_AND_TOOLS.md](REFERENCES_AND_TOOLS.md)：使用过的资料、工具和诊断命令。

## 维护规则

1. 代码、测试和对应记录进入同一个功能 PR，目标分支为 `dev`。
2. 只记录可由提交、日志、报告或测试证明的事实；计划项明确写“待实现”。
3. synthetic/mock、真实 Gazebo、真实麦克风三类证据不能相互替代。
4. 一个完整功能闭环后再 push 触发 CI；`main` 只接收经过 `dev` 集成的发布版本。
