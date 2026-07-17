# 测试结构

测试按“反馈速度”和“系统边界”分层：

| 目录 | 职责 | 是否启动 ROS graph |
| --- | --- | --- |
| `src/<package>/test/` | 包内算法、状态机与 C++ GTest | 否 |
| `tests/repository/` | CLI、目录、接口和 launch 静态契约 | 否 |
| `tests/integration/voice/` | ASR/VAD/KWS、脚本契约和语音算法 pytest；旧 probe 迁移中 | pytest 否；遗留 probe 是 |
| `tests/integration/control/` | CLI/cleanup shell 契约、仿真时钟与离线延迟 pytest | 否 |
| `tests/integration/slam_nav/` | SLAM/Nav2 脚本与数据 Adapter 的 pytest；旧 probe 迁移中 | pytest 否；遗留 probe 是 |
| `tests/evaluation/` | 数据集、回环、后端、消融和报告算法 | 否 |
| `tools/acceptance/probes/<domain>/` | 订阅真实 ROS graph 的可执行验收 Adapter | 是 |

pytest 会发现并导入 `test_*.py`，但不会执行文件里的 `main()`。因此把 ROS 程序放在 tests 下既会
模糊语义，也会让“pytest 通过”被误解为真实 graph 已验证。新的可执行 probe 必须放入
`tools/acceptance/probes/<domain>/` 且去掉 `test_` 前缀；control 域已完成迁移，voice/slam_nav 的
遗留 probe 将按领域逐步迁移。对应 `scripts/smoke_test_*.sh` 负责 ROS domain 隔离、进程启动、日志
与清理。看到 `pytest tests/integration` 通过，仍不能据此宣称真实 ROS graph 已完成验收。

`run_probe.sh` 将相对 probe 路径统一解释为仓库根路径，并以无缓冲 `python3 -u` 启动，因此即使
输出经过 `tee` 或重定向，阶段日志也会立即出现。runner 刻意不激活 Python/ROS 环境；直接调用时
必须先完成环境激活，公开 smoke/handler 则在调用它之前负责这一前置条件。

control probe 位于 `tools/acceptance/probes/control/`，共享消息 Adapter 位于
`tools/acceptance/typed_action_probe_utils.py`；它们只通过生产 transport 构造 typed 消息，不复制
第二套测试协议。canonical `slam-nav-e2e` 的主编排实现位于 `tools/acceptance/probes/slam_nav/`：

- `session_orchestrator.py`：唯一可执行入口与顶层会话编排；
- `cli.py`：被编排器调用的 ROS-free 参数 Interface；
- `session_observer.py`：`SessionObserver` typed ROS Adapter；
- `dynamic_scenario.py`：Nav2 goal、Gazebo 实体、检测与运行时事实的 ROS/Gazebo Adapter；
- `artifacts.py`：地图哈希、失败报告和终端摘要。

清理顺序与异常优先级由 `tools/acceptance/dynamic_scenario_transaction.py` 的 ROS-free 事务拥有，
运行时 Adapter 只注入取消、归位、清检测和验证回调。依赖由编排器单向流向各 Module；业务观测、
阈值与报告 `passed` 统一交给 ROS-free `tools/acceptance/slam_nav_evidence.py`，事务独立强制资源
后置条件。测试目录只验证这些 Interface、纯决策和端到端行为，避免把“可执行探针”与“pytest 断言”
混为一谈。

日常入口：

```bash
bash scripts/acceptance_test.sh --help
bash scripts/acceptance_test.sh core
```

局部开发可直接运行：

```bash
pytest -q tests/repository tests/evaluation
pytest -q src/embodied_agent_core/test
```

新增 pytest 应放入所属测试目录；新增可执行 probe 必须放入 `tools/acceptance/probes/<domain>/`，
禁止用 `test_*.py + main()` 混合两种职责。
