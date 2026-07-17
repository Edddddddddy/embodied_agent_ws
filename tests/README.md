# 测试结构

测试按“反馈速度”和“系统边界”分层：

| 目录 | 职责 | 是否启动 ROS graph |
| --- | --- | --- |
| `src/<package>/test/` | 包内算法、状态机与 C++ GTest | 否 |
| `tests/repository/` | CLI、目录、接口和 launch 静态契约 | 否 |
| `tests/integration/voice/` | ASR/VAD/KWS、连续会话与语音队列探针 | 由 runner 决定 |
| `tests/integration/control/` | typed Action、Lifecycle、Gazebo 控制探针 | 由 runner 决定 |
| `tests/integration/slam_nav/` | SLAM、Nav2、动态障碍端到端探针 | 由 runner 决定 |
| `tests/evaluation/` | 数据集、回环、后端、消融和报告算法 | 否 |

集成探针只订阅/发布公开 topic、Action、diagnostics、TF 和 odom；对应
`scripts/smoke_test_*.sh` 负责 ROS domain 隔离、进程启动、日志与清理。
其中一部分 `test_*.py` 是带 `main()` 的可执行 probe，并不会被 pytest 自动收集；它们必须由
`tools/acceptance/run_probe.sh` 或公开验收入口启动。看到 `pytest tests/integration` 通过，不能据此
宣称真实 ROS graph 已完成验收。

`run_probe.sh` 将相对 probe 路径统一解释为仓库根路径，并以无缓冲 `python3 -u` 启动，因此即使
输出经过 `tee` 或重定向，阶段日志也会立即出现。runner 刻意不激活 Python/ROS 环境；直接调用时
必须先完成环境激活，公开 smoke/handler 则在调用它之前负责这一前置条件。

canonical `slam-nav-e2e` 的主编排实现不再伪装成 `test_*.py`，而位于
`tools/acceptance/probes/slam_nav/`；其他较小的 ROS 集成 probe 仍按领域保留在 `tests/integration/`：

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

新增测试应放入所属领域目录，禁止重新把 `test_*.py` 堆回 `tests/integration/` 根目录。
