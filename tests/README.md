# 测试结构

测试按“反馈速度”和“系统边界”分层：

| 目录 | 职责 | 是否启动 ROS graph |
| --- | --- | --- |
| `src/<package>/test/` | 包内算法、状态机与 C++ GTest | 否 |
| `tests/repository/` | CLI、目录、接口和 launch 静态契约 | 否 |
| `tests/integration/voice/` | ASR/VAD/KWS、readiness、校准和脚本契约 pytest | 否 |
| `tests/integration/control/` | CLI/cleanup shell 契约、仿真时钟与离线延迟 pytest | 否 |
| `tests/integration/slam_nav/` | SLAM/Nav2 脚本、资产审计、rosbag Adapter 与自动任务触发的 pytest | 否 |
| `tests/evaluation/` | 数据集、回环、后端、消融和报告算法 | 否 |
| `tools/acceptance/probes/<domain>/` | runner 执行的 runtime 验收 Adapter：ROS graph、provider 或外部 API | 由场景决定 |

pytest 会发现并导入 `test_*.py`，但不会执行文件里的 `main()`。因此把 ROS 程序放在 tests 下既会
模糊语义，也会让“pytest 通过”被误解为真实 graph 已验证。新的可执行 probe 必须放入
`tools/acceptance/probes/<domain>/` 且去掉 `test_` 前缀；control、slam_nav、voice 三个领域均使用该
边界。对应 `scripts/smoke_test_*.sh` 或领域 handler 负责环境、ROS domain、进程、日志与清理。
看到 `pytest tests/integration` 通过，仍不能据此宣称 provider、外部 API 或真实 ROS graph 已验收。

`run_probe.sh` 将相对 probe 路径统一解释为仓库根路径，并以无缓冲 `python3 -u` 启动，因此即使
输出经过 `tee` 或重定向，阶段日志也会立即出现。runner 刻意不激活 Python/ROS 环境；直接调用时
必须先完成环境激活，公开 smoke/handler 则在调用它之前负责这一前置条件。

control、slam_nav、voice probe 分别位于 `tools/acceptance/probes/control/`、
`tools/acceptance/probes/slam_nav/`、`tools/acceptance/probes/voice/`。voice 目录既包含连续会话、
KWS/声纹等 ROS probe，也包含在线 API、离线 provider 等非 ROS runtime probe。共享消息 Adapter 位于
`tools/acceptance/typed_action_probe_utils.py`；需要构造 typed action 的 probe 通过生产 transport
复用该 Adapter，不复制第二套测试协议。slam_nav 目录既包含聚焦
mapping/localization/Nav2/动态障碍能力的 probe，也包含
canonical `unknown-world-slam-e2e` 的深模块编排：

- `session_orchestrator.py`：唯一可执行入口与顶层会话编排；
- `cli.py`：被编排器调用的 ROS-free 参数 Interface；
- `session_observer.py`：`SessionObserver` typed ROS Adapter；
- `dynamic_scenario.py`：Nav2 goal、Gazebo 实体、检测与运行时事实的 ROS/Gazebo Adapter；
- `artifacts.py`：地图哈希、失败报告和终端摘要。

Python probe 需要从自身位置推导仓库内日志、配置或地图路径时，统一调用
`tools/acceptance/paths.py:repository_root()`；由 CLI 显式传入完整路径的场景不需要重复定位。该函数
从当前文件或显式起点向上查找稳定仓库标记，不使用易受目录迁移影响的 `parents[n]`，因此主
worktree 和 Git worktree 使用同一定位语义。

清理顺序与异常优先级由 `tools/acceptance/dynamic_scenario_transaction.py` 的 ROS-free 事务拥有，
运行时 Adapter 只注入取消、归位、清检测和验证回调。依赖由编排器单向流向各 Module；业务观测、
阈值与报告 `passed` 统一交给 ROS-free `tools/acceptance/slam_nav_evidence.py`，事务独立强制资源
后置条件。测试目录只验证这些 Interface、纯决策和端到端行为，避免把“可执行探针”与“pytest 断言”
混为一谈。

日常入口：

```bash
bash scripts/acceptance_test.sh --help
bash scripts/acceptance_test.sh verify core
```

局部开发可直接运行：

```bash
pytest -q tests/repository tests/evaluation
pytest -q src/embodied_agent_core/test
```

新增 pytest 应放入所属测试目录；新增可执行 probe 必须放入 `tools/acceptance/probes/<domain>/`，
禁止用 `test_*.py + main()` 混合两种职责。
