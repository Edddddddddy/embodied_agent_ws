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
