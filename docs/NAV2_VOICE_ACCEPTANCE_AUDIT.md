# Nav2 语音目标点导航与多目标点巡航验收审计

本文档用于判断“语音目标点导航 + 多目标点巡航”是否真正完成。它不是开发计划，而是把需求逐条映射到当前代码、自动化证据和仍需人工提供的真实麦克风证据。

## 1. 需求拆解

| 需求 | 当前实现位置 | 自动化证据 | 完成判断 |
| --- | --- | --- | --- |
| 语音/文本命令能表达目标点导航 | `src/embodied_agent_core/embodied_agent_core/command_nlu.py`、`command_fallback.py` | `bash scripts/acceptance_test.sh continuous-navigation` | 已有自动化证据 |
| 支持自然话术，例如“先去门口再去书桌最后回起点” | `command_nlu.py` 的自然多目标解析逻辑 | `bash scripts/acceptance_test.sh continuous-navigation-natural` | 已有自动化证据 |
| 多目标点巡航输出 `follow_waypoints` | `command_nlu.py`、`ros_action_transport.py` | `continuous-navigation`、`continuous-navigation-natural` | 已有自动化证据 |
| 所有动作经过 C++ ActionGuard 限幅与强类型校验 | `src/embodied_agent_cpp/src/action_guard_node.cpp`、`action_validator.cpp` | `bash scripts/smoke_test_typed_action.sh`、`colcon test --packages-select embodied_agent_cpp` | 已有自动化证据 |
| 导航动作通过 ROS 2 Action 可反馈、可等待 result | `src/embodied_agent_cpp/src/typed_action_bridge_node.cpp`、`src/embodied_simulation/src/simulation_control_node.cpp` | `bash scripts/smoke_test_typed_action_server.sh` | 已有自动化证据 |
| `navigate_to/follow_waypoints` 能转成 Nav2 Action goal | `src/embodied_simulation/src/nav2_action_bridge_node.cpp`、语义地点配置 | `bash scripts/acceptance_test.sh nav2-bridge` | 已有自动化证据 |
| 真实 TurtleBot3/Nav2 仿真依赖和 launch 参数正确 | `src/embodied_simulation/launch/voice_nav2_turtlebot3.launch.py` | `bash scripts/acceptance_test.sh nav2-preflight` | 已有自动化证据 |
| 阶段门禁覆盖解析、队列、Nav2 bridge、preflight | `scripts/acceptance_test.sh` | `bash scripts/acceptance_test.sh nav2-stage` | 已有自动化证据 |
| 真实麦克风连续说目标点/巡航命令可被现场验收 | `scripts/continuous_nav2_voice_control.sh`、`continuous_live_check.py`、`continuous_nav2_voice_evidence.sh` | `bash scripts/acceptance_test.sh continuous-nav2-evidence offline` 生成的现场报告 | 需要用户在有麦克风的环境中补充证据 |

## 2. 当前最强自动化门禁

日常开发或合并前建议运行：

```bash
bash scripts/acceptance_test.sh nav2-stage
```

该门禁包含：

- `navigation-demo`
- `continuous-navigation`
- `continuous-navigation-natural`
- `nav2-bridge`
- `nav2-preflight`

如果刚改过 C++ 动作链路，还应补跑：

```bash
bash scripts/smoke_test_typed_action.sh
bash scripts/smoke_test_typed_action_server.sh
bash scripts/smoke_test_simulation.sh
colcon test --packages-select embodied_agent_cpp embodied_simulation --event-handlers console_direct+
```

## 3. 真实麦克风完成标准

只有自动化门禁通过，还不能证明真实语音体验完成。最终完成需要你在 WSL + 麦克风 + Gazebo/Nav2 环境中跑：

```bash
CONTINUOUS_LIVE_CHECK_REPORT=logs/nav2-live-check.json \
  CONTINUOUS_LIVE_CHECK_DURATION=240 \
  bash scripts/acceptance_test.sh continuous-nav2-evidence offline
```

推荐话术：

```text
小智
去门口
前往书桌
依次去门口、书桌、起点
停止巡航
退出控制
```

通过标准：

- 现场报告 `ok=true`。
- 至少出现一次 `navigate_to`。
- 至少出现一次 `follow_waypoints`。
- `navigate_to` 样本里必须带 `target`。
- `follow_waypoints` 样本里必须带两个以上 `waypoints`。
- 观察到 `awake` 和 `sleeping`。
- 最终 `/cmd_vel` 为 0。

复核报告：

```bash
bash scripts/acceptance_test.sh continuous-nav2-live-report logs/nav2-live-check.json
```

注意：当前报告读取只接受最新 schema。旧报告缺少 `asr_samples`、`action_candidate_samples`、`successful_action_samples` 等字段时会直接失败，这是为了避免旧证据误判为完成。

## 4. 当前完成度结论

截至本阶段，代码、自动化测试、Nav2 bridge 和文档入口已经具备完整闭环。还不能把长期目标标记为完全完成的唯一原因是：缺少用户真实麦克风现场跑出的 `continuous-nav2-evidence` 通过报告。

拿到该报告后，若 `continuous-nav2-live-report` 复核通过，即可认为“语音目标点导航 + 多目标点巡航”目标完成。
