# 架构事实报告

> 本文件由 `scripts/generate_architecture_facts.py` 从仓库确定性生成；不要手工修改。

## 当前规模

| 项目 | 当前事实 |
| --- | ---: |
| ROS 2 package | 12 |
| GitHub Actions 构建 package | 12 |
| 公开验收入口 | 7 |
| 高级帮助入口 | 133 |
| CLI router mode | 133 |
| `scripts/` 顶层文件 | 133 |
| `scripts/` 递归文件 | 133 |
| online Agent 主节点行数 | 549 |
| offline Agent 主节点行数 | 686 |
| 自定义 msg/srv/action 总数 | 37 |

## ROS 2 package 与 CI 矩阵

`embodied_agent_bringup`, `embodied_agent_core`, `embodied_agent_cpp`, `embodied_agent_interfaces`, `embodied_agent_middleware`, `embodied_navigation`, `embodied_offline_agent`, `embodied_online_agent`, `embodied_simulation`, `embodied_slam`, `embodied_slam_tools`, `embodied_voice_frontend`

## 公开验收入口

`continuous-offline`, `continuous-online`, `core`, `gazebo`, `nav2-stage`, `robotics-gate`, `slam-nav-e2e`

## 静态契约

- PASS：`ci_matrix_matches_ros_packages`
- PASS：`ci_push_and_pr_only_dev_main`
- PASS：`public_mode_count_is_7`
- PASS：`shell_entry_is_thin`
- PASS：`public_modes_are_routable`
- PASS：`advanced_modes_are_routable`
- PASS：`release_gates_use_current_workspace`
- PASS：`robotics_profile_covers_required_gates`

## Robotics release gate 覆盖

- PASS：`architecture_facts`
- PASS：`repository_and_agent_units`
- PASS：`required_cpp_packages`
- PASS：`continuous_multi_command`
- PASS：`nav2_stage`
- PASS：`slam_evaluation`
- PASS：`openloris_fixture`
- PASS：`dynamic_obstacle`

## 证据边界

本报告只证明仓库结构、CI/CLI 契约和静态覆盖关系；不证明真实麦克风成功率、模型精度、Gazebo 物理运动或真实场景 SLAM 泛化。

重新生成：

```bash
python3 scripts/generate_architecture_facts.py
bash scripts/acceptance_test.sh architecture-facts
```
