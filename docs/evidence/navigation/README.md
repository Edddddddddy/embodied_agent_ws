# Navigation 证据

## 会话级 E2E 证据

正式导航证据必须来自同一次 `slam-nav-e2e` session：

- `logs/acceptance/slam_nav/<session_id>/slam_nav_e2e_report.json`
- 同目录 `runtime.log` 与本次 YAML/PGM

报告必须证明 AMCL、`map→odom`、Nav2 lifecycle、语义 Action result、动态障碍预测/重规划和最终零速度。预置地图、旧日志或单独 planner fixture 不能产生完整链路 PASS。

## 聚合门禁

`logs/robotics_acceptance_report.json` 是 `robotics-gate` 对 repository、C++、Nav2 stage、公开 bag 和
动态障碍 stage 的聚合门禁结果。它适合证明各模块门禁均通过，但不运行同一次完整 Gazebo session，
因此不能独立证明 fresh map、AMCL、动态横穿重规划和最终停车的 E2E 事实。
