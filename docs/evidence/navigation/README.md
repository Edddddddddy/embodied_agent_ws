# Navigation 证据

正式导航证据来自同一次 SLAM/Nav2 session：

- `logs/acceptance/slam_nav/<session_id>/slam_nav_e2e_report.json`
- 同目录 `runtime.log` 与本次 YAML/PGM
- `logs/robotics_acceptance_report.json`

报告必须证明 AMCL、`map→odom`、Nav2 lifecycle、语义 Action result、动态障碍预测/重规划和最终零速度。预置地图、旧日志或单独 planner fixture 不能产生完整链路 PASS。
