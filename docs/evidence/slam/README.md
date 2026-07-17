# SLAM 证据

本目录保留可复核的小型公开数据集结果：

- `lidar_loop_candidates_*`：回环候选。
- `lidar_shadow_matches_*`：shadow matcher。
- `lidar_submap_*`：submap 匹配与消融。
- `lidar_temporal_*` / `lidar_sequence_*`：时序与跨序列约束。
- `gtsam_*`：scan-overlap、一致性门、鲁棒核与 switchable constraint。

不同 sequence 与不同算法阶段是消融链上的独立证据，不互相覆盖。真实主演示报告位于
`logs/acceptance/slam_nav/<session_id>/`，必须绑定本次地图哈希、world/任务配置和代码版本。公开 bag 与 synthetic fixture 不能冒充真实场地漂移。
