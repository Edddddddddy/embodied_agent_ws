# Navigation 证据索引

本目录只解释证据口径，不存放运行产物。`logs/` 不提交 Git；PR 在描述中引用本机 session 路径和摘要。

## 1. 会话级 E2E 证据：Unknown-world 正式闭环

正式“自主建图 → 定位 → 规划 → 避障”证据必须来自一次：

```bash
bash scripts/acceptance_test.sh unknown-world-slam-e2e
```

```text
logs/acceptance/unknown_world_slam_nav/<session_id>/
├── unknown_world_slam_e2e_report.json
├── runtime.log
├── unknown_world_map.yaml
├── unknown_world_map.pgm
└── acceptance_session.json
```

报告必须满足：

- `schema_version=4`、`evidence_kind=unknown_world_slam_nav_dynamic_replan`、`passed=true`；
- 地图来自本次 session，并通过总体/分区覆盖与 reachable unknown 检查；
- typed frontier 证据证明没有 active/reachable/blacklisted frontier；
- `mission_outcome=SUCCEEDED`，而不是只看当前 `phase`；
- AMCL 与 evaluator-only Gazebo truth 的定位质量通过；
- 至少 3 个本次地图动态采样目标成功，producer 与 evaluator 都确认 plan 不穿 unknown/occupied/map 外；
- 动态障碍重规划和最终零速度通过。

静态 truth map、场地区域和 Gazebo pose 仅供 `unknown_world_evidence.py` 离线评分。它们不得进入 robot
policy、mission YAML、frontier 恢复或目标抽样，否则证据失效。

当前发布候选 session `20260720T031306Z-1114546-814430c3` 已取得 schema v4 PASS：总体覆盖
`99.6657%`，最低分区 office `97.7593%`，reachable unknown `0.3343%`，障碍边界召回/false-free
`80.9322% / 0.2119%`；frontier available/active/blacklisted 为 `0/0/0`、accepted/terminal 为
`20/20`；AMCL/Gazebo 246 个对齐样本的 P95 为 `0.154311m`；3/3 目标成功、最小间距
`5.570m`，路径 unknown/occupied/map-outside 均为 0；动态重规划与终态新鲜零速通过。

诊断会话继续保留：`20260720T013943Z-1054618-0887ac81` 暴露了 `0.605m` 盲袋与厨房仅
`25%` 覆盖；`20260720T025009Z-1101237-7f8143c7` 暴露了 `0.38m` 观测近失与真实绕行被误认为
连续静止。修复后的 Burger profile 使用 `0.40m` 观测容差（上游默认仍为 `0.30m`），且在
reached 前复核最新地图安全性；meaningful escape 会打断连续静止熔断，Nav2 实际值
`0.10m/30s` 写入会话 YAML 留档。这些 FAIL 是根因链证据，不会从索引中伪删除，也不能与 PASS 指标混用。

## 2. Known-world 回归与语音演示

```bash
bash scripts/acceptance_test.sh slam-nav-e2e
bash scripts/acceptance_test.sh voice-slam-workplace-demo offline
```

旧回归报告位于：

```text
logs/acceptance/slam_nav/<session_id>/slam_nav_e2e_report.json
```

该 profile 可以使用 bootstrap 路线和语义地点，适合证明固定场景的集成稳定性和真人语音交互；不能
替代 unknown-world 自主探索证据。两个目录、schema 和 `evidence_kind` 不可混用。

## 3. 聚合门禁

`logs/robotics_acceptance_report.json` 是 `robotics-gate` 对 repository、C++、Nav2/SLAM stage、公开 bag
与动态障碍 stage 的聚合结果。它适合证明模块门禁，但不运行同一次完整 unknown-world Gazebo session，
因此不能独立证明 fresh map、真值隔离、定位 P95、动态采样目标和最终停车。

本阶段修改了 ROS 2 interface type hash。旧 overlay/旧 rosbag 即使可读，也不包含当前
`FrontierExplorationEvidence`、`SlamNavigationGoalEvidence` 与独立 `mission_outcome`，不得作为当前
strong typed 证据。
