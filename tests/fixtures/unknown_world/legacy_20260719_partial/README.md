# legacy_20260719_partial

该 fixture 固化自验收场次 `20260719T132544Z-192184-6da598f3`。旧报告按原门槛为
`PASS`，但地图仍有大面积未观测区域，因此必须被 unknown-world 地图质量契约判为
`INCOMPLETE`。

- `legacy_report.json`：保留原始报告内容，报告内的绝对路径仅作为历史记录。
- `built_map.yaml` / `built_map.pgm`：本次 SLAM 产物；YAML 使用相对图像路径，测试可移植。
- `truth_map.yaml` / `truth_map.pgm`：仅供验收器离线评分，不得注入探索或导航运行时。

真值隔离是本 fixture 的安全边界：测试可以量化覆盖率，机器人策略不能据此选择目标。
