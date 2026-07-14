# GTSAM 扫描重叠双证据固定图消融

- 序列：`corridor1-2`
- 位姿图：`logs/openloris/corridor1-2/scan_overlap_ablation/gtsam_graph_with_scan_overlap.txt`
- SHA256：`fa49951b7424b1bad120690b1e518f977f259169a7d61f8371f05bf24723765f`
- 图规模：488 nodes / 491 constraints
- 每组匹配位姿：488
- ATE 最优组：**cauchy_baseline**
- 公平性检查：**PASS**
- 原始扫描证据：1 条非局部边，unavailable=0
- 重叠率分布：min=0.8546 / median=0.8546 / P95=0.8546 / max=0.8546


| variant | innovation gate | overlap gate | min overlap | min innovation m | used edges | innovation reject | overlap reject | ATE RMSE m | ATE P95 m | RPE RMSE m | final m | ATE vs baseline |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| cauchy_baseline | False | False | 0.65 | 1.0 | 491 | 0 | 0 | 0.1484 | 0.2792 | 0.0644 | 0.4036 | +0.00% |
| naive_overlap | False | True | 0.65 | 0.0 | 491 | 0 | 0 | 0.1484 | 0.2792 | 0.0644 | 0.4036 | +0.00% |
| innovation_gate | True | False | 0.65 | 1.0 | 491 | 0 | 0 | 0.1484 | 0.2792 | 0.0644 | 0.4036 | +0.00% |
| dual_evidence | True | True | 0.65 | 1.0 | 491 | 0 | 0 | 0.1484 | 0.2792 | 0.0644 | 0.4036 | +0.00% |

> 边界：重叠率来自原始 LaserScan、时间戳关联和静态 TF，不使用真值。该门控位于
> Karto 已接受约束与 GTSAM 后端之间，不生成候选边，也不能证明前端 precision 提升。
> 单帧低重叠不能独立否决 chain-matching 约束，因此保留 naive 组作为反例。
> 单序列结果不决定发布状态；默认启用与否由独立固定图的多序列报告统一裁决。
