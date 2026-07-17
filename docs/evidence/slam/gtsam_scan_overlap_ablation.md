# GTSAM 扫描重叠双证据固定图消融

- 位姿图：`logs/openloris/corridor1-1/scan_overlap_ablation/gtsam_graph_with_scan_overlap.txt`
- SHA256：`17999f541b5b3fca11ae2d40428373a6a8c291b5c2cc86c349d9d838cc56677b`
- 图规模：1834 nodes / 2751 constraints
- 每组匹配位姿：1828
- ATE 最优组：**naive_overlap**
- 公平性检查：**PASS**
- 原始扫描证据：858 条非局部边，unavailable=0
- 重叠率分布：min=0.1714 / median=0.7185 / P95=0.8442 / max=0.8879


| variant | innovation gate | overlap gate | min overlap | min innovation m | used edges | innovation reject | overlap reject | ATE RMSE m | ATE P95 m | RPE RMSE m | final m | ATE vs baseline |
| --- | --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| cauchy_baseline | False | False | 0.65 | 1.0 | 2751 | 0 | 0 | 1.2236 | 2.3968 | 0.1595 | 2.4741 | +0.00% |
| naive_overlap | False | True | 0.65 | 0.0 | 2482 | 0 | 269 | 1.0600 | 2.2293 | 0.1233 | 1.5141 | -13.37% |
| innovation_gate | True | False | 0.65 | 1.0 | 2728 | 23 | 0 | 1.1713 | 2.1770 | 0.1427 | 1.8071 | -4.28% |
| dual_evidence | True | True | 0.65 | 1.0 | 2717 | 23 | 11 | 1.1521 | 2.1985 | 0.1427 | 1.7589 | -5.84% |

> 边界：重叠率来自原始 LaserScan、时间戳关联和静态 TF，不使用真值。该门控位于
> Karto 已接受约束与 GTSAM 后端之间，不生成候选边，也不能证明前端 precision 提升。
> 单帧低重叠不能独立否决 chain-matching 约束，因此保留 naive 组作为反例。
> 当前 0.65 阈值只在 corridor1-1 做过敏感性检查，门控默认关闭；多序列验证前不作为
> 通用参数发布。
