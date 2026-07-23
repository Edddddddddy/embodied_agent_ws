# GTSAM 扫描重叠门控多序列验证

- 独立固定图：2 条
- 公平性检查：**PASS**
- 默认启用决策：**keep_disabled_collect_more_sequences**
- 双证据 ATE 平均变化：-2.92%

| sequence | nodes | constraints | baseline ATE | innovation ATE | dual ATE | dual ATE change | dual P95 change | naive reject | dual reject |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| corridor1-1 | 1834 | 2751 | 1.2236 | 1.1713 | 1.1521 | -5.84% | -8.27% | 269 | 11 |
| corridor1-2 | 488 | 491 | 0.1484 | 0.1484 | 0.1484 | +0.00% | +0.00% | 0 | 0 |

> 决策规则：Enable only when every independent fixed graph improves ATE, no sequence regresses ATE P95 by more than 2%, and dual evidence rejects fewer edges than the naive overlap-only gate.

> 证据边界：该报告比较 Karto 已接受约束的独立固定图，真值只用于离线轨迹评分；运行时
> 扫描重叠和创新量不读取真值。它不能替代候选级回环 precision/recall 评测。
