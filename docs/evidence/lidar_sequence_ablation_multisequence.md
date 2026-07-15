# Multi-sequence LiDAR multi-hypothesis sequence ablation

- 证据门禁：**PASS**
- 固定参数：`{"mode": "multi_hypothesis", "minimum_confirmations": 3, "maximum_query_gap_s": 2.0, "maximum_pair_age_delta_s": 0.25, "maximum_translation_delta_m": 0.35, "maximum_yaw_delta_rad": 0.2, "maximum_hypotheses": 64}`

| Sequence | Ranked precision | Single-track precision | Multi-hypothesis precision |
| --- | ---: | ---: | ---: |
| corridor1-1 | 9.24% | 12.50% | 33.33% |
| corridor1-2 | 25.00% | 50.00% | 60.00% |

- 聚合精度：31.25% → 45.45%
- 假接受：11 → 6
- 保留真约束：5 → 5

> 边界：仍为 shadow-only；低召回意味着尚不能开放无人值守图边写入。
