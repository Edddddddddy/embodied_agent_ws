# Multi-sequence LiDAR temporal-consistency ablation

- 数据与策略门禁：**PASS**
- 固定参数：`{"minimum_confirmations": 4, "maximum_query_gap_s": 2.0, "maximum_pair_age_delta_s": 1.25, "maximum_translation_delta_m": 0.55, "maximum_yaw_delta_rad": 0.35}`

| Sequence | Ranked precision | Temporal precision | Precision delta | Recall delta |
| --- | ---: | ---: | ---: | ---: |
| corridor1-1 | 9.24% | 12.50% | +3.26% | -9.52% |
| corridor1-2 | 25.00% | 50.00% | +25.00% | -9.68% |

- 聚合精度：13.21% → 31.25%
- 聚合条件召回：12.57% → 2.99%

> 边界：这是 shadow-only 精度/召回权衡证据；尚未证明写入 Karto 后地图改善。
