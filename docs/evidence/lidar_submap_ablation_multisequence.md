# OpenLORIS scan-to-submap A/B

- A/B 数据契约：**PASS**
- 发布决策：**shadow_only_submap_quality_insufficient**
- 平均 precision 变化：+2.32%
- 平均 conditional recall 变化：-1.72%

| sequence | scan precision | submap precision | Δ precision | scan recall | submap recall | Δ median trans. error |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| corridor1-1 | 8.41% | 10.34% | +1.94% | 26.67% | 20.00% | -0.264 m |
| corridor1-2 | 33.78% | 36.49% | +2.70% | 40.32% | 43.55% | -0.612 m |

> 结论边界：相对改善不等于可上线；所有结果仍为 shadow evidence，图边写入关闭。
