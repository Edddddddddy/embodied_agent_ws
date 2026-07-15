# corridor1-2 LiDAR shadow scan matching

- 数据契约：**PASS**
- 已评分 pair：1075
- 真回环候选 pair：62

| Profile | Accepted | Precision | Conditional pair recall | Query recall | Median translation error |
| --- | ---: | ---: | ---: | ---: | ---: |
| cpp_default | 74 | 36.49% | 43.55% | 46.88% | 0.824 m |
| balanced_shadow | 3 | 66.67% | 3.23% | 6.25% | 0.329 m |
| conservative_shadow | 0 | 0.00% | 0.00% | 0.00% | n/a |
| cpp_ranked_single | 40 | 25.00% | 16.13% | 31.25% | 0.459 m |
| cpp_temporal | 8 | 50.00% | 6.45% | 12.50% | 0.437 m |

- balanced shadow 事件恢复：1/2

> 边界：配准结果只在 shadow 报告中评分，没有向 Ceres/GTSAM 位姿图写入边；真值仅用于离线标注。
